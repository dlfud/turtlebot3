import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
import time

class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.original_waypoints = []
        self.waypoints          = []
        self.current_idx        = 0

        # object 처리용 별도 경로 [object_pos, home_pos]
        self.object_waypoints   = []
        self.object_idx         = 0

        self.is_running         = False
        self.object_found       = False
        self.home_x             = None
        self.home_y             = None
        self.current_handle     = None

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.create_subscription(Path, '/coverage_path', self.path_callback, latched_qos)
        self.create_subscription(PoseStamped, '/object_pose', self.object_callback, latched_qos)
        self.get_logger().info('AutoNav Ready.')

    def path_callback(self, msg):
        if self.is_running:
            self.get_logger().warn('Already navigating, ignoring new path')
            return

        self.original_waypoints = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.waypoints = list(self.original_waypoints)
        self.home_x = self.waypoints[-1][0]
        self.home_y = self.waypoints[-1][1]

        self.current_idx = 0
        self.is_running = True
        self.get_logger().info(f'Received {len(self.waypoints)} waypoints')
        self.send_next_goal()

    def object_callback(self, msg):
        if self.object_found:
            return

        self.object_found = True
        obj_x = msg.pose.position.x
        obj_y = msg.pose.position.y
        self.get_logger().info(f'🎯 Object detected! Heading to object then home...')

        # object 전용 경로: [object위치, home위치]
        # current_idx는 건드리지 않음 → 나중에 이 지점부터 재개
        self.object_waypoints = [(obj_x, obj_y), (self.home_x, self.home_y)]
        self.object_idx = 0
        self.current_idx -= 1

        if self.current_handle is not None:
            self.current_handle.cancel_goal_async()

    def send_next_goal(self):
        if self.object_found:
            # object 처리 경로 주행
            if self.object_idx >= len(self.object_waypoints):
                # home까지 도착 완료 → 원래 경로 current_idx부터 재개
                self.get_logger().info(f'✅ Object handling done. Resuming patrol from waypoint [{self.current_idx}/{len(self.waypoints)}]')
                self.object_found = False
                self.object_waypoints = []
                self.object_idx = 0
                # current_idx는 그대로 유지 → 아래 정상 경로 로직으로 이어짐
            else:
                x, y = self.object_waypoints[self.object_idx]
                label = '[OBJECT]' if self.object_idx == 0 else '[HOME]'
                self.get_logger().info(f'Navigating to {label} ({x:.2f}, {y:.2f})')
                self.send_goal(x, y)
                return

        # 정상 순찰 경로
        if self.current_idx >= len(self.waypoints):
            self.get_logger().info('🔄 [LOOP] Arrived at HOME. Restarting patrol!')
            self.current_idx = 0
            self.waypoints = list(self.original_waypoints)
            time.sleep(1.0)

        x, y = self.waypoints[self.current_idx]
        total = len(self.waypoints)
        label = '[HOME]' if self.current_idx == total - 1 else f'[{self.current_idx + 1}/{total}]'
        self.get_logger().info(f'Navigating to {label} ({x:.2f}, {y:.2f})')
        self.send_goal(x, y)

    def send_goal(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0

        goal_msg      = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected! Skipping.')
            if self.object_found:
                self.object_idx += 1
            else:
                self.current_idx += 1
            self.send_next_goal()
            return

        self.current_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        if self.object_found:
            x, y = self.object_waypoints[self.object_idx]
            self.get_logger().info(f'✅ Reached object waypoint ({x:.2f}, {y:.2f})')
            # home이 아닐 때만 회전 (object 위치에서만)
            if self.object_idx < len(self.object_waypoints) - 1:
                self.rotate_one_turn()
            self.object_idx += 1
        else:
            x, y = self.waypoints[self.current_idx]
            self.get_logger().info(f'✅ Reached ({x:.2f}, {y:.2f})')
            if self.current_idx < len(self.waypoints) - 1:
                self.rotate_one_turn()
            self.current_idx += 1

        self.send_next_goal()

    def rotate_one_turn(self):
        self.get_logger().info('🔄 Rotating 360 degrees...')
        twist_msg = Twist()
        twist_msg.angular.z = 0.8
        start_time = time.time()
        while time.time() - start_time < 7.85:
            self.cmd_vel_pub.publish(twist_msg)
            time.sleep(0.1)
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info('🔄 Rotation complete.')

    def feedback_callback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'  Distance remaining: {dist:.2f}m', throttle_duration_sec=3.0)


def main(args=None):
    rclpy.init(args=args)
    node = AutoNav()
    rclpy.spin(node)

if __name__ == '__main__':
    main()