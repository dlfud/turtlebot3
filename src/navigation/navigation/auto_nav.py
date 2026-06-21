# auto_nav.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy  # ✅ QoS 추가

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException
from nav_msgs.msg import Path


class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')

        self.home_x = None
        self.home_y = None

        self.navigation_started = False
        self.returning_home = False

        self.waypoints = []
        self.current_idx = 0
        self.path_received = False

        # ✅ planner와 동일한 QoS로 구독
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.path_sub = self.create_subscription(
            Path,
            '/coverage_path',
            self.path_callback,
            latched_qos  # ✅ 10 → latched_qos
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.timer = self.create_timer(1.0, self.start_navigation)

        self.get_logger().info("AutoNav Started")

    def path_callback(self, msg):
        self.waypoints = msg.poses
        self.current_idx = 0           # ✅ path 갱신 시 인덱스 리셋
        self.navigation_started = False  # ✅ 새 path이면 네비게이션 재시작 허용
        self.returning_home = False
        self.path_received = True
        self.get_logger().info(f"Received {len(self.waypoints)} waypoints")

    def start_navigation(self):
        if self.navigation_started:
            return

        if len(self.waypoints) == 0:
            self.get_logger().info("Waiting for coverage path...")
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )

            self.home_x = transform.transform.translation.x
            self.home_y = transform.transform.translation.y

            self.get_logger().info(f'Home saved: ({self.home_x:.2f}, {self.home_y:.2f})')

            self.navigation_started = True
            self.send_goal(self.waypoints[0])

        except TransformException:
            self.get_logger().info('Waiting for map->base_link transform...')

    def send_goal(self, pose):
        self.get_logger().info(
            f'Sending goal: {pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}'
        )

        self._action_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f} m')

    def result_callback(self, future):
        self.get_logger().info('Goal completed')

        self.current_idx += 1

        if self.current_idx < len(self.waypoints):
            self.get_logger().info(
                f'Moving to waypoint {self.current_idx + 1}/{len(self.waypoints)}'
            )
            self.send_goal(self.waypoints[self.current_idx])
            return

        if not self.returning_home:
            self.returning_home = True
            self.get_logger().info('Coverage complete. Returning home.')

            home_pose = PoseStamped()
            home_pose.header.frame_id = 'map'
            home_pose.pose.position.x = self.home_x
            home_pose.pose.position.y = self.home_y
            home_pose.pose.orientation.w = 1.0

            self.send_goal(home_pose)
            return

        self.get_logger().info('Arrived home.')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = AutoNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()