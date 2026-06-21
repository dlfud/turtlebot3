import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy  # ✅ QoS 추가
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from scipy.ndimage import label, binary_dilation
import numpy as np


class CoveragePlanner(Node):

    def __init__(self):
        super().__init__('coverage_planner')

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos     # ← map_server와 동일한 QoS
        )

        # ✅ transient_local: 늦게 구독해도 마지막 메시지를 받을 수 있음
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.path_pub = self.create_publisher(
            Path,
            '/coverage_path',
            latched_qos
        )

        self.done = False

    def map_callback(self, msg):

        if self.done:
            return

        self.done = True

        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution

        grid = np.array(msg.data).reshape(height, width)

        obstacle = (grid == 100)
        inflated_obstacle = binary_dilation(obstacle, iterations=8)
        free = np.logical_and(grid == 0, np.logical_not(inflated_obstacle))

        labeled, num = label(free)

        largest = 0
        largest_size = 0
        for i in range(1, num + 1):
            size = np.sum(labeled == i)
            if size > largest_size:
                largest_size = size
                largest = i

        grid = np.where(labeled == largest, 0, 100)

        self.get_logger().info(f"Map size {width} x {height}")

        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()  # ✅ timestamp 추가

        row_step = 10   # ✅ 행 간격과 열 간격 분리
        col_step = 10

        for y in range(0, height, row_step):

            free_line = (grid[y] == 0)
            segments = []
            start = None

            for x in range(width):
                if free_line[x] and start is None:
                    start = x
                elif not free_line[x] and start is not None:
                    segments.append((start, x - 1))
                    start = None

            if start is not None:
                segments.append((start, width - 1))

            if (y // row_step) % 2 == 1:
                segments.reverse()

            for (x_start, x_end) in segments:

                if (y // row_step) % 2 == 0:
                    xs = range(x_start, x_end + 1, col_step)
                else:
                    xs = range(x_end, x_start - 1, -col_step)

                for x in xs:
                    pose = PoseStamped()
                    pose.header.frame_id = "map"
                    pose.pose.position.x = x * resolution + msg.info.origin.position.x
                    pose.pose.position.y = y * resolution + msg.info.origin.position.y
                    pose.pose.orientation.w = 1.0
                    path.poses.append(pose)

        self.path_pub.publish(path)
        self.get_logger().info(f"Generated {len(path.poses)} waypoints")


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()