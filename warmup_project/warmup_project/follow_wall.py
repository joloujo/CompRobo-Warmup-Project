import rclpy
from rclpy.node import Node, Publisher
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan


class FollowWall(Node):
    def __init__(self) -> None:
        super().__init__('follow_wall') # type: ignore
        self.neato_pub: Publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan_sub = self.create_subscription(LaserScan, "stable_scan", self.scan_callback, 10)
        self.create_timer(0.01, self.loop)

    def scan_callback(self, scan: LaserScan):
        forward_distance: float = scan.ranges[44]
        backward_distance: float = scan.ranges[134]

    def loop(self) -> None:
        msg: Twist = Twist(angular=Vector3(x=0.0, y=0.0, z=0.0), linear=Vector3(x=0.0, y=0.0, z=0.0))
        self.neato_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FollowWall()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()