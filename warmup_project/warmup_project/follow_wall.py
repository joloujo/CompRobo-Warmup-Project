import rclpy
from rclpy.node import Node, Publisher
from geometry_msgs.msg import Vector3, Point, Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np

class FollowWall(Node):
    forward_distance = 0.0
    backward_distance = 0.0

    def __init__(self) -> None:
        super().__init__('follow_wall') # type: ignore
        self.neato_pub: Publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.marker_pub: Publisher = self.create_publisher(MarkerArray, "wall_viz", 10)
        self.scan_sub = self.create_subscription(LaserScan, "stable_scan", self.scan_callback, 10)
        self.create_timer(0.01, self.loop)

    def scan_callback(self, scan: LaserScan):
        self.forward_distance: float = scan.ranges[45]
        self.backward_distance: float = scan.ranges[135]

    def loop(self) -> None:
        forward_marker = Marker()
        forward_marker.header.frame_id = "base_link"
        forward_marker.id = 0
        forward_marker.type = Marker.SPHERE
        forward_marker.action = Marker.MODIFY
        forward_marker.color.a = 1.0
        forward_marker.color.b = 255.0
        forward_marker.scale.x = 0.2
        forward_marker.scale.y = 0.2
        forward_marker.scale.z = 0.2
        forward_marker.pose.position = self.pol2cart(self.forward_distance, 45)
        backward_marker = Marker()
        backward_marker.header.frame_id = "base_link"
        backward_marker.id = 1
        backward_marker.type = Marker.SPHERE
        backward_marker.action = Marker.MODIFY
        backward_marker.color.a = 1.0
        backward_marker.color.b = 255.0
        backward_marker.scale.x = 0.2
        backward_marker.scale.y = 0.2
        backward_marker.scale.z = 0.2
        backward_marker.pose.position = self.pol2cart(self.backward_distance, 135)
        markers: MarkerArray = MarkerArray(markers=[forward_marker, backward_marker])
        angular_speed = (self.forward_distance - self.backward_distance) / 2
        if angular_speed > 0.5:
            angular_speed = 0.5
        elif angular_speed < -0.5:
            angular_speed = -0.5
        elif angular_speed < 0.1:
            angular_speed = 0.0
        elif np.isnan(angular_speed):
            print("was nan")
            angular_speed = 0.0
        
        print(angular_speed)
        msg: Twist = Twist(angular=Vector3(x=0.0, y=0.0, z=angular_speed), linear=Vector3(x=0.1, y=0.0, z=0.0))
        self.neato_pub.publish(msg)
        self.marker_pub.publish(markers)

        
    def pol2cart(self, rho, phi) -> Point:
        x = rho * np.cos(np.deg2rad(phi))
        y = rho * np.sin(np.deg2rad(phi))
        return(Point(x=x, y=y, z=0.0))


def main(args=None):
    rclpy.init(args=args)
    node = FollowWall()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()