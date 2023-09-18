import rclpy
from rclpy.node import Node, Publisher
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovariance, Pose, Quaternion
import numpy as np
from math import acos
from time import sleep


class ObstacleAvoider(Node):
    state = 0
    forward_range = 0.0
    left_range = 0.0
    last_orientation: Quaternion = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    latest_pose: PoseWithCovariance = PoseWithCovariance()

    def __init__(self) -> None:
        super().__init__('obstacle_avoider') # type: ignore
        self.neato_pub: Publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan_sub = self.create_subscription(LaserScan, "stable_scan", self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.create_timer(0.01, self.loop)

    def scan_callback(self, scan: LaserScan):
        self.forward_range = scan.ranges[0]
        self.left_range = scan.ranges[90]

    def odom_callback(self, pose: Odometry):
        self.latest_pose = pose.pose
        #print(f'pose callback called, pose: {pose.pose}')

    def loop(self) -> None:
        fwd_vel = 0.0
        rot_vel = 0.0
        if self.state == 0:
            #move forward
            fwd_vel = 0.5
            if self.forward_range < 1.0 and self.forward_range != 0.0:
                self.last_orientation = self.latest_pose.pose.orientation
                self.state = 1
                print(f"switching to state 1 with angle: {np.rad2deg(acos(self.last_orientation.w)*2)}")

        if self.state == 1:
            #turn right 90 deg
            current_w = self.latest_pose.pose.orientation.w
            print(f"in state 1 with current angle: {np.rad2deg(acos(current_w) * 2)}")
            last_w = self.last_orientation.w
            current_ang = np.rad2deg(acos(current_w) * 2)
            last_ang = np.rad2deg(acos(last_w) * 2)
            if current_ang - last_ang >= 90:
                self.state = 2
                self.last_orientation = self.latest_pose.pose.orientation
            else:
                rot_vel = -0.5
        if self.state == 2:

            #drive until obstacle is no longer present
            if self.left_range >= 2.0 and self.left_range != 0.0:
                self.state = 3
                self.last_orientation = self.latest_pose.pose.orientation
            else:
                fwd_vel = 0.5
        if self.state == 3:
            #turn left again
            current_w = self.latest_pose.pose.orientation.w
            last_w = self.last_orientation.w
            current_ang = np.rad2deg(acos(current_w) * 2)
            last_ang = np.rad2deg(acos(last_w) * 2)
            if current_ang - last_ang <= -90:
                self.state = 0
                self.last_orientation = self.latest_pose.pose.orientation
            else:
                rot_vel = 0.5
    
        msg: Twist = Twist(angular=Vector3(x=0.0, y=0.0, z=rot_vel), linear=Vector3(x=fwd_vel, y=0.0, z=0.0))
        self.neato_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    sleep(1)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()