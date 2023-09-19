import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray, Pose, PointStamped
from sensor_msgs.msg import LaserScan
import numpy as np

import tty
import select
import sys
import termios
from geometry_msgs.msg import Vector3
from collections import defaultdict
from pynput import keyboard

class FSM(Node):
    state = "follow_person" 

    detection_width = 1
    detection_min_dist = 1.5 * -1
    detection_max_dist = -0.2

    max_lin_speed = 0.2
    max_ang_speed = 0.5

    settings = termios.tcgetattr(sys.stdin)
    key_states: defaultdict[str, bool] = defaultdict(lambda: False)
    key = None

    def __init__(self):
        super().__init__('finite_state_machine') # type: ignore
        # create publisher to control the neato
        self.neato_pub = self.create_publisher(Twist, "cmd_vel", 10)
        # create publisher to publish point cloud
        self.point_cloud_pub = self.create_publisher(PoseArray, "point_cloud", 10)
        # create publisher to publish center of mass
        self.COM_pub = self.create_publisher(PointStamped, "center_of_mass", 10)
        # create subscriber to get laser scan data
        self.scan_sub = self.create_subscription(LaserScan, "stable_scan", self.scan_callback, 10)

        tty.setraw(sys.stdin.fileno())
        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        listener.start()
        self.neato_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_timer(0.01, self.run_loop)
        
    def scan_callback(self, scan):
        if self.state != "follow_person":
            return

        points_x, points_y = self.process_scan(scan)
        self.publish_point_cloud(points_x, points_y)

        # ---------- Calculate center of mass ----------
        # if no points are in the detection box, stop the neato
        if len(points_x) == 0:
            self.drive_neato(0.0, 0.0)
            return
        
        # if there are points in the detection box, drive the neato towards the center of mass
        center_x = np.mean(points_x)
        center_y = np.mean(points_y)

        self.publish_COM(center_x, center_y)

        self.control(center_x, center_y)

    def process_scan(self, scan):
        # get raw data into numpy arrays
        ranges_raw = np.array(scan.ranges)
        angles_raw = np.arange(len(ranges_raw)) * scan.angle_increment + scan.angle_min

        # create filter of missing points and points at the lidar
        good_points = np.logical_and(ranges_raw != 0, ranges_raw != np.inf)

        # apply data cleaning filter
        ranges = ranges_raw[good_points]
        angles =angles_raw[good_points]

        # convert from polar to cartesian coordinates
        points_x = ranges * np.cos(angles)
        points_y = ranges * np.sin(angles)
        
        # DEBUG print cartesian points
        # print(points_x, points_y)

        # create filter of the detection box
        within_x_range = np.logical_and(self.detection_min_dist <= points_x, points_x <= self.detection_max_dist)
        within_y_range = np.logical_and(-1*self.detection_width/2 <= points_y, points_y <= self.detection_width/2)
        filter = np.logical_and(within_x_range, within_y_range)

        # DEBUG print filter logic
        # print(-1*self.detection_width/2, self.detection_width/2, self.detection_min_dist, self.detection_max_dist)
        # print(within_x_range, within_y_range, filter)

        # apply detection box filter
        filtered_points_x = points_x[filter]
        filtered_points_y = points_y[filter]

        return filtered_points_x, filtered_points_y

    def publish_point_cloud(self, points_x, points_y):
        # create message
        msg = PoseArray()
        msg.header.frame_id = "base_laser_link"

        # add points to message
        for i in range(len(points_x)-1):
            # create pose
            pose = Pose()
            pose.position.x = points_x[i]
            pose.position.y = points_y[i]

            # DEBUG print pose
            # print(points_x[i], pose.position.y)

            # add pose to message
            msg.poses.append(pose) # type: ignore
        
        # publish message
        self.point_cloud_pub.publish(msg=msg)

    def publish_COM(self, center_x, center_y):
        # create message
        msg = PointStamped()
        msg.header.frame_id = "base_laser_link"

        # add center of mass to message
        msg.point.x = center_x
        msg.point.y = center_y

        # DEBUG print message
        # print(msg)

        # publish message
        self.COM_pub.publish(msg=msg)

    def control(self, target_x, target_y):
        # calculate distance and angle to target
        distance = np.sqrt(target_x**2 + target_y**2)
        angle = np.arctan2(target_y, target_x)

        command_linear = np.clip((distance-0.4)/2, -self.max_lin_speed, self.max_lin_speed)
        command_angular = np.clip(angle * -0.5, -self.max_ang_speed, self.max_ang_speed)

        # drive towards target
        self.drive_neato(linear=command_linear, angular=command_angular)

    def drive_neato(self, linear: float, angular: float):
        # create message
        msg = Twist()

        # add speed and angle to message
        msg.linear.x = linear
        msg.angular.z = angular

        # publish message
        self.neato_pub.publish(msg=msg)

    def on_press(self, key):
        if key == keyboard.Key.esc:
            self.key_states["esc"] = True
        try:
            # print(f'alphanumeric key {key.char} pressed')
            self.key_states[key.char] = True

            if key.char == "r":
                if self.state == "teleop":
                    self.state = "follow_person"
                else:
                    self.state = "teleop"
                print(self.state)
        except AttributeError:
            # print(f'special key {key} pressed')
            pass

    def on_release(self, key):
        try:
            # print(f'alphanumeric key {key.char} released')
            self.key_states[key.char] = False
        except:
            # print(f'special key {key} released')
            pass

    def run_loop(self):
        if self.state == "teleop":
            fwd_vel = 0.0
            rot_vel = 0.0
            if self.key_states["esc"]:
                # exit()
                pass
            if self.key_states["w"]:
                fwd_vel = 0.5
            if self.key_states["s"]:
                fwd_vel = -0.5
            if self.key_states["a"]:
                rot_vel = 1.0
            if self.key_states["d"]:
                rot_vel = -1.0
            self.neato_pub.publish(Twist(linear=Vector3(x=fwd_vel,y=0.0,z=0.0), angular=Vector3(x=0.0,y=0.0,z=rot_vel)))

def main(args=None):
    rclpy.init(args=args)
    node = FSM()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()