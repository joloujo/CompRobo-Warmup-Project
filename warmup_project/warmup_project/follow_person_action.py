import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray, Pose, PointStamped
from sensor_msgs.msg import LaserScan
import numpy as np
from rclpy.action import ActionServer
from time import sleep
from threading import Thread

from action_tutorials_interfaces.action import Followperson # type: ignore

class FollowPerson(Node):
    detection_width = 1
    detection_min_dist = 1.5 * -1
    detection_max_dist = -0.2

    max_lin_speed = 0.2
    max_ang_speed = 0.5
    
    scan = None
    new_scan = False

    def __init__(self):
        super().__init__('follow_person') # type: ignore
        # create publisher to control the neato
        self.neato_pub = self.create_publisher(Twist, "cmd_vel", 10)
        # create publisher to publish point cloud
        self.point_cloud_pub = self.create_publisher(PoseArray, "point_cloud", 10)
        # create publisher to publish center of mass
        self.COM_pub = self.create_publisher(PointStamped, "center_of_mass", 10)
        # create subscriber to get laser scan data
        self.scan_sub = self.create_subscription(LaserScan, "stable_scan", self.scan_callback, 10)

        self._action_server = ActionServer(
            self,
            Followperson,
            'follow_person_action',
            self.loop)
        
    def loop_thread(self, goal_handle):
        t = Thread(target=self.loop, args=[goal_handle])
        t.start()
        
    def scan_callback(self, scan):
        self.scan = scan
        self.new_scan = True
        print("new scan")
        return

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

    def loop(self, goal_handle):
        self.get_logger().info('Executing goal...')

        while True:
            while not self.new_scan:
                print("waiting")
                sleep(0.2)
                pass
            
            print("executing goal")

            self.new_scan = False

            points_x, points_y = self.process_scan(self.scan)
            self.publish_point_cloud(points_x, points_y)

            # ---------- Calculate center of mass ----------
            if len(points_x) == 0:
                # if no points are in the detection box, stop the neato
                self.drive_neato(0.0, 0.0)
            else:
                # if there are points in the detection box, drive the neato towards the center of mass
                center_x = np.mean(points_x)
                center_y = np.mean(points_y)

                self.publish_COM(center_x, center_y)

                self.control(center_x, center_y)

def main(args=None):
    rclpy.init(args=args)
    node = FollowPerson()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()