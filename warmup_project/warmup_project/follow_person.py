import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray, Pose, PointStamped
from sensor_msgs.msg import LaserScan, PointCloud2
import numpy as np

class FollowPerson(Node):
    detection_width = 1
    detection_min_dist = -1
    detection_max_dist = 0

    def __init__(self):
        super().__init__('follow_person') # type: ignore
        self.neato_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan_sub = self.create_subscription(LaserScan, "stable_scan", self.scan_callback, 10)
        self.create_timer(0.01, self.loop)

        self.point_cloud_pub = self.create_publisher(PoseArray, "point_cloud", 10)
        self.COM_pub = self.create_publisher(PointStamped, "center_of_mass", 10)

    def scan_callback(self, scan):
        points_x, points_y = self.process_scan(scan)
        self.publish_point_cloud(points_x, points_y)

        # ---------- Calculate center of mass ----------
        center_x = np.mean(points_x)
        center_y = np.mean(points_y)

        # DEBUG print center of mass
        # print(center_x, center_y)

        # create message
        msg = PointStamped()
        msg.header.frame_id = "base_laser_link"

        # add center of mass to message
        msg.point.x = center_x
        msg.point.y = center_y

        print(msg)

        # publish message
        self.COM_pub.publish(msg=msg)

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
        msg = Pose()

        # add center of mass to message
        msg.position.x = center_x
        msg.position.y = center_y

        # publish message
        self.COM_pub.publish(msg=msg)


    def loop(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = FollowPerson()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()