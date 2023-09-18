import rclpy
from rclpy.node import Node, Publisher
from rclpy.action import ActionServer
from threading import Thread
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool
from time import sleep
from math import pi

from action_tutorials_interfaces.action import Drivesquare # type: ignore

class DriveSquare(Node):
    lin_speed = 0.2
    ang_speed = 0.5

    def __init__(self) -> None:
        super().__init__('drive_square') # type: ignore
        self.neato_pub: Publisher = self.create_publisher(Twist, "cmd_vel", 10)

        self._action_server = ActionServer(
            self,
            Drivesquare,
            'drive_square',
            self.execute_callback)
        
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        self.drive_square()
        goal_handle.succeed()
        result = Drivesquare.Result()
        return result

    def drive(self, linear: float, angular: float):
        msg: Twist = Twist()
        msg.linear = Vector3(x=linear, y=0.0, z=0.0)
        msg.angular = Vector3(x=0.0, y=0.0, z=angular)
        self.neato_pub.publish(msg)

    def drive_square(self) -> None:
        print("driving in a square")
        self.drive(0.0, 0.0)
        sleep(0.1)
        for i in range(0, 4):
            self.drive_forward()
            self.pause()
            self.turn()
            self.pause()
        self.drive(0.0, 0.0)
        print("done")
    
    def drive_forward(self):
        print("driving forward")
        self.drive(self.lin_speed, 0.0)
        sleep(1 / self.lin_speed)
    
    def turn(self):
        print("turning")
        self.drive(0.0, self.ang_speed)
        sleep((pi/2) / self.ang_speed)

    def pause(self):
        self.drive(0.0, 0.0)
        sleep(0.2)

def main(args=None):
    rclpy.init(args=args)
    node: Node = DriveSquare()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()