import rclpy
from rclpy.node import Node, Publisher
from threading import Thread
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool
from time import sleep
from math import pi

class DriveSquare(Node):
    lin_speed = 0.2
    ang_speed = 1.5

    def __init__(self) -> None:
        super().__init__('drive_square') # type: ignore
        self.neato_pub: Publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.loop_thread = Thread(target=self.loop)
        self.loop_thread.start()

    def drive(self, linear: float, angular: float):
        msg: Twist = Twist()
        msg.linear = Vector3(x=linear, y=0.0, z=0.0)
        msg.angular = Vector3(x=0.0, y=0.0, z=angular)
        self.neato_pub.publish(msg)

    def loop(self) -> None:
        print("driving in a square")
        self.drive(0.0, 0.0)
        sleep(0.1)
        for i in range(0, 4):
            self.drive_forward()
            self.turn()
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

def main(args=None):
    rclpy.init(args=args)
    node: Node = DriveSquare()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()