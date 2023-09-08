import rclpy
from rclpy.node import Node, Publisher
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class DriveTest(Node):
    def __init__(self) -> None:
        super().__init__('DriveTest') # type: ignore
        self.neato_pub: Publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_timer(0.05, self.loop)

    def loop(self) -> None:
        print("among us 2: electric boogaloo")
        msg: Twist = Twist()
        msg.angular = Vector3(x=0.0, y=0.0, z=0.0)
        forward_vel = 0.1
        msg.linear = Vector3(x=forward_vel, y=0.0, z=0.0)
        self.neato_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node: Node = DriveTest()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()