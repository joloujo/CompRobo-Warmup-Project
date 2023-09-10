import tty
import select
import sys
import termios

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

from collections import defaultdict

import rclpy
from rclpy.node import Node
from pynput import keyboard



class Teleop(Node):
    settings = termios.tcgetattr(sys.stdin)
    key_states: defaultdict[str, bool] = defaultdict(lambda: False)
    key = None



    def on_press(self, key):
        if key == keyboard.Key.esc:
            self.key_states["esc"] = True
        try:
            print(f'alphanumeric key {key.char} pressed')
            self.key_states[key.char] = True
        except AttributeError:
            print(f'special key {key} pressed')

    def on_release(self, key):
        try:
            print(f'alphanumeric key {key.char} released')
            self.key_states[key.char] = False
        except:
            print(f'special key {key} released')
        

    def __init__(self):
        super().__init__('teleop') # type: ignore
        tty.setraw(sys.stdin.fileno())
        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        listener.start()
        self.neato_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_timer(0.01, self.run_loop)
    

    # def get_key(self) -> str:
    #     
    #     select.select([sys.stdin], [], [], 0)
    #     key = sys.stdin.read(1)
    #     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    #     return key
    
    def run_loop(self):
        fwd_vel = 0.0
        rot_vel = 0.0
        if self.key_states["esc"]:
            exit()
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
    node = Teleop()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


