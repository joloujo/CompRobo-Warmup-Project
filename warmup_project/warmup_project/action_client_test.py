import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future

from action_tutorials_interfaces.action import Drivesquare # type: ignore

class DriveSquareActionClient(Node):

    def __init__(self):
        super().__init__('drive_square_action_client') # type: ignore
        self._action_client = ActionClient(self, Drivesquare, 'drive_square')

    def send_goal(self):
        goal_msg = Drivesquare.Goal()

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)

    action_client = DriveSquareActionClient()

    future = action_client.send_goal()

    rclpy.spin(action_client) # type: ignore

if __name__ == '__main__':
    main()