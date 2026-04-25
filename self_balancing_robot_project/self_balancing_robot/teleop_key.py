import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios

INSTRUCTIONS = """
╔══════════════════════════════════════════╗
║   Self Balancing Robot - Keyboard Control ║
╠══════════════════════════════════════════╣
║   W : Move Forward                        ║
║   S : Move Backward                       ║
║   A : Turn Left                           ║
║   D : Turn Right                          ║
║   SPACE : Stop                            ║
║   Q : Quit                                ║
╚══════════════════════════════════════════╝
"""


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher = self.create_publisher(Twist, '/target_vel', 10)
        self.get_logger().info('Keyboard teleop started!')
        print(INSTRUCTIONS)

    def get_key(self):
        """Read a single keypress from terminal"""
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return key

    def run(self):
        """Main loop - listen for keypresses and publish commands"""
        while rclpy.ok():
            key = self.get_key()
            msg = Twist()

            if key == 'w' or key == 'W':
                msg.linear.x = 1.0
                print('▶ Moving FORWARD')
            elif key == 's' or key == 'S':
                msg.linear.x = -1.0
                print('◀ Moving BACKWARD')
            elif key == 'a' or key == 'A':
                msg.angular.z = 0.5
                print('↺ Turning LEFT')
            elif key == 'd' or key == 'D':
                msg.angular.z = -0.5
                print('↻ Turning RIGHT')
            elif key == ' ':
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                print('■ STOP')
            elif key == 'q' or key == 'Q':
                print('Quitting...')
                break

            self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
