import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

# Updated Key bindings for moving
moveBindings = {
    'w': (-1, 0, 0, 0),   # Forward
    's': (1, 0, 0, 0),  # Backward
    #'a': (0, 0, 0, 1),   # Turn Left
    #'d': (0, 0, 0, -1),  # Turn Right
    'q': (1, 0, 0, 1),   # Forward-Left Diagonal
    'e': (1, 0, 0, -1),  # Forward-Right Diagonal
    'z': (-1, 0, 0, 1),  # Backward-Left Diagonal
    'c': (-1, 0, 0, -1),  # Backward-Right Diagonal
    'x': (0, 0, 0, 0)  # Stop
}

class TeleopTwistKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_twist_keyboard')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.speed = 1.0  # Linear speed
        self.turn = 1.0   # Angular speed

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        self.get_logger().info("Use 'w', 's', 'a', 'd', 'q', 'e', 'z', 'c' to control the robot. CTRL+C to exit.")
        try:
            while True:
                key = self.getKey()
                if key in moveBindings.keys():
                    x, y, z, th = moveBindings[key]
                    self.get_logger().info(f"Key pressed: {key}. Moving with x: {x}, th: {th}")
                else:
                    x, y, z, th = 0, 0, 0, 0
                    if key == '\x03':  # CTRL+C
                        break

                twist = Twist()
                twist.linear.x = x * self.speed
                twist.linear.y = y * self.speed
                twist.linear.z = z * self.speed
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = th * self.turn
                self.pub.publish(twist)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopTwistKeyboard()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
