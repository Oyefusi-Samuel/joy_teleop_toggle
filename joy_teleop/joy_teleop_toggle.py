import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import subprocess
import time
import signal

class JoyTeleopToggle(Node):
    def __init__(self):
        super().__init__('joy_teleop_toggle')
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.teleop_enabled = False
        self.linear_velocity = 0.5
        self.angular_velocity = 1.0

        self.last_toggle_time = 0
        self.last_adjust_time = 0
        self.debounce_delay = 0.3

    def joy_callback(self, msg):
        toggle_button = 1     # Circle button to toggle teleop
        increase_linear = 7   # R2
        decrease_linear = 6   # L2
        increase_angular = 5  # R1
        decrease_angular = 4  # L1

        now = time.time()

        # Toggle teleop mode with debounce
        if msg.buttons[toggle_button] == 1 and (now - self.last_toggle_time) > self.debounce_delay:
            self.teleop_enabled = not self.teleop_enabled
            self.get_logger().info(f'Teleop enabled: {self.teleop_enabled}')
            self.last_toggle_time = now

        if not self.teleop_enabled:
            # Teleop disabled => stop robot
            self.cmd_pub.publish(Twist())
            return

        # Adjust velocities with debounce
        if (now - self.last_adjust_time) > self.debounce_delay:
            if msg.buttons[increase_linear] == 1:
                self.linear_velocity = min(self.linear_velocity + 0.05, 2.0)
                self.get_logger().info(f'Increased linear velocity to {self.linear_velocity:.2f}')
                self.last_adjust_time = now
            if msg.buttons[decrease_linear] == 1:
                self.linear_velocity = max(self.linear_velocity - 0.05, 0.0)
                self.get_logger().info(f'Decreased linear velocity to {self.linear_velocity:.2f}')
                self.last_adjust_time = now
            if msg.buttons[increase_angular] == 1:
                self.angular_velocity = min(self.angular_velocity + 0.05, 3.0)
                self.get_logger().info(f'Increased angular velocity to {self.angular_velocity:.2f}')
                self.last_adjust_time = now
            if msg.buttons[decrease_angular] == 1:
                self.angular_velocity = max(self.angular_velocity - 0.05, 0.0)
                self.get_logger().info(f'Decreased angular velocity to {self.angular_velocity:.2f}')
                self.last_adjust_time = now

        # Always publish twist based on joystick axes and current velocities
        twist = Twist()
        twist.linear.x = msg.axes[1] * self.linear_velocity   # Left stick vertical
        twist.angular.z = msg.axes[3] * self.angular_velocity # Right stick horizontal

        self.cmd_pub.publish(twist)


def main(args=None):
    joy_process = subprocess.Popen(['ros2', 'run', 'joy', 'joy_node'])

    rclpy.init(args=args)
    node = JoyTeleopToggle()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

    joy_process.send_signal(signal.SIGINT)
    joy_process.wait()


if __name__ == '__main__':
    main()
