import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock

import time
import math

class VelocityReceiver(Node):
    def __init__(self):
        super().__init__("velocity_receiver")

        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.last_command_time = None
        self.command_timeout = 0.5  #seconds

        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            1,
        )

        # Clock Publisher
        self.clock_publisher = self.create_publisher(
            Clock,
            "/clock",
            10,
        )

        self.get_logger().info("Velocity receiver is ready.")

    def cmd_vel_callback(self, msg):
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        if not (
            math.isfinite(linear_speed)
            and math.isfinite(angular_speed)
        ):
            self.linear_speed = 0.0
            self.angular_speed = 0.0
            self.last_command_time = None
            self.get_logger().warning("Invalid velocity commands; stopping.")

        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.last_command_time = time.monotonic()
        self.get_logger().info(
            f"Received: forward={linear_speed: .2f} m/s, "
            f"turn={angular_speed: .2f} rad/s"
        )

    def get_velocity_command(self):
        if self.last_command_time is None:
            return 0.0, 0.0

        command_age = time.monotonic() - self.last_command_time
        if command_age > self.command_timeout:
            return 0.0, 0.0

        return self.linear_speed, self.angular_speed

    def publish_sim_time(self, simulation_time):
        total_nanoseconds = round(simulation_time * 1_000_000_000)

        msg = Clock()
        msg.clock.sec = total_nanoseconds // 1_000_000_000
        msg.clock.nanosec = total_nanoseconds % 1_000_000_000
        self.clock_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityReceiver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()