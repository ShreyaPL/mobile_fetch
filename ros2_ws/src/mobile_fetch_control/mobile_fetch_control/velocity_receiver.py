import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Velocityreceiver(Node):
    def __init__(self):
        super().__init__("velocity_receiver")

        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10,
        )

        self.get_logger().info("Velocity receiver is ready.")

    def cmd_vel_callback(self, msg):
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        self.get_logger().info(
            f"Received: forward={linear_speed: .2f} m/s, "
            f"turn={angular_speed: .2f} rad/s"
        )

def main(args=None):
    rclpy.init(args=args)
    node = Velocityreceiver()

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