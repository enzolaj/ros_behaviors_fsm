'''
alternates: drive a square, stop for 5 s, repeat. publishes to desired_cmd_vel
'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from ros_behaviors_fsm.drive_square import DriveSquareBehavior


class FiniteStateController(Node):
    STOP_DURATION = 5.0

    def __init__(self):
        super().__init__("finite_state_controller")
        self.square = DriveSquareBehavior()
        self.state = "SQUARE"
        self.stop_start = None
        self.cmd_vel_pub = self.create_publisher(Twist, "desired_cmd_vel", 10)
        self.timer = self.create_timer(0.1, self.run_loop)

    def run_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9
        cmd = Twist()

        if self.state == "SQUARE":
            if self.square.is_finished():
                self.state = "STOP"
                self.stop_start = now
                self.get_logger().info("state -> STOP")
            else:
                cmd = self.square.compute_command(now)

        elif self.state == "STOP":
            if now - self.stop_start > self.STOP_DURATION:
                self.state = "SQUARE"
                self.square.reset(now)
                self.get_logger().info("state -> SQUARE")

        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FiniteStateController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()