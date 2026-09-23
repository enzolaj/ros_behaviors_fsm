'''
will publish to cmd_vel, subscribe to bump, desired_vel
acts as safety gate before finally publishing good vel
so this will also timeout if there arent any commands coming just in case
'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from neato2_interfaces.msg import Bump
from rclpy.duration import Duration
from std_msgs.msg import String


class SafetyNode(Node):
    TIMEOUT = Duration(seconds=1.0)
    def __init__(self):
        super().__init__('safety_node')
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Twist, "desired_cmd_vel", self.desired_cmd_vel_callback, 10)
        self.create_subscription(Bump, "bump", self.bump_callback, 10)
        self.state_sub = self.create_subscription(
            String, "state", self.state_callback, 10
        )
        self.hit_bump = False
        self.last_desired_vel_time = None
        self.timer = self.create_timer(0.1, self.timecheck)

    def bump_callback(self, msg):
        self.hit_bump = bool(msg.left_front or msg.left_side or msg.right_front or msg.right_side)
        if self.hit_bump:
            self.stop()

    def desired_cmd_vel_callback(self, msg):
        self.last_desired_vel_time = self.get_clock().now()
        # Refuse to pass through motion while a bumper is active, so a behavior
        # node can't immediately override the e-stop on its next command.
        if self.hit_bump:
            self.stop()
            return
        self.cmd_vel_pub.publish(msg)

    def timecheck(self):
        if self.last_desired_vel_time is None:
            return
        if self.get_clock().now() - self.last_desired_vel_time > self.TIMEOUT:
            self.stop()


    def stop(self):
        self.cmd_vel_pub.publish(Twist())
        print("STOPPING")

    def state_callback(self, msg):
        if msg.data == "STOP":
            self.stop()
        else:
            # Picking a new behavior state is treated as a manual override/reset:
            # otherwise a stuck hit_bump latch (e.g. if the bump sensor never
            # sends a "cleared" message once contact ends) would block motion
            # forever with no way to recover.
            self.hit_bump = False

            



# i was told the other way of cleanup was dangerous by a friend
def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()         # cleanup

if __name__ == '__main__':
    main()
