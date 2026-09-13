'''
will publish to cmd_vel, subscribe to scan, bump, desired_vel
acts as safety gate before finally publishing good vel
so this will also timeout if there arent any commands coming just in case
'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from neato2_interfaces.msg import Bump
from rclpy.duration import Duration
from ros_behaviors_fsm.collision_avoidance import (
    CollisionAvoidanceBehavior,
)

class SafetyNode(Node):
    TIMEOUT = Duration(seconds=1.0)
    def __init__(self):
        super().__init__('safety_node')
        self.check_block = CollisionAvoidanceBehavior()
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Twist, "desired_cmd_vel", self.desired_cmd_vel_callback, 10)
        self.create_subscription(LaserScan, "scan", self.scan_callback, 10)
        self.create_subscription(Bump, "bump", self.bump_callback, 10)
        self.latest_ranges = None
        self.hit_bump = False
        self.last_desired_vel_time = None
        self.timer = self.create_timer(0.1, self.timecheck)

    def scan_callback(self, msg):
        self.latest_ranges = msg.ranges

    def bump_callback(self, msg):
        self.hit_bump = bool(msg.left_front or msg.left_side or msg.right_front or msg.right_side)

    def desired_cmd_vel_callback(self, msg):
        self.last_desired_vel_time = self.get_clock().now()
        if self.check_block.is_blocked(self.latest_ranges, self.hit_bump):
            self.cmd_vel_pub.publish(Twist())
        else:
            self.cmd_vel_pub.publish(msg)

    def timecheck(self):
        if self.last_desired_vel_time is None:
            return
        if self.get_clock().now() - self.last_desired_vel_time > self.TIMEOUT:
            self.cmd_vel_pub.publish(Twist())


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
    