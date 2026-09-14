import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from neato2_interfaces.msg import Bump
 
class CollisionAvoidanceNode(Node):
    def __init__(self):
        super().__init__("collision_avoidance")
        self.stop_distance = 0.5      # meters
        self.forward_speed = 0.2      # m/s
        self.cmd_vel_pub = self.create_publisher(Twist, "desired_cmd_vel", 10)
        self.scan_sub = self.create_subscription(LaserScan, "scan", self.scan_callback, 10)
        self.bump_sub = self.create_subscription(Bump, "bump", self.bump_callback, 10)
        self.latest_ranges = None
        self.bump_hit = False
        self.timer = self.create_timer(0.1, self.run_loop)
 
    def compute_command(self, scan_ranges, bump_hit):
        return Twist()
 
    def is_blocked(self, scan_ranges, bump_hit):
        return False

    def scan_callback(self, msg):
        self.latest_ranges = msg.ranges
 
    def bump_callback(self, msg):
        self.bump_hit = bool(msg.left_front or msg.left_side or msg.right_front or msg.right_side)
        
    def run_loop(self):
        cmd = self.compute_command(self.latest_ranges, self.bump_hit)
        self.cmd_vel_pub.publish(cmd)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
