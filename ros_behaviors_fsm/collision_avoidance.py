import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from neato2_interfaces.msg import Bump
from std_msgs.msg import String

class CollisionAvoidanceNode(Node):
    """This node stops the motors when an object is under a certain distance threshold,
    then commands the robot to turn.

    Publishers:
        Twist cmd-vel message

    Subscribers: 
        Bump
        Scan
    """

    def __init__(self):
        """Initializes class"""
        super().__init__("collision_avoidance")
        self.distance_threshold = 0.5      # meters
        self.obstacle_distance = None      # meters 
        self.forward_speed = 0.3      # m/s
        self.obstacle_detection_fov = 30 # deg

        self.cmd_vel_pub = self.create_publisher(Twist, "desired_cmd_vel", 10)
        self.scan_sub = self.create_subscription(LaserScan, "scan", self.scan_callback, 10)
        self.bump_sub = self.create_subscription(Bump, "bump", self.bump_callback, 10)
        self.state_sub = self.create_subscription(
            String, "state", self.state_callback, 10
        )

        # Track active behavior state
        self.is_active = False

        self.bump_hit = False
        self.Kp = 0.2
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
 
    def compute_command(self):
        """Decide how the Neato should move. If no sensor data is received, stay still. 
        If the distance threshold is met or a bump occurs, stay still. Otherwise,
        move slower the closer you get to an obstacle."""
        msg = Twist()
        if self.obstacle_distance is None:
            msg.linear.x = 0.0
        elif self.is_blocked():
            msg.linear.x = 0.0 
        else: 
            # drive speed proportional to how close it is to the obstacle, slower when closer
            speed_x = self.Kp*(self.obstacle_distance - self.distance_threshold)
            msg.linear.x = min(speed_x, self.forward_speed) # boudned by min speed
        return msg
 
    def is_blocked(self):
        """A block is when the bump sensor is triggered or the distance threshold
        is met."""
        return (
            self.obstacle_distance is not None
            and self.obstacle_distance <= self.distance_threshold
        ) or self.bump_hit

    def state_callback(self, msg):
        was_active = self.is_active
        self.is_active = msg.data == "COLLISION_AVOIDANCE"

        if self.is_active and not was_active:
            print("start COLLISION_AVOIDANCE")

    def scan_callback(self, msg):
        """Log the latest obstacle distance straight ahead from the Neato."""
        self.obstacle_distance = msg.ranges[0]
 
    def bump_callback(self, msg):
        """Log the latest bump state."""
        self.bump_hit = bool(msg.left_front or msg.left_side or msg.right_front or msg.right_side)
        
    def run_loop(self):
        # Do not run or publish commands if we are not the active state
        if not self.is_active:
            return
        cmd = self.compute_command()
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
