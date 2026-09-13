
import math
 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
 

class DriveSquareBehavior:
    def __init__(self):
        self.forward_speed = .3
        self.turn_speed = .6
        self.phase = "DRIVE"        # "DRIVE" or "TURN"
        self.phase_start_time = None
        self.corners_turned = 0
        self.drive_duration = 3 #seconds
        self.turn_duration = 2.7 #seconds
        self.pause_duration = .5
 
    def reset(self, now):
        self.phase = "DRIVE"
        self.phase_start_time = now
        self.corners_turned = 0
 
    def is_finished(self):
        return self.corners_turned >= 4
 
    def compute_command(self, now):

        cmd = Twist()
 
        if self.phase_start_time is None:
            self.reset(now)
 
        elapsed = now - self.phase_start_time
 
        if self.phase == "DRIVE":
            if elapsed > self.drive_duration:
                self.phase = "TURN"
                self.phase_start_time = now
            else:
                cmd.linear.x = self.forward_speed
                cmd.angular.z = 0.0
        elif self.phase == "TURN":
            if elapsed > self.turn_duration:
                self.phase = "DRIVE"
                self.phase_start_time = now
                self.corners_turned += 1
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_speed
        return cmd

 
 
class DriveSquareNode(Node):
 
    def __init__(self):
        super().__init__("drive_square")
        self.behavior = DriveSquareBehavior()

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
 
        # Timer: fires 10x per second. This is our control loop.
        self.timer = self.create_timer(0.1, self.run_loop)
 
    def run_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9
 
        cmd = self.behavior.compute_command(now)
        self.cmd_vel_pub.publish(cmd)
 
        if self.behavior.is_finished():
            self.cmd_vel_pub.publish(Twist())   # zeros
            self.timer.cancel()
            # lol 
            self.get_logger().info("Square complete.")
 
 
def main(args=None):
    rclpy.init(args=args)
    node = DriveSquareNode()
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