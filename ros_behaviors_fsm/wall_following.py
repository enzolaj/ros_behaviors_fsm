import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String


class WallFollowerNode(Node):

    def __init__(self):
        super().__init__("wall_following")
        self.forward_speed = 0.2 #m/ds
        self.target_distance = 0.15 # meters to stay away from the wall
        self.side = -1 #  -1 for right wall, 1 for left wall

        # control values
        self.p_dist = 1.5   
        self.p_angle = 1.0  


        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        
        self.create_subscription(LaserScan, "scan", self.scan_callback, qos_profile_sensor_data)

    def compute_command(self, msg):
            cmd = Twist()
            cmd.linear.x = self.forward_speed

            # 270 = right. 290 = 20deg above right
            b = msg.ranges[270]  
            a = msg.ranges[290]  
            
            # just drive straight if there is no valid point
            if b == 0.0 or math.isinf(b) or a == 0.0 or math.isinf(a):
                cmd.angular.z = 0.0
                return cmd

            theta = math.radians(20) # hardcoded based on a,b
            
            # angle of the wall relative to the robot (alpha)
            numerator = a * math.cos(theta) - b
            denominator = a * math.sin(theta)
            alpha = math.atan2(numerator, denominator)
            
            # perpendicular distance to the wall
            current_distance = b * math.cos(alpha)
            
            # find how far we arent
            dist_error = current_distance - self.target_distance
            
            # -1 multiplier because the wall is on the right, make not hardcoded later
            steer = -1.0 * (self.p_dist * dist_error + self.p_angle * alpha) # still need to have tuned explanation
            
            # clamp the turn speed
            cmd.angular.z = max(-1.0, min(1.0, steer))

            return cmd

    def scan_callback(self, msg):
        cmd = self.compute_command(msg)
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
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