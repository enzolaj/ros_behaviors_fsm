import math
import random
 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
 
 
class CookieFollowNode(Node):
    """This node uses the RANSAC algorithm to fit a circle of known radius (a cookie) """
 
    def __init__(self):
        """Initializes the node, sets RANSAC parameters, and creates the publisher and subscribers"""
        super().__init__("cookie_following")
 
        # radius of the cookie, radius filter value
        self.target_radius = 0.24
        self.radius_tol = 0.03
 
        # inlier tolerance, lower = more restrictive but miss circle, higher = wall can be circle
        self.inlier_tol = 0.02
 
        # iterations for the ransac algorithm
        self.ransac_iterations = 200
 
        # inliers needed to fit, counted over the whole scan
        self.min_inliers = 6
 
        # p2 and p3 are picked within this distance of p1
        # every point on the cookie is within one diameter of every other point on it
        # aka local sample
        self.sample_radius = 2.0 * self.target_radius
 
        # control values, lowkey not that useful bc angle should be faster and there isnt much overshoot
        self.kp_linear = 0.5
        self.kp_angular = 0.9
        self.max_linear = 0.2 #m/s
        self.max_angular = 6.7 #rad/s
 
        # measured to the center, so this must exceed the radius to avoid collisions (still have safety backup tho)
        self.stop_distance = 0.65
 
        # Track active behavior state
        self.is_active = False

        # lidar mounting on the neato (from tf2_echo base_link base_laser_link)
        # rotated 180 deg about z and 0.084 m behind the center of base_link
        self.lidar_offset_x = -0.084
 
        self.cmd_vel_pub = self.create_publisher(Twist, "desired_cmd_vel", 10)
        self.state_sub = self.create_subscription(String, "state", self.state_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, "scan", self.scan_callback, qos_profile_sensor_data)
        self.marker_pub = self.create_publisher(Marker, "cookie_debug", 10)
 
    def state_callback(self, msg):
        """
        Updates the active state of the node based on incoming state messages.
        args:
            msg (String): The incoming state message.
        returns: None
        """
        was_active = self.is_active
        self.is_active = msg.data == "COOKIE_FOLLOW"
 
        # If we just switched into COOKIE_FOLLOW, print a notification
        if self.is_active and not was_active:
            print("start COOKIE FOLLOWING")
 
    def scan_callback(self, msg):
        """
        Processes incoming laser scan messages to find the cookie and commands (published vel) the neato to follow it.
        args:
            msg (LaserScan): The incoming laser scan message containing range data.
        returns: None
        """
        # same state architecture from other nodes
        # Do not run or publish commands if we are not the active state
        if not self.is_active:
            return
 
        # iterate through all points from lidar scan and convert to cartesian
        points = []
        for i, r in enumerate(msg.ranges):
            # drop out of range filter to avoid bad points
            if r > msg.range_min and r < msg.range_max:
                angle = msg.angle_min + (i * msg.angle_increment)
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append((x, y))
 
        if len(points) < 3:
            return
 
        best_center = None
        best_inlier_count = 0
 
        for _ in range(self.ransac_iterations):
            # pick a random point, then two more from its nearby area
            p1 = random.choice(points)
            neighbors = []
            for p in points:
                # get nearby points, this also makes it easy for wraparound logic bc index would go 361->0
                if p != p1 and math.hypot(p[0] - p1[0], p[1] - p1[1]) < self.sample_radius:
                    neighbors.append(p)
            # if no neighbors, try again
            if len(neighbors) < 2:
                continue
            p2, p3 = random.sample(neighbors, 2)
 
            cx, cy, r = self.get_circle_from_3_points(p1, p2, p3)

            # the next lines are filters that reject the proposes circle
            # if circle isn't found, try again (cx is none = no circle)
            if cx is None:
                continue

            # if the radius is too diff, try again
            if abs(r - self.target_radius) > self.radius_tol:
                continue
 
            # if center isn't further from robot than points, try again 
            if math.hypot(cx, cy) <= math.hypot(p1[0], p1[1]):
                continue

            # RANSAC logic
            inliers = 0
            for px, py in points:
                dist_to_center = math.hypot(px - cx, py - cy)
                if abs(dist_to_center - r) < self.inlier_tol:
                    inliers += 1
            if inliers > best_inlier_count and inliers >= self.min_inliers:
                best_inlier_count = inliers
                best_center = (cx, cy)
 
        cmd = Twist()
        if best_center is not None:
            cx, cy = best_center
 
            # debug visualization using a flat cylinder to show both center and boundary
            # this was ai-assisted and human reviewed :D
            m = Marker()
            m.header = msg.header
            m.type, m.action = Marker.CYLINDER, Marker.ADD
            m.pose.position.x, m.pose.position.y = cx, cy
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = self.target_radius * 2.0
            m.scale.z, m.color.r, m.color.a = 0.02, 1.0, 0.5
            self.marker_pub.publish(m)
 
            # cx, cy are in the lidar frame, which is flipped 180 deg from the robot
            # rotating by 180 deg negates x and y, then shift by where the lidar sits
            bx = self.lidar_offset_x - cx
            by = -cy
 
            # now in robot frame, atan2 works fine (positive = left)
            distance = math.hypot(bx, by)
            angle = math.atan2(by, bx)
 
            if distance > self.stop_distance:
                #compute forward and angular vel and clamp to max speed
                cmd.linear.x = min(self.max_linear, self.kp_linear * distance)
                cmd.angular.z = max(-self.max_angular, min(self.max_angular, self.kp_angular * angle))
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
        else:
            # no reliable detection, so give up because we are sad and lazy
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
 
            # hide marker when cookie is lost
            m = Marker(action=Marker.DELETE)
            m.header = msg.header
            self.marker_pub.publish(m)
 
        self.cmd_vel_pub.publish(cmd)
 
    def get_circle_from_3_points(self, p1, p2, p3):
        """Calculates the circumcircle of three 2D points.
        args:
            p1 (tuple): The first cartesian coordinate (x, y).
            p2 (tuple): The second cartesian coordinate (x, y).
            p3 (tuple): The third cartesian coordinate (x, y).
        returns:
            tuple: (cx, cy, radius) of the circumcircle, or (None, None, None) if points are nearly collinear.
        """
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
 
        # D is twice the triangle area. goes to 0 if points form a line
        D = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
 
        # prevent flat walls from generating massive circles
        if abs(D) < 1e-4:
            return None, None, None
 
        # use circumcenter formula
        cx = ((x1**2 + y1**2) * (y2 - y3) + (x2**2 + y2**2) * (y3 - y1) + (x3**2 + y3**2) * (y1 - y2)) / D
        cy = ((x1**2 + y1**2) * (x3 - x2) + (x2**2 + y2**2) * (x1 - x3) + (x3**2 + y3**2) * (x2 - x1)) / D
 
        # any point provides the radius, just use first
        radius = math.hypot(x1 - cx, y1 - cy)
 
        return cx, cy, radius
 
 
def main(args=None):
    rclpy.init(args=args)
    node = CookieFollowNode()
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