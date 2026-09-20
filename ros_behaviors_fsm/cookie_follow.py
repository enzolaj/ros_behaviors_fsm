import math
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from std_msgs.msg import String


class CookieFollowNode(Node):
    """This node uses the RANSAC algorithm to fit a circle of known radius (a cookie) """

    def __init__(self):
        """Initializes the node, sets RANSAC parameters, and creates the publisher and subscribers"""
        super().__init__("cookie_following")

        # these params are set based on one google search, radius matches default cylinder in gazebo
        # filter to match size for point scan to, filters before RANSAC
        self.target_radius = 0.5
        self.radius_tol = 0.05

        # inlier tolerance, lower = more restrictive but miss circle, higher = wall can be circle
        self.inlier_tol = 0.03

        # iterations for the ransac algorithm
        self.ransac_iterations = 50

        # inliers needed to fit (this is -3 bc the 3 points form the circle, so 8 is 5 other points)
        self.min_inliers = 8

        # control values
        self.kp_linear = 0.5
        self.kp_angular = 1.0

        # measured to the center, so this must exceed the radius to avoid collisions (still have safety backup tho)
        self.stop_distance = 0.65

        # Track active behavior state
        self.is_active = False

        self.cmd_vel_pub = self.create_publisher(Twist, "desired_cmd_vel", 10)
        self.state_sub = self.create_subscription(String, "state", self.state_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, "scan", self.scan_callback, 10)
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
        # Do not run or publish commands if we are not the active state
        if not self.is_active:
            return

        # iterate through all points from lidar scan and convert to cartesian
        points = []
        for i, r in enumerate(msg.ranges):
            # drop out of range filter to avoid bad points
            if r > msg.range_min and r < msg.range_max:
                angle = math.radians(i)
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append((x, y))

        # if no valid points just return
        if len(points) < 3:
            return

        # start of RANSAC logic
        best_center = None
        best_inlier_count = 0

        for _ in range(self.ransac_iterations):
            # pick 3 random points that are valid
            p1, p2, p3 = random.sample(points, 3)

            cx, cy, r = self.get_circle_from_3_points(p1, p2, p3)

            # if no circle center (just x can tell) then go
            if cx is None:
                continue

            # if not in size tol then go
            if abs(r - self.target_radius) > self.radius_tol:
                continue

            # count inliers, automatic +3 from counting sample is negligble
            inliers = 0
            for px, py in points:
                dist_to_center = math.hypot(px - cx, py - cy)
                if abs(dist_to_center - r) < self.inlier_tol:
                    inliers += 1

            # update best candidate
            if inliers > best_inlier_count and inliers >= self.min_inliers:
                best_inlier_count = inliers
                best_center = (cx, cy)

        cmd = Twist()
        if best_center is not None:
            cx, cy = best_center

            # debug visualization using a flat cylinder to show both center and boundary
            m = Marker()
            m.header = msg.header
            m.type, m.action = Marker.CYLINDER, Marker.ADD
            m.pose.position.x, m.pose.position.y = cx, cy
            m.scale.x = m.scale.y = self.target_radius * 2.0
            m.scale.z, m.color.r, m.color.a = 0.02, 1.0, 0.5
            self.marker_pub.publish(m)

            # scan is in robot frame, atan2 works fine(positive = left)
            # hypot with 2 input = 0,0 base, which works in neato frame, same with atan
            distance = math.hypot(cx, cy)
            angle = math.atan2(cy, cx)

            if distance > self.stop_distance:
                #compute forward and angular vel and clamp to mid speed
                cmd.linear.x = min(0.2, self.kp_linear * distance)
                cmd.angular.z = min(0.174, self.kp_angular * angle)
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

        # D is twice the triangle's area. goes to 0 if points form a line
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