"""
Follows the wall on the Neato's right side using lidar data.

Active only in the WALL_FOLLOWING state. A controller on distance
and heading error keeps the robot parallel to the wall, and a forward check
triggers a turn at corners.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist, Point, PointStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA, Float32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point
from collections import deque

class WallFollowerNode(Node):
    """This node uses triangulation with two lidar points (one perpendicular to the Neato and
    another 20 degrees ahead of it) to follow the wall on its right side. A single lidar point
    that is straight ahead determines when to turn for corners."""

    def __init__(self):
        """Initializes node; sets subscribers and publishers; and triangulation and collision
        avoidance parameters."""

        super().__init__("wall_following")
        self.forward_speed = 0.2 # m/s
        self.target_distance = 0.3 # meters to stay away from the wall
        self.side = -1 # -1 for right wall, 1 for left wall
        self.max_turn = 1.0 # rad/s
        self.max_dist_error = 0.5 # m

        # control values
        self.p_dist = 1.5
        self.damping_ratio = 1 # add margins for lag?
        # critical damping: p_angle = 2 * zeta * sqrt(v * p_dist)
        self.p_angle = 2.0 * self.damping_ratio * math.sqrt(
            self.forward_speed * self.p_dist)
        self.get_logger().info(
            f"p_dist={self.p_dist:.2f}, p_angle={self.p_angle:.2f} "
            f"(zeta={self.damping_ratio})")

        self.fixed_frame = "odom" # frame where hit point trail is stored
        self.hit_history = deque(maxlen=200)  # oldest points drop off automatically
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(MarkerArray, "wall_markers", 10)
        # [dist_error, alpha, steer_before_clamp] for plotting from a bag
        self.debug_pub = self.create_publisher(Float32MultiArray, "wall_debug", 10)

        self.cmd_vel_pub = self.create_publisher(Twist, "desired_cmd_vel", 10)

        self.create_subscription(LaserScan, "scan", self.scan_callback, qos_profile_sensor_data)
        self.state_sub = self.create_subscription(
            String, "state", self.state_callback, 10
        )

        # Track active behavior state
        self.is_active = False

    def state_callback(self, msg):
        """Sets the node active only when the state is WALL_FOLLOWING.

        Args:
            msg (String): The current state published by the state machine.
        """
        was_active = self.is_active
        self.is_active = msg.data == "WALL_FOLLOWING"

        if self.is_active and not was_active:
            print("start WALL_FOLLOWING")

    @staticmethod
    def valid_point(range_index, msg):
        """Determines if a range is within the sensor's limits. False for 0, inf, and NaN.

        Args:
            range_index (float): Range reading from the lidar scan, in meters.
            msg (LaserScan): The scan the reading came from.

        Returns:
            bool: True if the reading is valid.
        """
        return msg.range_min < range_index < msg.range_max

    def wall_ahead(self, msg):
        """Determines if the lidar reading straight ahead of the Neato is under a threshold.

        If so, there is a wall ahead.

        Args:
            msg (LaserScan): The incoming lidar scan.

        Returns:
            bool: True if a wall is within 4 * target_distance ahead.
        """
        x = msg.ranges[0]
        # ignore 0 / inf readings so a bad point doesn't count as a wall
        return self.valid_point(x, msg) and x <= 4 * self.target_distance


    def scan_callback(self, msg):
        """Computes and publishes the steering command from the latest scan.

        Args:
            msg (LaserScan): The incoming lidar scan.
        """
        # Do not run or publish commands if we are not the active state
        if not self.is_active:
            return

        cmd = Twist()
        cmd.linear.x = self.forward_speed

        # 270 = right. 290 = 20deg above right
        b = msg.ranges[270]
        a = msg.ranges[290]

        # just drive straight if there is no valid point
        if not (self.valid_point(a, msg) and self.valid_point(b, msg)):
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            return

        theta = math.radians(20) # hardcoded based on a,b

        # angle of the wall relative to the robot (alpha)
        numerator = a * math.cos(theta) - b
        denominator = a * math.sin(theta)
        alpha = math.atan2(numerator, denominator)

        # perpendicular distance to the wall
        current_distance = b * math.cos(alpha)

        # find how much we're off from target distance
        dist_error = current_distance - self.target_distance
        capped_error = max(-self.max_dist_error, min(self.max_dist_error, dist_error))
        # -1 multiplier because the wall is on the right, make not hardcoded later
        steer = -1.0 * (self.p_dist * capped_error + self.p_angle * alpha)

        # turn if wall ahead
        if self.wall_ahead(msg):
            cmd.angular.z = -1.0 * self.side
        # else, clamp the turn speed
        else:
            cmd.angular.z = max(-self.max_turn, min(self.max_turn, steer))


        self.cmd_vel_pub.publish(cmd)

        self.debug_pub.publish(
            Float32MultiArray(data=[dist_error, alpha, steer]))
        self.publish_markers(msg.header.frame_id,
                            self.polar_to_point(b, 270),
                            self.polar_to_point(a, 290))


    @staticmethod
    def polar_to_point(range_index, angle):
        """Transforms a point from polar to Cartesian coordinates.

        Args:
            range_index (float): Range reading from the lidar scan, in meters.
            angle (int): Angle of the reading in degrees.

        Returns:
            Point: The point in the laser frame, before the 180 deg flip in publish_markers.
        """
        return Point(x = range_index * math.cos(math.radians(angle)),
                     y = range_index * math.sin(math.radians(angle)),
                     z = 0.0)


# LLMs were used to write the framework for these visualization functions. Debugging and revisions were done on our own.
    @staticmethod
    def marker(frame, marker_id, marker_type):
        """Creates a marker with the shared fields filled in. The caller sets the rest."""
        m = Marker()
        m.header.frame_id = frame # stamp at 0, use latest transform
        m.ns = "wall_follower"
        m.id = marker_id # same ns + id, replace old marker
        m.type = marker_type
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.lifetime = Duration(seconds=0.5).to_msg()
        return m

    def to_fixed_frame(self, points, frame):
        """Transforms points given in 'frame' into the fixed frame. [] if TF not ready."""
        try:
            tf = self.tf_buffer.lookup_transform(self.fixed_frame, frame, Time())
        except TransformException as e:
            self.get_logger().warn(f"TF not ready: {e}", throttle_duration_sec = 2.0)
            return []
        return [do_transform_point(PointStamped(point=p), tf). point for p in points]


    def publish_markers(self, frame, point_b, point_a):
        """Publishes markers showing the current lidar points and the last 200 hits."""
        origin = Point(x=0.0, y=0.0, z=0.0)

        # Point has no arithmetic operators, so negate x and y explicitly (180 deg about z)
        point_b = Point(x=-point_b.x, y=-point_b.y, z=point_b.z)
        point_a = Point(x=-point_a.x, y=-point_a.y, z=point_a.z)

        # lines from laser to hit points and wall between (triangle)
        triangle = self.marker(frame, 0, Marker.LINE_STRIP)
        triangle.scale.x = 0.01  # line width, m
        triangle.color = ColorRGBA(r=0.2, g=0.9, b=0.2, a=1.0)
        triangle.points = [origin, point_b, point_a, origin]

        # current hit points
        current = self.marker(frame, 1, Marker.SPHERE_LIST)
        current.scale.x = current.scale.y = current.scale.z = 0.06
        current.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        current.points = [point_b, point_a]

        # previous hit points, stored in fixed frame to pin to wall
        self.hit_history.extend(self.to_fixed_frame([point_b, point_a], frame))
        history = self.marker(self.fixed_frame, 2, Marker.POINTS)
        history.scale.x = history.scale.y = 0.03  # POINTS: x = width, y = height
        history.color = ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.7)
        history.points = list(self.hit_history)

        self.marker_pub.publish(MarkerArray(markers=[triangle, current, history]))




def main(args=None):
    """Runs the node and sends a zero velocity on shutdown."""
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