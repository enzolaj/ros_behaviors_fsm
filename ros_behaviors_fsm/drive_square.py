"""
Drives the Neato in a single square using timed open-loop commands.

Active only in the SQUARE_DRIVE state. Each corner is a timed turn and each
side is a timed drive. After four sides, the node publishes on
drive_square_done so the state machine can move to WALL_FOLLOWING.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class DriveSquareNode(Node):

    def __init__(self):
        """Initializes the node, the square timing parameters, and the publishers and subscribers."""
        super().__init__("drive_square")
        self.forward_speed = 0.3
        self.turn_speed = 0.6
        # start with TURN isntead of DRIVE so won't hit the cookie once eaten
        self.phase = "TURN"  # "DRIVE" or "TURN"
        self.phase_start_time = None
        # hardcoded, measured by visually testing how long it takes complete 1m driving/90 turning
        self.drive_duration = 3  # seconds
        self.turn_duration = 2.7  # seconds
        self.pause_duration = 0.5

        # A square has 4 sides and corners to turn; each side is one DRIVE phase + one TURN phase
        self.turns_completed = 0
        self.num_sides = 4

        # Track active behavior state
        self.is_active = False

        self.cmd_vel_pub = self.create_publisher(Twist, "desired_cmd_vel", 10)
        self.done_pub = self.create_publisher(String, "drive_square_done", 10)
        self.state_sub = self.create_subscription(
            String, "state", self.state_callback, 10
        )

        # Timer: fires 10x per second. This is our control loop.
        self.timer = self.create_timer(0.1, self.run_loop)

    def state_callback(self, msg):
        """Activates the node when the state changes to SQUARE_DRIVE and restarts the square.

        Args:
            msg (String): The current state published by the state machine.
        """
        was_active = self.is_active
        self.is_active = msg.data == "SQUARE_DRIVE"

        # only start if it is changed, do nothing when already in the process
        if self.is_active and not was_active:
            print("start SQUARE DRIVING")
            now = self.get_clock().now().nanoseconds / 1e9
            self.reset(now)

    def reset(self, now):
        """Resets the square to the first TURN phase.

        Args:
            now (float): The current time in seconds.
        """
        self.phase = "TURN"
        self.phase_start_time = now
        self.turns_completed = 0

    # used to send completed signal back to FSM
    # doing this via a new topic instead of adding an existing state in the enum, to keep the FSM the central processing node and only node writing to the state topic
    def finish_square(self):
        """Deactivates the node and publishes on drive_square_done."""
        self.is_active = False
        print("SQUARE DRIVE COMPLETE")
        msg = String()
        msg.data = "DRIVE_SQUARE_DONE"
        self.done_pub.publish(msg)

    def compute_command(self, now):
        """Computes the velocity command for the current phase and advances phases when their time runs out.

        Args:
            now (float): The current time in seconds.

        Returns:
            Twist: The velocity command for this tick. Zero once the square is complete.
        """
        cmd = Twist()

        if self.phase_start_time is None:
            self.reset(now)

        elapsed = now - self.phase_start_time

        if self.phase == "DRIVE":
            if elapsed > self.drive_duration:
                self.turns_completed += 1
                if self.turns_completed >= self.num_sides:
                    self.finish_square()
                    # cmd stays zero so we stop cleanly on the final side
                else:
                    self.phase = "TURN"
                    self.phase_start_time = now
            else:
                cmd.linear.x = self.forward_speed
                cmd.angular.z = 0.0
        elif self.phase == "TURN":
            if elapsed > self.turn_duration:
                self.phase = "DRIVE"
                self.phase_start_time = now
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_speed
        return cmd

    def run_loop(self):
        """Publishes the current command at 10 Hz while the node is active."""
        # Do not run or publish commands if we are not the active state
        if not self.is_active:
            return

        now = self.get_clock().now().nanoseconds / 1e9

        cmd = self.compute_command(now)
        self.cmd_vel_pub.publish(cmd)

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
