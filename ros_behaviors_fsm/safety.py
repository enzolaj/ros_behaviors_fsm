'''
Will publish to cmd_vel, subscribe to bump, desired_vel
Acts as safety gate before finally publishing good vel
Will also timeout if there arent any commands coming just in case

This node represents the collision avoidance behavior, and serves as the safety e-stop for the system

Every behavior publishes to desired_cmd_vel and this is the only node that publishes to cmd_vel
it only forwards the command if:
    - the bumper hasnt been hit since the last state change
    - the state isnt STOP
    - commands are still coming in (otherwise timeout stop)
'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from neato2_interfaces.msg import Bump
from rclpy.duration import Duration
from std_msgs.msg import String


class SafetyNode(Node):
    """This node sits between the behavior nodes and the neato. It forwards desired_cmd_vel
    to cmd_vel unless the bumper was hit, the state is STOP, or commands stopped coming."""

    # timeout duration when no commands are sent anymore for a while for any software or communication failure reasons
    TIMEOUT = Duration(seconds=1.0)
    def __init__(self):
        """Initializes the node and creates the publisher, subscribers, and timeout timer"""
        super().__init__('safety_node')
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Twist, "desired_cmd_vel", self.desired_cmd_vel_callback, 10)
        self.create_subscription(Bump, "bump", self.bump_callback, 10)
        self.state_sub = self.create_subscription(
            String, "state", self.state_callback, 10
        )
        self.hit_bump = False # latched on a bump, only cleared when the state changes
        self.stopped = False # true while in STOP so nothing gets through
        self.last_desired_vel_time = None
        self.timer = self.create_timer(0.1, self.timecheck) # periodically checks if the commands is idle

    def bump_callback(self, msg):
        """
        Latches hit_bump and stops the neato if any of the bump sensors are triggered.
        args:
            msg (Bump): The incoming bump sensor message.
        returns: None
        """
        # stop if any of the bump is triggered
        # only set to true here so it doesnt reset when the bumper gets released
        if msg.left_front or msg.left_side or msg.right_front or msg.right_side:
            self.hit_bump = True
            self.stop()

    def desired_cmd_vel_callback(self, msg):
        """
        Forwards the desired velocity to cmd_vel unless the bumper was hit or we are in STOP.
        args:
            msg (Twist): The velocity requested by the active behavior node.
        returns: None
        """
        self.last_desired_vel_time = self.get_clock().now()
        # stop if bumper is triggered currently
        if self.hit_bump or self.stopped:
            self.stop()
            return
        self.cmd_vel_pub.publish(msg)

    def timecheck(self):
        """Stops the neato if no desired_cmd_vel has come in for longer than TIMEOUT."""
        if self.last_desired_vel_time is None:
            return
        if self.get_clock().now() - self.last_desired_vel_time > self.TIMEOUT:
            self.stop()
            # reset so it only stops once instead of spamming every tick
            self.last_desired_vel_time = None


    def stop(self):
        """Publishes a zero velocity to stop the neato."""
        self.cmd_vel_pub.publish(Twist())
        print("STOPPING")

    def state_callback(self, msg):
        """
        Stops on the STOP state, otherwise clears the bump latch so behaviors can drive again.
        args:
            msg (String): The incoming state message.
        returns: None
        """
        if msg.data == "STOP":
            # keep blocking commands until we get a different state
            self.stopped = True
            self.stop()
        else:
            # reset the hit bumper status when manually changing states, otherwise this is latched on to be true and nothing will run
            self.stopped = False
            self.hit_bump = False


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
