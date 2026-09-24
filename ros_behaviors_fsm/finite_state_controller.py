"""
Finite-state controller that switches between the Neato's behaviors.

Owns the current state and is the only node that publishes on /state. Behavior
nodes report events on their own topics, and this node decides whether each
event causes a transition. The state can also be set manually from the terminal.
"""
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32

class FiniteStateController(Node):
    def __init__(self):
        """Initializes the node, starts the keyboard input thread, and schedules the first state publish."""
        super().__init__("finite_state_controller")
        self.state_pub = self.create_publisher(String, "state", 10)
        self.drive_square_done_sub = self.create_subscription(
            String, "drive_square_done", self.drive_square_done_callback, 10
        )

        # have extra info just in case for future developments
        # these numbers are not used right now
        self.cookies_found_sub = self.create_subscription(
            Int32, "cookies_found", self.cookies_found_callback, 10
        )
        self.cookies_eaten_sub = self.create_subscription(
            Int32, "cookies_eaten", self.cookies_eaten_callback, 10
        )

        # by default starts in SQUARE_DRIVE
        self.current_state = "SQUARE_DRIVE"

        # start a separate thread for listening to the keyboard input
        self.input_thread = threading.Thread(target=self.terminal_input_loop, daemon=True)
        self.input_thread.start()

        # need to wait for other nodes to be up first to subscribe first before making the first publish
        def publish_initial_state():
            """Publishes the initial state once, after some time for things to subscribe and load."""
            startup_timer.cancel()
            self.set_state(self.current_state)

        # wait for 1 second before publishing
        startup_timer = self.create_timer(1.0, publish_initial_state)

    def set_state(self, new_state):
        """Updates the current state and publishes it on /state.

        Args:
            new_state (str): The state to switch to.
        """
        self.current_state = new_state
        msg = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)
        print(f">> Published State: {self.current_state}\n")

    def drive_square_done_callback(self, msg):
        """Switches from SQUARE_DRIVE to WALL_FOLLOWING when the square is finished.

        Args:
            msg (String): The completion message from the drive_square node.
        """
        # only trigger when switching from SQUARE_DRIVE to WALL_FOLLOWING
        if self.current_state == "SQUARE_DRIVE":
            self.set_state("WALL_FOLLOWING")

    def cookies_found_callback(self, msg):
        """Switches from WALL_FOLLOWING to COOKIE_FOLLOW when a new cookie is detected.

        Args:
            msg (Int32): The running count of cookies found.
        """
        # the call back does not need to use the data, but just to keep the number for future reference
        if self.current_state == "WALL_FOLLOWING":
            self.set_state("COOKIE_FOLLOW")

    def cookies_eaten_callback(self, msg):
        """Switches from COOKIE_FOLLOW to SQUARE_DRIVE when a cookie is reached.

        Args:
            msg (Int32): The running count of cookies eaten.
        """
        # the call back does not need to use the data, but just to keep the number for future reference
        if self.current_state == "COOKIE_FOLLOW":
            self.set_state("SQUARE_DRIVE")

    # menu for manual control of the state
    def print_menu(self):
        """Prints the manual state selection menu to the terminal."""
        print("""
=============================
Select Behavior State:
1: Square Drive
2: Wall Following
3: Cookie Following
0: Stop (Idle)
=============================
Enter choice: """, end="", flush=True)

    def terminal_input_loop(self):
        """Reads keyboard input in a background thread and sets the state manually.

        Keys 1, 2, 3, and 0 select SQUARE_DRIVE, WALL_FOLLOWING, COOKIE_FOLLOW,
        and STOP. Invalid input is ignored.
        """
        while rclpy.ok():
            self.print_menu()
            try:
                user_input = input().strip()
            except EOFError:
                break

            new_state = None
            if user_input == "1":
                new_state = "SQUARE_DRIVE"
            elif user_input == "2":
                new_state = "WALL_FOLLOWING"
            elif user_input == "3":
                new_state = "COOKIE_FOLLOW"
            elif user_input == "0":
                new_state = "STOP"
            else:
                print(f"Invalid input: '{user_input}'. Please enter 0, 1, 2, or 3.")
                continue

            self.set_state(new_state)


def main(args=None):
    rclpy.init(args=args)
    node = FiniteStateController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
