import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class FiniteStateController(Node):
    def __init__(self):
        super().__init__("finite_state_controller")
        self.state_pub = self.create_publisher(String, "state", 10)
        self.drive_square_done_sub = self.create_subscription(
            String, "drive_square_done", self.drive_square_done_callback, 10
        )

        # by default starts in SQUARE_DRIVE
        self.current_state = "SQUARE_DRIVE"

        # Separate thread so input() doesn't block rclpy.spin()
        self.input_thread = threading.Thread(target=self.terminal_input_loop, daemon=True)
        self.input_thread.start()

        # Give other nodes' subscriptions a moment to come up via discovery before
        # publishing the default starting state, so it isn't published into the void.
        def publish_initial_state():
            startup_timer.cancel()
            self.set_state(self.current_state)

        # wait for 1 second before publishing
        startup_timer = self.create_timer(1.0, publish_initial_state)

    def set_state(self, new_state):
        self.current_state = new_state
        msg = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)
        print(f">> Published State: {self.current_state}\n")

    def drive_square_done_callback(self, msg):
        # Guard against duplicate/late messages: only act on this while we're
        # actually still in SQUARE_DRIVE, so it can only trigger the switch once.
        if msg.data == "DRIVE_SQUARE_DONE" and self.current_state == "SQUARE_DRIVE":
            self.set_state("WALL_FOLLOWING")

    def print_menu(self):
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
