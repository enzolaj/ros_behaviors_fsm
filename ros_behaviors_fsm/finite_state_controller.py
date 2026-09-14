'''
alternates: drive a square, stop for 5 s, repeat. publishes to desired_cmd_vel
'''
import rclpy
from rclpy.node import Node

class FiniteStateController(Node):
    """
    based on keyboard input, change state topic
    """
    state = "hi meow"
    return state


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


if __name__ == '__main__':
    main()
