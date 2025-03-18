#ROS Depdendencies
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray

#package dependencies
from swarm_gui.messaging_helper import msg_to_array, array_to_msg


#TODO: figure out how to do key controls with this and talk to the others
import pynput
from pynput.keyboard import Key
import numpy as np


## ROS Node ##
class Keyinput_manager(Node):

    def __init__(self):
        super().__init__('keyinput_handler')

        #Various pub/subs
        self.formation_toggle_publisher_ = self.create_publisher(Bool, "formation_toggle", 10)

        #Help for later 
        #https://pynput.readthedocs.io/en/latest/keyboard.html

        self.prev_control = np.zeros(2).reshape((2,1))
        self.control_count = 4
        self.curr_key_state = {}

        self.listener = pynput.keyboard.Listener(on_press=self.on_press_callback)
        self.listener.start()



    def on_press_callback(self, key):

        self.get_logger().info("callback reached")

        self.get_logger().info('alphanumeric key {0} pressed'.format(
            key.char))

        if key.char in ['x', 'X', Key.up, Key.down, Key.left, Key.right]:
            self.get_logger().info(key)


def main(args=None):
    rclpy.init(args=args)

    keyboard_node = Keyinput_manager()

    rclpy.spin(keyboard_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    keyboard_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()