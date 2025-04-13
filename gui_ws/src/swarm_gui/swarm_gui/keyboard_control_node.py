#ROS Depdendencies
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray
from geometry_msgs.msg import Twist

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
        self.control_publisher_ = self.create_publisher(Twist, "/agent0/cmd_vel", 10)

        self.state_subscriber_ = self.create_subscription(Float64MultiArray, "agent_states", self.agent_states_callback, 10)

        #Help for later 
        #https://pynput.readthedocs.io/en/latest/keyboard.html

        self.prev_control = np.zeros(2).reshape((2,1))
        self.control_count = 4
        self.key_list = [Key.up, Key.down, Key.left, Key.right]
        self.curr_key_state = {}
        self.damping_constant = 2.5
        self.max_speed = 0.5
        self.accel_time = 1

        for key in self.key_list:
            self.curr_key_state[key] = False

        self.agent_states = None

        self.timer_freq = 10
        self.timer_period = 1/self.timer_freq
        self.timer = self.create_timer(self.timer_period, self.control_update_callback)

        self.listener = pynput.keyboard.Listener(on_press=self.on_press_callback,
                                                 on_release=self.on_release_callback)
        self.listener.start()



    def on_press_callback(self, key):

        char_key = False
        arrow_key = False
        try:
            if key.char:
                char_key = True
        except:
            char_key = False
            if key in self.key_list:
                arrow_key = True


        if char_key:

            #toggle the formation
            if key.char in ['x', 'X']:
                msg = Bool()
                msg.data = True
                self.formation_toggle_publisher_.publish(msg)
            return
        
        if arrow_key:
            self.curr_key_state[key] = True
            return

    def on_release_callback(self, key):
        
        #if the arrow key is released note that
        if key in self.key_list:
            self.curr_key_state[key] = False

    def agent_states_callback(self, msg):
        agent_states = msg_to_array(msg)
        self.agent_states = agent_states

    def control_update_callback(self):

        #only process when we've gotten agent state info
        if self.agent_states is None:
            return
        

        #vars for readability
        left = self.curr_key_state[Key.left]
        right = self.curr_key_state[Key.right]
        up = self.curr_key_state[Key.up]
        down = self.curr_key_state[Key.down]


        max_linear = 0.5
        max_rot = 0.5

        if up:
            linear_velocity = max_linear
        elif down:
            linear_velocity = -max_linear
        else:
            linear_velocity = 0.0


        if left:
            rot_velocity = max_rot
        elif right:
            rot_velocity = -max_rot
        else:
            rot_velocity = 0.0


        msg = Twist()
        msg.linear.x = linear_velocity
        msg.angular.z = rot_velocity

        self.control_publisher_.publish(msg)
 


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