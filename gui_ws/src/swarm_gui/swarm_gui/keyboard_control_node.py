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
        self.control_publisher_ = self.create_publisher(Float64MultiArray, "leader_control", 10)

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
        # self.get_logger().info(f"key state: {self.curr_key_state}")

        #only process when we've gotten agent state info
        if self.agent_states is None:
            return
        
        #prepare for the control computation
        leader_state = self.agent_states[0]
        curr_xdot = leader_state[2]
        curr_ydot = leader_state[3]
        speed_check = lambda dir, v, vmax: np.sign(v) != dir or np.abs(v) < vmax

        #vars for readability
        left = self.curr_key_state[Key.left]
        right = self.curr_key_state[Key.right]
        up = self.curr_key_state[Key.up]
        down = self.curr_key_state[Key.down]

        if left:
            u_H = -self.max_speed/self.accel_time if speed_check(-1, curr_xdot, self.max_speed) else 0.0
        elif right:
            u_H = self.max_speed/self.accel_time if speed_check(1, curr_xdot, self.max_speed) else 0.0
        else:
            u_H = -self.damping_constant*curr_xdot

        if down:
            u_V = -self.max_speed/self.accel_time if speed_check(-1, curr_ydot, self.max_speed) else 0.0
        elif up:
            u_V = self.max_speed/self.accel_time if speed_check(1, curr_ydot, self.max_speed) else 0.0
        else:
            u_V = -self.damping_constant*curr_ydot 


        control = np.array([u_H, u_V]).reshape(-1, 1)
        # self.get_logger().info(f"control: {control}")


        msg = array_to_msg(control)
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