#ROS Depdendencies
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage

from swarm_gui.messaging_helper import msg_to_array, array_to_msg
import numpy as np

class gazebo_sim_interface(Node):

    def __init__(self):
        super().__init__('gazebo_sim_handler')

        self.num_agents = 5
        self.state_dim = 4
        self.most_recent_agent_state = np.zeros((self.num_agents, self.state_dim))


        #create a unique callback for each agent via a lambda function
        


    def agent_state_update_callback(self, msg:Odometry, agent_idx):

        state = np.zeros(self.state_dim)

        #retrieve state information from the message
        state[0] = msg.pose.pose.position.x
        state[1] = msg.pose.pose.position.y
        state[2] = msg.twist.twist.linear.x
        state[3] = msg.twist.twist.linear.y

        #update the state information
        self.most_recent_agent_state[agent_idx]





def main(args=None):
    rclpy.init(args=args)

    gazebo_sim_node = gazebo_sim_interface()

    rclpy.spin(gazebo_sim_node)

    rclpy.shutdown()



