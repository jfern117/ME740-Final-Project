#ROS Depdendencies
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

import numpy as np
import networkx as nx

from swarm_gui.messaging_helper import msg_to_array, array_to_msg




class automation_manager(Node):

    def __init__(self):
        super().__init__('automation_manager')

        self.num_agents = 5

        #make a publisher for each follower agent
        self.follower_pub_list = []
        for agent_idx in range(1, self.num_agents):
            curr_pub = self.create_publisher(Twist, f"/agent{agent_idx}/cmd_vel", 10)
            self.follower_pub_list.append(curr_pub)

        fps = 20
        self.timer_period = 1/fps
        self.timer = self.create_timer(self.timer_period, self.autonomous_control_callback)

        self.agent_states = None
        self.dynamic_compensators = None
        self.agent_state_sub = self.create_subscription(Float64MultiArray, "agent_states", self.agent_state_callback, 10)

        self.agent_headings = None
        self.agent_headings_sub = self.create_subscription()

        self.agent_deviations = None
        self.deviation_sub = self.create_subscription(Float64MultiArray, "agent_deviations", self.deviation_callback, 10)
    


        #control stuff
        self.network_graph = nx.complete_graph(self.num_agents)
        self.state_dim = 4
        self.control_dim = 2

        #gain stuff
        #TODO: if this are nowhere near close to working, look at the set point regulation from the paper
        gamma0 = 2
        gamma1 = 2
        self.consensus_gain = np.zeros((2, self.state_dim))
        self.consensus_gain[0, 0] = gamma0
        self.consensus_gain[0, 2] = gamma1
        self.consensus_gain[1, 1] = gamma0
        self.consensus_gain[1, 3] = gamma1

        self.velocity_eps = 0.05 

    #update callbacks
    def agent_state_callback(self, msg):
        self.agent_states = msg_to_array(msg)

    def deviation_callback(self, msg):
        self.agent_deviations = msg_to_array(msg)

    def agent_heading_callback(self, msg):
        self.agent_headings = array_to_msg(msg)
        
    def init_compensators(self):
        """
        Sets the intial velocity direction
        """
        self.dynamic_compensators = np.ones(self.num_agents - 1)*0.1
        neg_check = (self.agent_states[1:, 0] < self.agent_deviations[1:, 0]).flatten()
        self.dynamic_compensators[neg_check] *= -1

    def compute_agent_control_effort(self, agent_idx):

        neighbor_list = list(self.network_graph.neighbors(agent_idx))

        #this is the method done in toyota (paper)
        offset_states = self.agent_states - self.agent_deviations

        consensus_sum = np.zeros(self.state_dim)

        for neighbor_idx in range(len(neighbor_list)):
            consensus_sum += offset_states[agent_idx] - offset_states[neighbor_idx]

        control_input = -self.consensus_gain @ consensus_sum.reshape((-1,1))

        return control_input.flatten()


    def compute_control_efforts(self):
        control_efforts = np.zeros((self.num_agents - 1, self.control_dim))

        for agent_idx in range(1, self.num_agents):
            control_efforts[agent_idx-1] = self.compute_agent_control_effort(agent_idx)
        
        return control_efforts
    
    def compute_agent_velocity_commands(self, control_effort, agent_idx):

        pass



    def autonomous_control_callback(self):

        #only compute if we have everything we need
        if self.agent_deviations is None:
            return
        
        if self.agent_states is None:
            return
        
        if self.agent_headings is None:
            return
        
        #initalize the dynamic compensators if necessary
        if self.dynamic_compensators is None:
            self.init_compensators()


        control_efforts = self.compute_control_efforts()



def main(args = None):
    rclpy.init(args=args)

    automation_manager_node = automation_manager()

    rclpy.spin(automation_manager_node)

    automation_manager_node.destroy_node()
    rclpy.shutdown


if __name__ == '__main__':
    main()
