#ROS Depdendencies
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

import numpy as np
import networkx as nx
import scipy.linalg

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
        self.agent_headings_sub = self.create_subscription(Float64MultiArray, "agent_headings", self.agent_heading_callback, 10)

        self.agent_deviations = None
        self.deviation_sub = self.create_subscription(Float64MultiArray, "agent_deviations", self.deviation_callback, 10)



        #lower level control stuff
        self.state_dim = 4
        self.low_level_control_dim = 2
        kp = 10
        kd = 10
        self.K_ll = np.array([[kp, 0, kd, 0],
                              [kp, 0, kd, 0]])
        
        A_linearized = np.zeros((self.state_dim, self.state_dim))
        A_linearized[0, 2] = 1
        A_linearized[1, 3] = 1

        B_linearized = np.zeros((self.state_dim, self.low_level_control_dim))
        B_linearized[2, 0] = 1
        B_linearized[3, 1] = 1

        self.A_ll = A_linearized - B_linearized @  self.K_ll
        self.B_ll = B_linearized @  self.K_ll

    
        #consensus control stuff
        self.network_graph = nx.complete_graph(self.num_agents)
        self.lambda_2 = nx.laplacian_spectrum(self.network_graph)[1]
        self.graph_connectivity_constant = 10 + 1/self.lambda_2
        self.control_dim = 4
        Q0 = 5*np.eye(self.state_dim) + 2 * (self.B_ll @ self.B_ll.T)
        self.consensus_gain = self.compute_consensus_gain(Q0)

        self.velocity_eps = 0.01 
        self.velocity_limit = 1.0
        self.rot_vel_limit = 2.0

    #update callbacks
    def agent_state_callback(self, msg):
        self.agent_states = msg_to_array(msg)

    def deviation_callback(self, msg):
        self.agent_deviations = msg_to_array(msg)

    def agent_heading_callback(self, msg):
        self.agent_headings = msg_to_array(msg).flatten()

    def compute_consensus_gain(self, Q0):
        """
        """

        A_mat = self.A_ll.T
        rhs_mat = -Q0 + 2*(self.B_ll @ self.B_ll.T)

        R_mat = scipy.linalg.solve_continuous_lyapunov(A_mat, rhs_mat)
        R_inv = np.linalg.inv(R_mat)

        consensus_gain = self.graph_connectivity_constant * (self.B_ll.T @ R_inv)

        return consensus_gain

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

        high_level_control_input = -self.consensus_gain @ consensus_sum.reshape((-1,1))


        #compute the lower level control inputs
        low_level_control_input = - self.K_ll @ (self.agent_states[agent_idx].reshape(-1,1) - high_level_control_input)
        

        return low_level_control_input.flatten()


    def compute_control_efforts(self):
        control_efforts = np.zeros((self.num_agents - 1, self.low_level_control_dim))

        for agent_idx in range(1, self.num_agents):
            control_efforts[agent_idx-1] = self.compute_agent_control_effort(agent_idx)
        
        return control_efforts
    
    def compute_agent_velocity_commands(self, control_effort, agent_idx):

        u1 = control_effort[0]
        u2 = control_effort[1]
        heading = self.agent_headings[agent_idx]

        #calculate the rate of change of the compensator
        zetadot = u1*np.cos(heading) + u2*np.sin(heading)

        #update the compensator value (using forward euler for the sake of simplicity)
        self.dynamic_compensators[agent_idx-1] += zetadot*self.timer_period

        #we're gonna do basic check to prevent singularity issue
        if np.abs(self.dynamic_compensators[agent_idx-1]) > self.velocity_eps:
            omega = (-u1*np.sin(heading) + u2*np.cos(heading))/self.dynamic_compensators[agent_idx-1]
        else:
            omega = 0.0 #lets just make it simple

        cmd = Twist()

        #adding in a velocity limit
        if np.abs(self.dynamic_compensators[agent_idx-1]) >= self.velocity_limit:
            self.dynamic_compensators[agent_idx-1] = self.velocity_limit

        if np.abs(omega) >= self.rot_vel_limit:
            omega= self.velocity_limit

        cmd.linear.x = self.dynamic_compensators[agent_idx-1]
        cmd.angular.z = omega

        return cmd



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

        for agent_idx in range(1, self.num_agents):
            vel_cmd = self.compute_agent_velocity_commands(control_efforts[agent_idx - 1], agent_idx)
            self.follower_pub_list[agent_idx-1].publish(vel_cmd)

def main(args = None):
    rclpy.init(args=args)

    automation_manager_node = automation_manager()

    rclpy.spin(automation_manager_node)

    automation_manager_node.destroy_node()
    rclpy.shutdown


if __name__ == '__main__':
    main()
