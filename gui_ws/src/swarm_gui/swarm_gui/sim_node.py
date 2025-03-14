#ROS Dependencies
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

#other dependencies
import networkx as nx
import numpy as np

#helpers
from swarm_gui.sim_helper import sim_update


class Sim_handler(Node):

    def __init__(self):
        super().__init__('sim_handler')
        self.state_publisher_ = self.create_publisher(Float64MultiArray, 'agents_states', 10)
        fps = 20
        self.timer_period = 1/fps
        self.timer = self.create_timer(self.timer_period, self.sim_frame_callback)

        #system state stuff
        #TODO: eventually should make it so this is from a shared central config. For now just hardcode
        self.num_agents = 5
        self.state_dim = 4
        self.control_dim = 2
        self.network_graph = nx.complete_graph(self.num_agents)

        self.A = np.zeros(self.state_dim, self.state_dim)
        self.A[0, 2] = 1
        self.A[1, 3] = 1

        self.B = np.zeros((self.state_dim, 2))
        self.B[2, 0] = 1
        self.B[3, 1] = 1

        #consensus gain (based on toyota's paper)
        #NOTE: to switch to Annuswamy's setup, we'd need a tiered control loop. Worrying about that later
        gamma0 = 2
        gamma1 = 2
        self.consensus_gain = np.zeros((2, self.state_dim))
        self.consensus_gain[0, 0] = gamma0
        self.consensus_gain[0, 2] = gamma1
        self.consensus_gain[1, 1] = gamma0
        self.consensus_gain[1, 3] = gamma1

        #TODO: eventally will need a node + subscription to handle chaging this
        self.current_leader_control = np.zeros(2)

        #TODO: work out communication exchange for agent state information and deviation info
        #IDEA: sim has GT for agents states, gives to GUI. GUI has GT for formation, gives deviation info to sim
        #placeholder starting agent states
        starting_offset = 0.5*np.sqrt(2)/2
        agent_states = np.zeros((self.num_agents, self.state_dim))
        agent_states[0] = np.array([0, 0, 0, 0])
        agent_states[1] = np.array([0.5, 0, 0, 0])
        agent_states[2] = np.array([starting_offset, -starting_offset, 0, 0])
        agent_states[3] = np.array([0, -0.5, 0, 0])
        agent_states[4] = np.array([-starting_offset, -starting_offset, 0, 0])

        self.prev_agent_states = agent_states

        #placeholder deviations
        desired_deviations = np.zeros((self.num_agents, self.state_dim))
        desired_deviations[0] = np.zeros(4)
        desired_deviations[1] = np.array([1, 0, 0 ,0])
        desired_deviations[2] = np.array([0, 1, 0, 0])
        desired_deviations[3] = np.array([-1, 0, 0, 0])
        desired_deviations[4] = np.array([0, -1, 0, 0])

        self.current_desired_deviations = desired_deviations



    #TODO: implement
    def sim_frame_callback(self):


        udpated_agent_states = sim_update(self.A,
                                          self.B,
                                          self.prev_agent_states,
                                          self.current_leader_control,
                                          self.network_graph,
                                          self.current_desired_deviations,
                                          self.consensus_gain,
                                          self.timer_period)



        #TODO: make helper functions to format agent states into message format and to take the message out

        msg = Float64MultiArray()
        self.state_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    gui_publisher = Sim_handler()

    rclpy.spin(gui_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gui_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()