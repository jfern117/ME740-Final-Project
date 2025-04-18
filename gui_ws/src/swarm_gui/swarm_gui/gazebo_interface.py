#ROS Depdendencies
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry

from swarm_gui.messaging_helper import msg_to_array, array_to_msg
import numpy as np

class gazebo_sim_interface(Node):

    def __init__(self):
        super().__init__('gazebo_sim_handler')

        self.num_agents = 5
        self.state_dim = 4
        self.most_recent_agent_state = np.zeros((self.num_agents, self.state_dim))
        self.most_recent_agent_headings = np.zeros(self.num_agents)

        #don't send state info until we've got data from every agent
        self.agent_init_tracking = np.zeros(self.num_agents, dtype=bool)

        #create a unique callback for each agent via a lambda function
        self.agent_state_update_sub_list = []
        for agent_idx in range(self.num_agents):
            curr_sub = self.create_subscription(Odometry,
                                                f'agent{agent_idx}/odom',
                                                lambda msg, idx = agent_idx : self.agent_state_update_callback(msg, idx),
                                                10)
            self.agent_state_update_sub_list.append(curr_sub)

            self.agent_state_pub_ = self.create_publisher(Float64MultiArray, "agent_states", 10)
            self.agent_heading_pub_ = self.create_publisher(Float64MultiArray, "agent_headings", 10)
            self.timer_period = 1/20
            self.timer = self.create_timer(self.timer_period, self.agent_state_pub_callback)




    def agent_state_update_callback(self, msg:Odometry, agent_idx):

        #track agent spawning for init configuration handling
        if not self.agent_init_tracking[agent_idx]:
            self.agent_init_tracking[agent_idx] = True

        state = np.zeros(self.state_dim)

        #retrieve state information from the message
        state[0] = msg.pose.pose.position.x
        state[1] = msg.pose.pose.position.y
        state[2] = msg.twist.twist.linear.x
        state[3] = msg.twist.twist.linear.y

        #update the state information
        self.most_recent_agent_state[agent_idx] = state

        #get the heading
        #NOTE: Technically, getting the heading isn't 100% physically accurate, but it is good enough for this basic purpose.
        quat_x = msg.pose.pose.orientation.x
        quat_y = msg.pose.pose.orientation.y
        quat_z = msg.pose.pose.orientation.z
        quat_w = msg.pose.pose.orientation.w

        #commpute the heading
        #we are computing this as the rotation about the z-axis in a tait bryan 321 euler angle sense, see following for details
        #source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        heading = np.arctan2(2*(quat_w*quat_z + quat_x*quat_y), 1 - 2*(np.square(quat_y) + np.square(quat_z)))

        self.most_recent_agent_headings[agent_idx] = heading

        # self.get_logger().info(f"retrieved state {state} from agent {agent_idx}")

    def agent_state_pub_callback(self):

        if np.all(self.agent_init_tracking):
            self.agent_state_pub_.publish(array_to_msg(self.most_recent_agent_state))
            self.agent_heading_pub_.publish(array_to_msg(self.most_recent_agent_headings.reshape(-1, 1)))


def main(args=None):
    rclpy.init(args=args)

    gazebo_sim_node = gazebo_sim_interface()

    rclpy.spin(gazebo_sim_node)

    rclpy.shutdown()



