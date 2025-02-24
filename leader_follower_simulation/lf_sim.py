import numpy as np
import networkx as nx
import scipy.integrate
import matplotlib.pyplot as plt
import scipy.signal

from matplotlib.animation import FuncAnimation
from matplotlib.animation import PillowWriter, FFMpegWriter


from sim_helper import plot_sim_frame, consensus_control_input, single_agent_state_derivative

#suppose we have N + 1 vehicles in a 2D plane with second order integrator dynamics (p = v, vdot = u)

#TODO: vary and test

## Simulation setup ##

#variables we'll likely adjust frequently
sim_time = 10
sim_dt = 0.001

#setting up network. For simplicity, everything can talk to one another. The leader will be agent 0
num_followers = 4
num_agents = num_followers + 1
state_dim = 4

#create the network. Note to future Jimmy, this can also be done with nx.complete_graph()
comm_network = nx.Graph()
comm_network.add_nodes_from(np.arange(num_agents))
edges = []
for idx in range(num_followers):
    edges = edges + [(idx, other.item()) for other in np.arange(idx+1, num_agents)]
comm_network.add_edges_from(edges)


#each agent has a four element state: [x, y, xdot, ydot]. Here are the starting states
starting_offset = np.sqrt(2)/2
agent_states = np.zeros((num_agents, state_dim))
agent_states[0] = np.array([0, 0, 0, 0.5])
agent_states[1] = np.array([1, 0, 0, 0])
agent_states[2] = np.array([starting_offset, -starting_offset, 0, 0])
agent_states[3] = np.array([0, -1, 0, 0])
agent_states[4] = np.array([-starting_offset, -starting_offset, 0, 0])

# Each agent need a designated deviation from the leader
desired_deviations = np.zeros([num_agents, state_dim])
desired_deviations[0] = np.zeros(4)
desired_deviations[1] = np.array([-2, 0, 0, 0])
desired_deviations[2] = np.array([0, 2, 0, 0])
desired_deviations[3] = np.array([2, 0, 0 ,0])
desired_deviations[4] = np.array([0, -2, 0, 0])

#dynamics equations - double integrator
A = np.zeros((state_dim, state_dim))
A[0, 2] = 1
A[1, 3] = 1

B = np.zeros((state_dim, 2))
B[2, 0] = 1
B[3, 1] = 1

#consensus gain (based on toyota's paper)
gamma0 = 1
gamma1 = 1
consensus_gain = np.zeros((2, state_dim))
consensus_gain[0, 0] = gamma0
consensus_gain[0, 2] = gamma1
consensus_gain[1, 1] = gamma0
consensus_gain[1, 3] = gamma1

## Simulation ##
def sim_dynamics_update(t, x):
    """
    """
    #assuming that we are flattening our input and output
    curr_state = x.reshape(num_agents, state_dim)

    #prepare our output variable
    state_derivative = np.zeros(np.shape(curr_state))

    #calculate the change in the leader separately
    state_derivative[0] = single_agent_state_derivative(A, B, curr_state[0], control = np.zeros(2)).flatten()

    #compute the dynamic updates for all the followers
    for agent_idx in range(1, num_agents):
        control = consensus_control_input(network=comm_network,
                                          agent_states=curr_state,
                                          desired_deviations=desired_deviations,
                                          agent_selection=agent_idx,
                                          gain = consensus_gain).flatten()

        state_derivative[agent_idx] = single_agent_state_derivative(A,
                                                                    B,
                                                                    state = curr_state[agent_idx],
                                                                    control = control).flatten()

    return state_derivative.flatten()


num_steps = int(sim_time/sim_dt) + 1
timestamps = np.arange(num_steps)*sim_dt
system_states = np.zeros((num_steps, num_agents, state_dim))
system_states[0] = agent_states

for sim_idx in range(num_steps - 1):

    rk45_solver = scipy.integrate.RK45(fun = sim_dynamics_update,
                                       t0 = timestamps[sim_idx],
                                       t_bound = timestamps[sim_idx] + sim_dt,
                                       y0 = system_states[sim_idx].flatten())

    while rk45_solver.status == "running":
        rk45_solver.step()

    system_states[sim_idx + 1] = rk45_solver.y.reshape((num_agents, state_dim))


## Viewing the Sim ##
frame_rate = 20  # Frames per second
real_time_playback = 1  # Set to 2 for 2x speed
view_edge_buffer = 1

save_animation = True
save_name = "test.mp4"


samples_per_frame = int((1/(frame_rate*sim_dt))*real_time_playback)

#get plotting data from the simulation
x_data = system_states[:, :, 0]
y_data = system_states[:, :, 1]



fig, ax, agent_points, agent_goal_circles = plot_sim_frame(system_states[0], desired_deviations)


#animation helper functions
def ani_init():
    
    for agent_idx in range(len(agent_points)):
        agent_points[agent_idx].set_data([], [])
        agent_goal_circles[agent_idx].set_center([0, 0])

    return agent_points + agent_goal_circles


def ani_update(frame):

    idx = frame*samples_per_frame

    curr_states = system_states[idx]

    for agent_idx in range(len(agent_points)):
        agent_points[agent_idx].set_data([curr_states[agent_idx, 0]], [curr_states[agent_idx, 1]])
        agent_goal_circles[agent_idx].set_center([desired_deviations[agent_idx, 0] + curr_states[0, 0], desired_deviations[agent_idx, 1] + curr_states[0, 1]])


    #iterative update the axis limits
    x_lower = np.min(curr_states[:, 0]) - view_edge_buffer
    x_upper = np.max(curr_states[:, 0]) + view_edge_buffer
    y_lower = np.min(curr_states[:, 1]) - view_edge_buffer
    y_upper = np.max(curr_states[:, 1]) + view_edge_buffer

    ax.set_xlim(x_lower, x_upper)
    ax.set_ylim(y_lower, y_upper)

    return agent_points + agent_goal_circles


num_frames = num_frames = len(system_states) // samples_per_frame
ani = FuncAnimation(fig, ani_update, frames = num_frames, init_func=ani_init, blit = True)

if save_animation:
    ani.save(save_name, writer=FFMpegWriter(fps=frame_rate))
    print(f"Animation saved as {save_name}")
else:
    plt.show()



