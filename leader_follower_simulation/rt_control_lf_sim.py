import numpy as np
import networkx as nx
import scipy.integrate
import matplotlib.pyplot as plt
import scipy.signal

import pygame


from sim_helper import consensus_control_input, single_agent_state_derivative

#suppose we have N + 1 vehicles in a 2D plane with second order integrator dynamics (p = v, vdot = u)

#Game stuff
frame_rate = 20
frame_dt = 1/frame_rate

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
agent_states[0] = np.array([0, 0, 0, 0])
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

#global reference to current keys
keys = []
max_speed = 0.5 #m/s
accel_time = 1.0

def get_control(leader_state):
    """
    """

    damping_constant = 0.001
    curr_xdot = leader_state[2]
    curr_ydot = leader_state[3]

    #handle horizontal control input
    left = keys[pygame.K_LEFT]
    right = keys[pygame.K_RIGHT]

    speed_check = lambda dir, v, vmax: np.sign(v).item() == dir and np.abs(v) < vmax


    #TODO: there's an issue with speed check?
    if left:
        u_H = -max_speed/accel_time #-max_speed/accel_time if speed_check(-1, curr_xdot, max_speed) else 0
    elif right:
        u_H = max_speed/accel_time# max_speed/accel_time if speed_check(1, curr_xdot, max_speed) else 0
    else:
        u_H = 0#-damping_constant*curr_xdot

    #handle vertical control input
    up = keys[pygame.K_UP]
    down = keys[pygame.K_DOWN]

    if down:
        u_V = -max_speed/accel_time if speed_check(-1, curr_ydot, max_speed) else 0
    elif up:
        u_V = max_speed/accel_time if speed_check(1, curr_ydot, max_speed) else 0
    else:
        u_V = -damping_constant*curr_xdot

    print(np.array([u_H, u_V]))

    return np.array([u_H, u_V])



## Simulation ##
def sim_dynamics_update(t, x):
    """
    """
    #assuming that we are flattening our input and output
    curr_state = x.reshape(num_agents, state_dim)

    #prepare our output variable
    state_derivative = np.zeros(np.shape(curr_state))

    #calculate the change in the leader separately
    user_control = get_control(curr_state[0])
    state_derivative[0] = single_agent_state_derivative(A, B, curr_state[0], control = user_control ).flatten()

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



#setup the game
width, height = 800, 800
radius = 5
color = (0, 255, 0)
pygame.init()
screen = pygame.display.set_mode((width, height))
pix_per_meter = 100
offset = 400
clock = pygame.time.Clock()


running = True

while running:

    #clear the screen (make it white)
    screen.fill((255, 255, 255))

    #stop the sim when a quit event is processed
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            print("Quitting Game")
            running = False


    keys = pygame.key.get_pressed()

    rk45_solver = scipy.integrate.RK45(fun = sim_dynamics_update,
                                       t0 = 0,
                                       t_bound = frame_dt,
                                       y0 = agent_states.flatten())

    while rk45_solver.status == "running":
        rk45_solver.step()

    agent_states = rk45_solver.y.reshape((num_agents, state_dim))

    leader_state = agent_states[0]

    leader_pos_pix = (leader_state[0:2]*pix_per_meter + offset).astype(int)



    pygame.draw.circle(screen, color, leader_pos_pix, radius)

    pygame.display.update()
    clock.tick(frame_rate)


    

    





# num_steps = int(sim_time/sim_dt) + 1
# timestamps = np.arange(num_steps)*sim_dt
# system_states = np.zeros((num_steps, num_agents, state_dim))
# system_states[0] = agent_states

# for sim_idx in range(num_steps - 1):

#     rk45_solver = scipy.integrate.RK45(fun = sim_dynamics_update,
#                                        t0 = timestamps[sim_idx],
#                                        t_bound = timestamps[sim_idx] + sim_dt,
#                                        y0 = system_states[sim_idx].flatten())

#     while rk45_solver.status == "running":
#         rk45_solver.step()

#     system_states[sim_idx + 1] = rk45_solver.y.reshape((num_agents, state_dim))
