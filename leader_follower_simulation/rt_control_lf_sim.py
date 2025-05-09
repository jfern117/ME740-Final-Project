import numpy as np
import networkx as nx
import scipy.integrate
import scipy.signal
from copy import deepcopy

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

#add a list of lists to track the last 1000 states for drawing
trailing_traj = [list() for idx in range(num_agents)]
max_traj_track = 500

#each agent has a four element state: [x, y, xdot, ydot]. Here are the starting states
starting_offset = np.sqrt(2)/2
agent_states = np.zeros((num_agents, state_dim))
agent_states[0] = np.array([0, 0, 0, 0])
agent_states[1] = np.array([1, 0, 0, 0])
agent_states[2] = np.array([starting_offset, -starting_offset, 0, 0])
agent_states[3] = np.array([0, -1, 0, 0])
agent_states[4] = np.array([-starting_offset, -starting_offset, 0, 0])

# Each agent need a designated deviation from the leader
formation_list = []
formation_selection = 0
desired_deviations = np.zeros([num_agents, state_dim])
desired_deviations[0] = np.zeros(4)

#Formation 0: cross
desired_deviations[1] = np.array([1, 0, 0 ,0])
desired_deviations[2] = np.array([0, 1, 0, 0])
desired_deviations[3] = np.array([-1, 0, 0, 0])
desired_deviations[4] = np.array([0, -1, 0, 0])
formation_list.append(deepcopy(desired_deviations))

#Formation 1: (Vertical) Line
desired_deviations[1] = np.array([0, 1, 0, 0])
desired_deviations[2] = np.array([0, 2, 0, 0])
desired_deviations[3] = np.array([0, -1, 0 ,0])
desired_deviations[4] = np.array([0, -2, 0, 0])
formation_list.append(deepcopy(desired_deviations))

#Formation 2: V formation (leader front)
desired_deviations[1] = np.array([1, -1, 0, 0])
desired_deviations[2] = np.array([0.5, -0.5, 0, 0])
desired_deviations[3] = np.array([-0.5, -0.5, 0 ,0])
desired_deviations[4] = np.array([-1, -1, 0, 0])
formation_list.append(deepcopy(desired_deviations))

#dynamics equations - double integrator
A = np.zeros((state_dim, state_dim))
A[0, 2] = 1
A[1, 3] = 1

B = np.zeros((state_dim, 2))
B[2, 0] = 1
B[3, 1] = 1

#consensus gain (based on toyota's paper)
gamma0 = 2
gamma1 = 2
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

    damping_constant = 2.5
    curr_xdot = leader_state[2]
    curr_ydot = leader_state[3]

    #handle horizontal control input
    left = keys[pygame.K_LEFT]
    right = keys[pygame.K_RIGHT]

    #we apply the max accel input if (1) it counteracts your current direction of movement or (2) you are less than the current max speed
    speed_check = lambda dir, v, vmax: np.sign(v) != dir or np.abs(v) < vmax

    if left:
        u_H = -max_speed/accel_time if speed_check(-1, curr_xdot, max_speed) else 0
    elif right:
        u_H = max_speed/accel_time if speed_check(1, curr_xdot, max_speed) else 0
    else:
        u_H = -damping_constant*curr_xdot

    #handle vertical control input
    up = keys[pygame.K_UP]
    down = keys[pygame.K_DOWN]

    if down:
        u_V = -max_speed/accel_time if speed_check(-1, curr_ydot, max_speed) else 0
    elif up:
        u_V = max_speed/accel_time if speed_check(1, curr_ydot, max_speed) else 0
    else:
        u_V = -damping_constant*curr_ydot

    control = np.array([u_H, u_V])

    return control



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
                                          desired_deviations=formation_list[formation_selection],
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
# color = (0, 255, 0)
pygame.init()
screen = pygame.display.set_mode((width, height))
pix_per_meter = 100
w_offset = width/2
h_offset = height/2
clock = pygame.time.Clock()



def get_agent_display_position(agent_position):
    """
    """
    agent_pos_pix = (agent_position[0:2]*pix_per_meter).astype(int)
    agent_pos_pix[0] += w_offset
    agent_pos_pix[1] += h_offset
    agent_pos_pix[1] = height - agent_pos_pix[1]

    return agent_pos_pix


running = True
colors = [pygame.Color("green"), pygame.Color("red"), pygame.Color("blue"), pygame.Color("black"), pygame.Color("purple") ]
prev_formation_press = None

while running:

    #clear the screen (make it white)
    screen.fill((255, 255, 255))

    #stop the sim when a quit event is processed
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            print("Quitting Game")
            running = False


    keys = pygame.key.get_pressed()
    if prev_formation_press is None:
        prev_formation_press = keys[pygame.K_x]

    if keys[pygame.K_x] and not prev_formation_press:
        formation_selection = (formation_selection + 1) % len(formation_list)
    
    prev_formation_press = keys[pygame.K_x]
        

    rk45_solver = scipy.integrate.RK45(fun = sim_dynamics_update,
                                       t0 = 0,
                                       t_bound = frame_dt,
                                       y0 = agent_states.flatten())

    while rk45_solver.status == "running":
        rk45_solver.step()

    agent_states = rk45_solver.y.reshape((num_agents, state_dim))

    for agent_idx in range(num_agents):

        curr_state = agent_states[agent_idx]
        agent_pos_pixels = get_agent_display_position(curr_state)
        pygame.draw.circle(screen, colors[agent_idx], agent_pos_pixels, radius)

        #draw the desired goal
        curr_goal = formation_list[formation_selection][agent_idx] + agent_states[0]
        goal_pos_pixels = get_agent_display_position(curr_goal)
        pygame.draw.circle(screen, colors[agent_idx], goal_pos_pixels, radius*3, 2)

        #update the agent's trailing trajectory
        trailing_traj[agent_idx].append((agent_pos_pixels[0], agent_pos_pixels[1]))
        if len(trailing_traj[agent_idx]) > max_traj_track:
            trailing_traj[agent_idx].pop(0) #remove the earliest element
        
        if len(trailing_traj[agent_idx]) >= 2:
            pygame.draw.lines(screen, colors[agent_idx], False, trailing_traj[agent_idx])


    pygame.display.update()
    clock.tick(frame_rate)
