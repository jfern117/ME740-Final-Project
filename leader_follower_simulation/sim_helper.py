import numpy as np
import networkx as nx
import matplotlib.pyplot as plt


def plot_sim_frame(agent_states, desired_deviation, radius = 0.1):
    """
    """

    fig, ax = plt.subplots(1)


    num_agents = len(agent_states)
    agents = []
    goals = []
    for agent_idx in range(num_agents):
        line = ax.plot(agent_states[agent_idx, 0], agent_states[agent_idx, 1], marker = "o", label = f"Agent {agent_idx}")[0]
        agents.append(line)
        
        #create the circle, offset from the leader agent's position
        circle = plt.Circle(agent_states[0, 0:2] + desired_deviation[agent_idx, 0:2], radius=radius,
                            edgecolor = line.get_color(),
                            facecolor = "none",
                            linestyle = "--")
        ax.add_patch(circle)

        goals.append(circle)

    ax.legend(handles = agents)
    
    return fig, ax, agents, goals

def consensus_control_input(network:nx.Graph, agent_states, desired_deviations, agent_selection, gain):
    """
    """

    num_agents, state_dim = np.shape(agent_states)

    #the summation is effectively done over the neighbor set
    neighbor_list = list(network.neighbors(agent_selection))

    #this is the method done in toyota. If it works, we'll try the version where the deviations are summed out as a constant bias term
    offset_states = agent_states - desired_deviations

    consensus_sum = np.zeros(state_dim)

    for neighbor_idx in range(len(neighbor_list)):
        consensus_sum += offset_states[agent_selection] - offset_states[neighbor_idx]

    control_input = -gain @ consensus_sum.reshape((-1,1))

    return control_input


def single_agent_state_derivative(A, B, state, control):
    """
    """
    state_dot = A @ state.reshape((-1,1)) + B @ control.reshape((-1,1))
    return state_dot


