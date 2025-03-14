import numpy as np
import scipy.integrate
import networkx as nx

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


def sim_update(A, B, agent_states, user_control, network, desired_deviations, consensus_gain, dt):
    """
    """

    num_agents, state_dim = np.shape(agent_states)

    #helper function for use in the RK45 simualtor
    def sim_dynamics_update(t, x):

        #assuming that we are flattening our input and output
        curr_state = x.reshape(num_agents, state_dim)

        #prepare our output variable
        state_derivative = np.zeros(np.shape(curr_state))

        #calculate the change in the leader separately
        state_derivative[0] = single_agent_state_derivative(A, B, curr_state[0], control = user_control ).flatten()

        #compute the dynamic updates for all the followers
        for agent_idx in range(1, num_agents):
            control = consensus_control_input(network=network,
                                            agent_states=curr_state,
                                            desired_deviations=desired_deviations,
                                            agent_selection=agent_idx,
                                            gain = consensus_gain).flatten()

            state_derivative[agent_idx] = single_agent_state_derivative(A,
                                                                        B,
                                                                        state = curr_state[agent_idx],
                                                                        control = control).flatten()

        return state_derivative.flatten()
    

    rk45_solver = scipy.integrate.RK45(fun = sim_dynamics_update,
                                       t0 = 0,
                                       t_bound = dt,
                                       y0 = agent_states.flatten())
    
    while rk45_solver.status == "running":
        rk45_solver.step()

    updated_agent_states = rk45_solver.y.reshape((num_agents, state_dim))

    return updated_agent_states

    


