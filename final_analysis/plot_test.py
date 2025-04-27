import numpy as np
import matplotlib.pyplot as plt


data = np.load("agent_states.npy")
num_agents = 5

goal_positions_1 = np.zeros((num_agents, 2))
goal_positions_1[1] = np.array([2.5, 0.0])
goal_positions_1[2] = np.array([0.0, 2.5])
goal_positions_1[3] = np.array([-2.5, 0.0])
goal_positions_1[4] = np.array([0.0, -2.5])

goal_positions_2 = np.zeros((num_agents, 2))
goal_positions_2[1] = np.array([2, 2])
goal_positions_2[2] = np.array([-2, 2])
goal_positions_2[3] = np.array([-2, -2])
goal_positions_2[4] = np.array([2, -2])

plot_1_list = []
goal_1_list = []
plt.figure(figsize=(10,10))
for agent_idx in range(num_agents):

    x_data = data[0:len(data)//2, agent_idx, 0]
    y_data = data[0:len(data)//2, agent_idx, 1]

    line = plt.plot(x_data, y_data, label = f"Agent {agent_idx}")[0]
    plot_1_list.append(line)

    #add the goal positon
    goal = plt.plot(goal_positions_1[agent_idx, 0] + data[len(data)//2, 0, 0], goal_positions_1[agent_idx, 1] + data[len(data)//2, 0, 1], "x", label = "Goal Position", c = line.get_color())[0]
    goal_1_list.append(goal)

handles_1 = [goal_1_list[0]] + plot_1_list
plt.title("Agent State Plots (First Half - Cross Formation)")
plt.xlabel("x Position (m)")
plt.ylabel("y Position (m)")
plt.legend(handles = handles_1)
plt.show()
plt.close()

plot_2_list = []
goal_2_list = []
plt.figure(figsize=(10,10))
for agent_idx in range(num_agents):

    x_data = data[len(data)//2:, agent_idx, 0]
    y_data = data[len(data)//2:, agent_idx, 1]

    line = plt.plot(x_data, y_data, label = f"Agent {agent_idx}")[0]
    plot_2_list.append(line)

    #add the goal positon
    goal = plt.plot(goal_positions_2[agent_idx, 0] + data[-1, 0, 0], goal_positions_2[agent_idx, 1] + data[-1, 0, 1], "x", label = "Goal Position", c = line.get_color())[0]
    goal_2_list.append(goal)

handles_2 = [goal_2_list[0]] + plot_2_list
plt.title("Agent State Plots (Second Half - Square Formation)")
plt.xlabel("x Position (m)")
plt.ylabel("y Position (m)")
plt.legend(handles = handles_2)
plt.show()