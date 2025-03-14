import pyqtgraph as pg
import numpy as np
from PyQt5.QtCore import QSize, Qt

#We have two, very similar plots, so we can 
def init_plot(plot:pg.PlotWidget, agent_states, desired_deviations, color_list, goal_radius):
    """
    Inputs:
        - plot: the pyqtgraph plotting widget with which to plot
        - agent_states: a numpy array of shape (num agents, num dimensions) containing the state of the agents.
                        The first two dimensions are assumed to be x, y for the purposes of plotting
        - desired_deviations: a numpy array of shape (num agents, num dimensions) containing the deviations of agents from
                              the leader agent. It is assumed that the leader is agent 0
    """

    agent_plot_list = []
    agent_goal_list = []
    for agent_idx in range(len(agent_states)):

        #current state
        curr_state = agent_states[agent_idx]
        agent_plot_list.append(plot.plot([curr_state[0]], [curr_state[1]],
                                                            symbol = "o",
                                                            symbolBrush=color_list[agent_idx]))
        
        #desired deviations
        curr_desired_state = desired_deviations[agent_idx] + agent_states[0]
        curr_goal_circle = pg.QtWidgets.QGraphicsEllipseItem(curr_desired_state[0] - goal_radius,
                                                                curr_desired_state[1] - goal_radius,
                                                                2*goal_radius,
                                                                2*goal_radius)
        #best place I could find pen info:
        #https://pyqtgraph.readthedocs.io/en/latest/_modules/pyqtgraph/functions.html#mkPen
        curr_pen = pg.mkPen(color_list[agent_idx])
        curr_pen.setStyle(Qt.DashLine)
        curr_goal_circle.setPen(curr_pen)
        agent_goal_list.append(curr_goal_circle)
        plot.addItem(agent_goal_list[agent_idx])

    return agent_plot_list, agent_goal_list

def update_plot(plot:pg.PlotWidget, agent_plot_list, goal_plot_list, agent_states, deviation_states, goal_radius):
    """
    """
    for agent_idx in range(len(agent_plot_list)):
        curr_plot = agent_plot_list[agent_idx]
        curr_state = agent_states[agent_idx]
        curr_plot.setData([curr_state[0]], [curr_state[1]])

        if len(goal_plot_list) > 0:
            curr_goal_plot = goal_plot_list[agent_idx]
            goal_state = deviation_states[agent_idx] + agent_states[0]
            curr_goal_plot.setRect(goal_state[0] - goal_radius, 
                                    goal_state[1] - goal_radius,
                                    2*goal_radius,
                                    2*goal_radius)