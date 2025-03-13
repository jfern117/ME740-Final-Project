#PyQT5 Depdendencies
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QWidget, QTabWidget, QPushButton #ui elements
from PyQt5.QtWidgets import QVBoxLayout #layouts
from PyQt5.QtCore import QSize, Qt
import pyqtgraph as pg 
import sys


#see tutorial at https://www.pythonguis.com/tutorials/plotting-pyqtgraph/
class sim_tab(QWidget):

    def __init__(self, parent):
        super().__init__(parent)

        self.main_app = parent

        self.tab_layout = QVBoxLayout()
        # self.label = QLabel("Rx: ")
        # self.tab_layout.addWidget(self.label)
        self.setLayout(self.tab_layout)

        #Placeholder for now, starting agent states
        starting_offset = 0.5*np.sqrt(2)/2
        num_agents = 5
        state_dim = 4
        agent_states = np.zeros((num_agents, state_dim))
        agent_states[0] = np.array([0, 0, 0, 0])
        agent_states[1] = np.array([0.5, 0, 0, 0])
        agent_states[2] = np.array([starting_offset, -starting_offset, 0, 0])
        agent_states[3] = np.array([0, -0.5, 0, 0])
        agent_states[4] = np.array([-starting_offset, -starting_offset, 0, 0])

        self.agent_states = agent_states

        #Placeholder desired agent states
        desired_deviations = np.zeros([num_agents, state_dim])
        desired_deviations[0] = np.zeros(4)
        desired_deviations[1] = np.array([1, 0, 0 ,0])
        desired_deviations[2] = np.array([0, 1, 0, 0])
        desired_deviations[3] = np.array([-1, 0, 0, 0])
        desired_deviations[4] = np.array([0, -1, 0, 0])

        self.desired_deviations = desired_deviations

        #adding initial formation + testing
        init_formation = formation(self.desired_deviations, description="Default formation: cross")
        self.main_app.formation_list.append(init_formation)

        #this is going to be the view graph until we get a FPV
        self.central_view = pg.PlotWidget()
        self.tab_layout.addWidget(self.central_view)
        
        #setup the plot
        self.central_view.setBackground("w")
        self.central_view.setMouseEnabled(x=False, y=False) #disable scrolling + zooming on the plot (we're gonna manage that)

        self.agent_plot_list = []
        self.agent_goal_list = []
        self.color_list = ["g", "r", "b", "k", "m" ]
        self.goal_radius = 0.1
        for agent_idx in range(len(self.agent_states)):

            #current state
            curr_state = self.agent_states[agent_idx]
            self.agent_plot_list.append(self.central_view.plot([curr_state[0]], [curr_state[1]],
                                                               symbol = "o",
                                                               symbolBrush=self.color_list[agent_idx]))
            
            #desired deviations
            curr_desired_state = self.desired_deviations[agent_idx] + self.agent_states[0]
            curr_goal_circle = pg.QtWidgets.QGraphicsEllipseItem(curr_desired_state[0] - self.goal_radius,
                                                                 curr_desired_state[1] - self.goal_radius,
                                                                 2*self.goal_radius,
                                                                 2*self.goal_radius)
            #best place I could find pen info:
            #https://pyqtgraph.readthedocs.io/en/latest/_modules/pyqtgraph/functions.html#mkPen
            curr_pen = pg.mkPen(self.color_list[agent_idx])
            curr_pen.setStyle(Qt.DashLine)
            curr_goal_circle.setPen(curr_pen)
            self.agent_goal_list.append(curr_goal_circle)
            self.central_view.addItem(self.agent_goal_list[agent_idx])


    def init_agent_plots(self):
        """
        """
        for agent_idx in range(len(self.agent_states)):

            #current state
            curr_state = self.agent_states[agent_idx]
            self.agent_plot_list.append(self.central_view.plot([curr_state[0]], [curr_state[1]],
                                                               symbol = "o",
                                                               symbolBrush=self.color_list[agent_idx]))
            
            #desired deviations
            curr_desired_state = self.desired_deviations[agent_idx] + self.agent_states[0]
            curr_goal_circle = pg.QtWidgets.QGraphicsEllipseItem(curr_desired_state[0] - self.goal_radius,
                                                                 curr_desired_state[1] - self.goal_radius,
                                                                 2*self.goal_radius,
                                                                 2*self.goal_radius)
            #best place I could find pen info:
            #https://pyqtgraph.readthedocs.io/en/latest/_modules/pyqtgraph/functions.html#mkPen
            curr_pen = pg.mkPen(self.color_list[agent_idx])
            curr_pen.setStyle(Qt.DashLine)
            curr_goal_circle.setPen(curr_pen)
            self.agent_goal_list.append(curr_goal_circle)
            self.central_view.addItem(self.agent_goal_list[agent_idx])

    def update_agent_plots(self):
        """
        """
        


class formation_tab(QWidget):

    def __init__(self, parent):
        super().__init__(parent)

        self.tab_layout = QVBoxLayout()
        self.button = QPushButton("Press me!") #this is a placeholder for future useful stuff
        self.tab_layout.addWidget(self.button)
        self.setLayout(self.tab_layout)

#TODO: set this one up later
class setting_tab(QWidget):
    pass

#https://www.pythonguis.com/tutorials/creating-your-first-pyqt-window/
class gui_app(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Control GUI")

        self.formation_list = []

        self.tab_list = [sim_tab(self), formation_tab(self)]

        #the tabs contain our primary gui elements, so we create a tab widget and add our tabs to it
        self.tab_widget = QTabWidget()
        self.setCentralWidget(self.tab_widget)

        #save the setting tab for later
        self.tab_widget.addTab(self.tab_list[0], "Control")
        self.tab_widget.addTab(self.tab_list[1], "Formations")




class formation():
    def __init__(self, deviations, description = ""):
        self.agent_deviations = deviations
        self.description = description

