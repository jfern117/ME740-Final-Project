#ROS Depdendencies
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

#PyQT5 Depdendencies
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QWidget, QTabWidget, QPushButton #ui elements
from PyQt5.QtWidgets import QVBoxLayout #layouts
from PyQt5.QtCore import QSize, Qt
import pyqtgraph as pg 
import sys

#Other requirements
from threading import Thread #need to run both GUI + ros at same time

## ROS Node ##
class MinimalSubscriber(Node):

    def __init__(self, gui_app):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.app = gui_app

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        # self.app.tab_list[0].label.setText(f"Rx: {msg.data}")

## GUI Setup ##

#see tutorial at https://www.pythonguis.com/tutorials/plotting-pyqtgraph/
class sim_tab(QWidget):

    def __init__(self, parent):
        super().__init__(parent)

        self.tab_layout = QVBoxLayout()
        # self.label = QLabel("Rx: ")
        # self.tab_layout.addWidget(self.label)
        self.setLayout(self.tab_layout)

        #Placeholder for now, starting agent states
        starting_offset = np.sqrt(2)/2
        num_agents = 5
        state_dim = 4
        agent_states = np.zeros((num_agents, state_dim))
        agent_states[0] = np.array([0, 0, 0, 0])
        agent_states[1] = np.array([1, 0, 0, 0])
        agent_states[2] = np.array([starting_offset, -starting_offset, 0, 0])
        agent_states[3] = np.array([0, -1, 0, 0])
        agent_states[4] = np.array([-starting_offset, -starting_offset, 0, 0])

        self.agent_states = agent_states

        #this is going to be the view graph until we get a FPV
        self.central_view = pg.PlotWidget()
        self.tab_layout.addWidget(self.central_view)
        
        #setup the plot
        self.central_view.setBackground("w")
        self.central_view.setMouseEnabled(x=False, y=False) #disable scrolling + zooming on the plot (we're gonna manage that)

        self.agent_plot_list = []
        self.color_list = ["g", "r", "b", "k", "m" ]
        for agent_idx in range(len(self.agent_states)):
            curr_state = self.agent_states[agent_idx]
            self.agent_plot_list.append(self.central_view.plot([curr_state[0]], [curr_state[1]],
                                                               symbol = "o",
                                                               symbolBrush=self.color_list[agent_idx]))
            
    def plot_agents(self):
        pass


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

        self.tab_list = [sim_tab(self), formation_tab(self)]

        #the tabs contain our primary gui elements, so we create a tab widget and add our tabs to it
        self.tab_widget = QTabWidget()
        self.setCentralWidget(self.tab_widget)

        #save the setting tab for later
        self.tab_widget.addTab(self.tab_list[0], "Control")
        self.tab_widget.addTab(self.tab_list[1], "Formations")
        

def main(args=None):
    rclpy.init(args=args)

    

    app = QApplication([])
    window = gui_app()
    window.show()


    #setup the ROS node and the GUI
    minimal_subscriber = MinimalSubscriber(window)

    #setup the thread to run the ros messaging
    def ros_thread_func():
        rclpy.spin(minimal_subscriber)
    
    #since this is a daemon, it auto-terminates when the program finishes
    ros_ctrl_thread = Thread(target=ros_thread_func, daemon=True)
    ros_ctrl_thread.start()
    
    #run the GUI loop in the main program
    app.exec()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()