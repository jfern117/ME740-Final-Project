#PyQT5 Depdendencies
from PyQt5.QtWidgets import QMainWindow, QLabel, QWidget, QTabWidget, QPushButton, QTableWidget, QTableWidgetItem, QComboBox, QLineEdit, QTextEdit #ui elements
from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout, QGridLayout, QStackedLayout #layouts
from PyQt5.QtCore import Qt, pyqtSignal, QObject
from PyQt5.QtGui import QColor, QImage, QPixmap
import pyqtgraph as pg 
from swarm_gui.agent_plot_helper import init_plot, update_plot
from swarm_gui.messaging_helper import msg_to_array, array_to_msg

#used to save the formations
import yaml
import tkinter as tk
from tkinter import filedialog

#other dependencies
import numpy as np
import cv2




#see tutorial at https://www.pythonguis.com/tutorials/plotting-pyqtgraph/
class sim_tab(QWidget):

    #If the plot is updated in a different thread, the GUI will very quickly crash. Here is a fix that resolves that: https://stackoverflow.com/questions/64307813/pyqtgraph-stops-updating-and-freezes-when-grapahing-live-sensor-data
    class plot_signaler(QObject):
        plot_update_signal = pyqtSignal(int)

    def __init__(self, parent):
        super().__init__(parent)

        #need a static reference to the shared main app state
        self.main_app = parent

        #we're setting up a stacked layout to be able to have a FPV view with an overlaid minimap
        self.main_layout = QGridLayout(self)
        self.setLayout(self.main_layout)

        #add the background image layer
        self.fpv_view_label = QLabel()
        self.fpv_view_label.setScaledContents(True)
        self.main_layout.addWidget(self.fpv_view_label, 1, 0, 2, 3)
        
        #add the overlaid minimap layer
        self.overlay_widget = QWidget(self)
        self.overlay_widget.setAttribute(Qt.WA_TransparentForMouseEvents)
        self.overlay_widget.setStyleSheet("background: transparent;")
        self.overlay_layout = QVBoxLayout()
        self.overlay_layout.setContentsMargins(0, 0, 0, 0) #sets the margins of the upper corners (I think this makes it flush?)
        self.overlay_layout.setAlignment(Qt.AlignTop | Qt.AlignRight) #put our minimap in the upper right

        self.plot_view = pg.PlotWidget() #this is our minimap!
        self.plot_view.setFixedSize(200,200)
        self.overlay_layout.addWidget(self.plot_view)
        self.overlay_widget.setLayout(self.overlay_layout)

        self.main_layout.addWidget(self.overlay_widget, 0, 2)

        #add two info tabs
        self.left_info_tab_layout = QVBoxLayout()
        self.curr_formation_label = QLabel()
        self.left_info_tab_layout.addWidget(self.curr_formation_label)
        self.main_layout.addLayout(self.left_info_tab_layout, 0, 0)
        
        #setup the plot
        self.plot_view.setBackground("w")
        self.plot_view.setMouseEnabled(x=False, y=False) #disable scrolling + zooming on the plot (we're gonna manage that)
        self.plot_view.showGrid(x=True, y= True)
        self.plot_view.setAspectLocked(True)

        self.agent_plot_list = []
        self.agent_goal_list = []
        self.goal_radius = 0.1
        self.init_agent_plots()

        self.signal_object = self.plot_signaler()
        self.signal_object.plot_update_signal.connect(self.update_agent_plots)


    def init_agent_plots(self):
        """
        """
        self.agent_plot_list, self.agent_goal_list = init_plot(self.plot_view,
                                                               self.main_app.agent_states,
                                                               self.main_app.get_agent_deviations(),
                                                               self.main_app.color_list,
                                                               self.goal_radius)

    def update_agent_states(self, updated_agent_states):
        #update stuff provided by the function call
        self.main_app.agent_states = updated_agent_states


    def update_agent_plots(self, int_value):
        """
        """

        #giving the option to add cases where this isnt' plotted
        if not int_value:
            return

        #update if this is the selected window (trying to fix bug with udpating plots)
        if self.main_app.tab_widget.currentIndex() == 0:

            update_plot(self.plot_view, 
                        self.agent_plot_list, 
                        self.agent_goal_list, 
                        self.main_app.agent_states, 
                        self.main_app.get_agent_deviations(), 
                        self.goal_radius)
            
            self.update_camera_view()
            self.curr_formation_label.setText(f"Current formation: {self.main_app.formation_list[self.main_app.selected_formation].name}")

    def update_camera_view(self):
        """
        """
        
        #lets just test how the layout looks + the method
        frame = np.random.randint(0, 255, (480, 640, 3), dtype = np.uint8)

        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) #this is random, why do this?

        height, width, channel = np.shape(rgb_image)
        bytes_per_line = channel*width #each element is a byte
        Qimg = QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(Qimg)

        self.fpv_view_label.setPixmap(pixmap)




#helper class for the formation tab. Will likely be useful for later    
class formation():
    def __init__(self, deviations, name = "", description = ""):
        self.agent_deviations = deviations
        self.description = description
        self.name = name

class formation_tab(QWidget):

    def __init__(self, parent):
        super().__init__(parent)

        self.main_app:gui_app = parent

        self.top_level_tab_layout = QHBoxLayout()

        #create a plot to show the formation being designed
        self.formation_view_plot = pg.PlotWidget()
        self.top_level_tab_layout.addWidget(self.formation_view_plot)
        self.agent_plot_list = []
        self.init_plots()

        #add GUI elements on the right side
        self.right_side_layout = QVBoxLayout()

        self.formation_dropdown = QComboBox(self)
        self.update_dropdown(update=False) #populate the drop down
        self.formation_dropdown.currentIndexChanged.connect(self.formation_selection_updated)
        
        #intiailize the name text input
        self.name_layout = QHBoxLayout()
        self.name_label = QLabel("Name:")
        self.formation_name = QLineEdit(self)
        self.name_layout.addWidget(self.name_label)
        self.name_layout.addWidget(self.formation_name)

        #setup the description text input
        self.description_layout = QHBoxLayout()
        self.description_label = QLabel("Description:")
        self.formation_description = QTextEdit(self)
        self.description_layout.addWidget(self.description_label)
        self.description_layout.addWidget(self.formation_description)

        #setup the table with the agent colors
        self.num_cols = 3
        self.formation_table = QTableWidget(self.main_app.num_agents, self.num_cols, parent = self)
        self.formation_table.setHorizontalHeaderLabels(["Agent Color", "X", "Y"])
        self.formation_table.setVerticalHeaderLabels([str(element) for element in range(self.main_app.num_agents)])
        for agent_idx in range(self.main_app.num_agents):
            item = QTableWidgetItem("")
            item.setFlags(item.flags() & ~Qt.ItemIsEditable)
            item.setBackground(QColor(self.main_app.color_list[agent_idx]))
            self.formation_table.setItem(agent_idx, 0, item)

        #set the leader agent state and don't make it editable
        for col_idx in range(1, self.num_cols):
            item = QTableWidgetItem("0.0")
            item.setFlags(item.flags() & ~Qt.ItemIsEditable)
            self.formation_table.setItem(0, col_idx, item)

        self.save_load_layout = QHBoxLayout()
        self.save_formations_button = QPushButton("Save Formations", self)
        self.save_formations_button.clicked.connect(self.save_formations_to_disk)
        self.load_formations_button = QPushButton("Load Formations", self)
        self.load_formations_button.clicked.connect(self.load_formations_from_disk)
        self.save_load_layout.addWidget(self.save_formations_button)
        self.save_load_layout.addWidget(self.load_formations_button)
        
        #load in the formation data
        self.load_formation(self.main_app.formation_list[self.main_app.selected_formation])

        #make update cell function call
        self.formation_table.cellChanged.connect(self.table_cell_updated)

        #add the push button to add new formations
        self.formation_update_button = QPushButton("Update")
        self.formation_update_button.clicked.connect(self.save_or_update_formation_clicked)
        
        #add all the widgets
        self.right_side_layout.addWidget(self.formation_dropdown)
        self.right_side_layout.addLayout(self.name_layout)
        self.right_side_layout.addLayout(self.description_layout)
        self.right_side_layout.addWidget(self.formation_table)
        self.right_side_layout.addWidget(self.formation_update_button)
        self.right_side_layout.addLayout(self.save_load_layout)

        self.top_level_tab_layout.addLayout(self.right_side_layout)

        #setup the plot
        self.formation_view_plot.setBackground("w")
        self.formation_view_plot.setMouseEnabled(x=False, y=False) #disable scrolling + zooming on the plot (we're gonna manage that)
        self.formation_view_plot.showGrid(x=True, y= True)
        self.formation_view_plot.setAspectLocked(True)

        #setup the layout
        self.setLayout(self.top_level_tab_layout)

    def update_dropdown(self, update = True):
        """
        """

        #clear all items
        self.formation_dropdown.clear()

        for formation_idx in range(len(self.main_app.formation_list)):
            self.formation_dropdown.addItem(self.main_app.formation_list[formation_idx].name)

        self.formation_dropdown.addItem("Create Formation") #this one will be used to add new formations

        if update:
            self.formation_dropdown.setCurrentIndex(0)
            self.load_formation(self.main_app.formation_list[0])


    def load_formation(self, formation:formation):
        """
        """
        #set the name line edit
        self.formation_name.setText(formation.name)

        #set the multiline description
        self.formation_description.setText(formation.description)

        #set the data in the table
        for agent_idx in range(1, self.main_app.num_agents):
            x_item = QTableWidgetItem(str(formation.agent_deviations[agent_idx][0]))
            y_item = QTableWidgetItem(str(formation.agent_deviations[agent_idx][1]))
            self.formation_table.setItem(agent_idx, 1, x_item)
            self.formation_table.setItem(agent_idx, 2, y_item)

        self.update_plots(formation)


    def init_plots(self):
        """
        """
        self.agent_plot_list, agent_goal_list = init_plot(self.formation_view_plot,
                                                          self.main_app.get_agent_deviations(selection = 0),
                                                          self.main_app.get_agent_deviations(selection = 0),
                                                          self.main_app.color_list,
                                                          1) #placeholder goal radius
        
        #remove all the agent goals
        for item in agent_goal_list:
            self.formation_view_plot.removeItem(item)
        

    def update_plots(self, formation:formation):
        """
        """

        update_plot(self.formation_view_plot, 
                    self.agent_plot_list, 
                    [], 
                    formation.agent_deviations, 
                    [], 
                    0)


    def table_cell_updated(self, row, col):

        table_data = float(self.formation_table.item(row, col).text())

        agent_selection = row
        update_x = col - 1 == 0

        static_col = col + 1 if update_x else col -1
        static_data = float(self.formation_table.item(row, static_col).text())

        plot_to_update = self.agent_plot_list[agent_selection]
        x_data = table_data if update_x else static_data
        y_data = table_data if not update_x else static_data
        plot_to_update.setData([x_data], [y_data])


    def formation_selection_updated(self, index):
        """
        """

        item_count = self.formation_dropdown.count()

        #TODO: handle new formation setting
        if index != item_count - 1:
            selected_formation = self.main_app.formation_list[index]
            self.formation_update_button.setText("Update")
        else:
            selected_formation = formation(np.zeros((self.main_app.num_agents, self.main_app.state_dim)))
            self.formation_update_button.setText("Save")
        
        self.load_formation(selected_formation)

    def save_or_update_formation_clicked(self):

        def get_data_from_table(table:QTableWidget):
            
            deviations = np.zeros((self.main_app.num_agents, self.main_app.state_dim))

            for agent_idx in range(self.main_app.num_agents):
                deviations[agent_idx][0] = float(self.formation_table.item(agent_idx, 1).text())
                deviations[agent_idx][1] = float(self.formation_table.item(agent_idx, 2).text())

            return deviations
        
        curr_formation = self.formation_dropdown.currentIndex()
        num_formations = self.formation_dropdown.count()

        update_name = self.formation_name.text()
        update_description = self.formation_description.toPlainText()
        update_deviations = get_data_from_table(self.formation_table)

        #handle the case when we're adding a new formation
        if curr_formation == num_formations - 1 :
            new_formation = formation(update_deviations, name = update_name, description= update_description)
            self.main_app.formation_list.append(new_formation)
            self.formation_dropdown.insertItem(num_formations-1, update_name)

        #udpate the formation info
        else:
            self.main_app.formation_list[curr_formation].name = update_name
            self.main_app.formation_list[curr_formation].description = update_description
            self.main_app.formation_list[curr_formation].agent_deviations = update_deviations

    def save_formations_to_disk(self):
        """
        """
        save_dict = {}
        for formation_idx in range(len(self.main_app.formation_list)):
            label = f"formation{formation_idx}"
            curr_formation = self.main_app.formation_list[formation_idx]
            save_dict[label] = {"name":curr_formation.name, 
                                "description":curr_formation.description, 
                                "agent_deviations":curr_formation.agent_deviations.tolist()}
        
        #based on info I find online for getting a save pop up
        # https://stackoverflow.com/questions/75478982/is-there-a-way-to-choose-where-to-save-a-file-via-a-pop-up-window-in-python
        root = tk.Tk()
        root.withdraw()

        save_name = filedialog.asksaveasfilename(defaultextension=".yaml")

        if save_name:
            with open(save_name, "w") as file:
                yaml.dump(save_dict, file)

    def load_formations_from_disk(self):

        #https://docs.python.org/3/library/dialog.html
        root = tk.Tk()
        root.withdraw()

        file_name = filedialog.askopenfilename(defaultextension="*.yaml")

        if file_name:
            with open(file_name, "r") as file:
                data_dictionary = yaml.safe_load(file)

                key_list = list(data_dictionary.keys())
                formation_list = []
                for key_idx in range(len(key_list)):
                    formation_list.append(formation(deviations=np.array(data_dictionary[key_list[key_idx]]["agent_deviations"]),
                                                    name=data_dictionary[key_list[key_idx]]["name"],
                                                    description=data_dictionary[key_list[key_idx]]["description"]))
                self.main_app.formation_list = formation_list
                self.main_app.selected_formation = 0

                self.update_dropdown()

#TODO: set this one up later
class setting_tab(QWidget):
    pass

#https://www.pythonguis.com/tutorials/creating-your-first-pyqt-window/
class gui_app(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Control GUI")
        self.ros_node = None

        #Placeholder for now, starting agent states
        starting_offset = 0.5*np.sqrt(2)/2
        self.num_agents = 5
        self.state_dim = 4

        #Placeholder desired agent states
        desired_deviations = np.zeros((self.num_agents, self.state_dim))
        desired_deviations[0] = np.zeros(4)
        desired_deviations[1] = np.array([1, 0, 0 ,0])
        desired_deviations[2] = np.array([0, 1, 0, 0])
        desired_deviations[3] = np.array([-1, 0, 0, 0])
        desired_deviations[4] = np.array([0, -1, 0, 0])

        #The sim has GT on the agent states, the GUI needs to be provided them
        self.agent_states = np.zeros(np.shape(desired_deviations))

        #adding initial formation + testing
        init_formation = formation(desired_deviations, name = "Cross", description="Default formation: cross")

        self.formation_list = [init_formation]
        self.selected_formation = 0
        
        self.color_list = ["green", "red", "blue", "black", "magenta" ]

        self.tab_list = [sim_tab(self), formation_tab(self)]

        #the tabs contain our primary gui elements, so we create a tab widget and add our tabs to it
        self.tab_widget = QTabWidget()
        self.setCentralWidget(self.tab_widget)

        #save the setting tab for later
        self.tab_widget.addTab(self.tab_list[0], "Control")
        self.tab_widget.addTab(self.tab_list[1], "Formations")

    def get_agent_deviations(self, selection = None):

        if selection is None:
            selection = self.selected_formation
        return self.formation_list[selection].agent_deviations

    #GUI/ROS interface functions
    #This may not be the best way to handle this but I believe it works for now, and can be fixed later
    def set_ros_node(self, ros_node):
        self.ros_node = ros_node

    def update_agent_state(self, agent_states):
        self.agent_states = agent_states

    def publish_deviations(self):
        msg = array_to_msg(self.formation_list[self.selected_formation].agent_deviations)
        self.ros_node.deviation_publisher_.publish(msg)


