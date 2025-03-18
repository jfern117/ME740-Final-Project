#ROS Depdendencies
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray

#pyqt5 dependencies
from PyQt5.QtWidgets import QApplication

#helper classes
#ros is very cool and intutive: https://stackoverflow.com/questions/57426715/import-modules-in-package-in-ros2
from swarm_gui.gui import gui_app
from swarm_gui.messaging_helper import msg_to_array, array_to_msg

#Other requirements
from threading import Thread #need to run both GUI + ros at same time

## ROS Node ##
class Gui_Handler(Node):

    def __init__(self, gui_app:gui_app):
        super().__init__('gui_handler')
        self.agent_state_sub_ = self.create_subscription(
            Float64MultiArray,
            'agent_states',
            self.agent_state_callback,
            10)
        
        self.toggle_formation_sub_ = self.create_subscription(Bool, "formation_toggle", self.formation_toggle_callback, 10)
        
        self.deviation_publisher_ = self.create_publisher(Float64MultiArray,
                                                          "agent_deviations",
                                                          10)

        self.main_app = gui_app

    def agent_state_callback(self, msg):

        #TODO: make this less janky. For now we just publish each time we get the state
        self.main_app.publish_deviations()

        agent_states = msg_to_array(msg)
        self.main_app.tab_list[0].update_agent_plots(agent_states) #this helper function updates the agent states

    def formation_toggle_callback(self, msg):
        self.main_app.selected_formation = (self.main_app.selected_formation + 1) % len(self.main_app.formation_list)

def main(args=None):
    rclpy.init(args=args)

    #setup the GUI
    app = QApplication([])
    window = gui_app()
    window.resize(1000, 600)
    window.show()

    #setup the ROS node. The GUI is the parent for the ROS node, but currently needs to be made after it, so there's a method to setting it
    gui_handler_node = Gui_Handler(window)
    window.set_ros_node(gui_handler_node)

    #setup the thread to run the ros messaging
    def ros_thread_func():
        rclpy.spin(gui_handler_node)
    
    #since this is a daemon, it auto-terminates when the program finishes
    ros_ctrl_thread = Thread(target=ros_thread_func, daemon=True)
    ros_ctrl_thread.start()
    
    #run the GUI loop in the main program
    app.exec()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gui_handler_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()