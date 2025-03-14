#ROS Depdendencies
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

#pyqt5 dependencies
from PyQt5.QtWidgets import QApplication

#helper classes
#ros is very cool and intutive: https://stackoverflow.com/questions/57426715/import-modules-in-package-in-ros2
from swarm_gui.gui_helper import gui_app

#Other requirements
from threading import Thread #need to run both GUI + ros at same time

## ROS Node ##
class Gui_Handler(Node):

    def __init__(self, gui_app):
        super().__init__('gui_handler')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.app = gui_app

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)

        #simulate what would happen if agents we're slowly moving
        agent_states = self.app.agent_states
        agent_states[0][1] += 0.01
        self.app.tab_list[0].update_agent_plots(agent_states)

def main(args=None):
    rclpy.init(args=args)

    app = QApplication([])
    window = gui_app()
    window.resize(1000, 600)
    window.show()


    #setup the ROS node and the GUI
    gui_handler_node = Gui_Handler(window)

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