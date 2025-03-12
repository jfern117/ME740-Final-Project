#ROS Depdendencies
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

#PyQT5 Depdendencies
import numpy
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QWidget, QTabWidget, QPushButton #ui elements
from PyQt5.QtWidgets import QVBoxLayout #layouts
from PyQt5.QtCore import QSize, Qt
import sys

#Other requirements
from threading import Thread #need to run both GUI + ros at same time


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
        self.app.tab_list[0].label.setText(f"Rx: {msg.data}")


class sim_tab(QWidget):
    def __init__(self, parent):
        super().__init__(parent)

        self.tab_layout = QVBoxLayout()
        self.label = QLabel("Rx: ")
        self.tab_layout.addWidget(self.label)
        self.setLayout(self.tab_layout)


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