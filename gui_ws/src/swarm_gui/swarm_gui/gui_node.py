#ROS Depdendencies
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

#PyQT5 Depdendencies
import numpy
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QLabel
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
        self.app.label.setText(f"Rx: {msg.data}")

#https://www.pythonguis.com/tutorials/creating-your-first-pyqt-window/
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("My App")
        # button = QPushButton("Press me!")
        self.label = QLabel("Rx: ")

        self.setCentralWidget(self.label)

def main(args=None):
    rclpy.init(args=args)

    

    app = QApplication([])
    window = MainWindow()
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