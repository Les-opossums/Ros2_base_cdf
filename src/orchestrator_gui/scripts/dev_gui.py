#!/usr/bin/env python3

# Import des librairies
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
import time
import numpy as np
import matplotlib.pyplot as plt
import threading
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from orchestrator_gui.qt_interface import MyStackWidgetMain
from PySide2 import QtCore, QtWidgets, QtGui
import threading

class GUI(Node):
    def __init__(self):
        super().__init__("gui_node")
        app = QtWidgets.QApplication()
        my_gui = MyStackWidgetMain()
        gui_thread = threading.Thread(target=my_gui.show())
        # my_gui.show()

def main(args=None):
    rclpy.init(args=args)
    gui_node = GUI()
    gui_node.get_logger().info('Hello World!.\n')

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(gui_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        gui_node.get_logger().info('Keyboard interrupt, shutting down.\n')
    gui_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()