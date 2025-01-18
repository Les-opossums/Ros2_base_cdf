import rclpy
from rclpy.node import Node
from cdf_msgs.srv import Init
import tkinter as tk
from .interface import GUI

class IhmNode(Node, GUI):
    def __init__(self):
        super().__init__('ihm_node')
        GUI.__init__(self)

def main(args=None):
    rclpy.init(args=args)
    #root = tk.Tk()
    node = IhmNode() #root)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()