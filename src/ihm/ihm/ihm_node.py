import rclpy
from rclpy.node import Node
from cdf_msgs.srv import Init
import tkinter as tk
from .interface import ColorChoiceApp

class IhmNode(Node, ColorChoiceApp):
    def __init__(self, root):
        super().__init__('ihm_node')
        ColorChoiceApp.__init__(self, root)

def main(args=None):
    rclpy.init(args=args)
    root = tk.Tk()
    node = IhmNode(root)
    node.run_ihm()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()