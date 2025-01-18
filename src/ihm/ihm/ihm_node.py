import rclpy
from rclpy.node import Node
from cdf_msgs.srv import Init
import tkinter as tk
from .interface import GUI
from cdf_msgs.srv import Init

class IhmNode(Node, GUI):
    def __init__(self):
        super().__init__('ihm_node')
        gui =  GUI()
        gui.run_color()
        clr = gui.get_color()
        self.get_logger().info("Color selected: " + clr)
        gui.run_script()
        scr = gui.get_script()
        self.get_logger().info("Script selected: " + str(src))
        
def main(args=None):
    rclpy.init(args=args)
    #root = tk.Tk()
    node = IhmNode() #root)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()