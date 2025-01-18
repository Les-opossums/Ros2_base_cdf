import rclpy
from rclpy.node import Node
from cdf_msgs.srv import Init
import tkinter as tk
from .interface import GUI
from cdf_msgs.srv import Init

class IhmNode(Node, GUI):
    def __init__(self):
        super().__init__('ihm_node')
        self.gui =  GUI()

    def main(self):
        while True:
            self.gui.reload = False
            self.gui.run_color()
            if self.gui.reload:
                continue
            self.clr = self.gui.get_color()
            self.get_logger().info("Color selected: " + self.clr)
            self.gui.run_script()
            if self.gui.reload:
                continue
            self.scr = self.gui.get_script()
            self.get_logger().info("Script selected: " + str(self.scr))
            self.update_parameters()
            self.gui.run_validation()
            if self.gui.reload:
                continue
            break 

    def update_parameters(self):
        client = self.create_client(Init, 'set_parameters')
        request = Init.Request()
        request.team_color = self.clr
        request.script_number = self.scr
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
def main(args=None):
    rclpy.init(args=args)
    #root = tk.Tk()
    node = IhmNode() #root)
    node.main()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()