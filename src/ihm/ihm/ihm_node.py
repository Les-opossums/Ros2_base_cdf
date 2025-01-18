import rclpy
from rclpy.node import Node
from cdf_msgs.srv import Init
import tkinter as tk
from .interface import GUI
from cdf_msgs.srv import Init
from std_msgs.msg import Int32, Bool

class IhmNode(Node, GUI):
    def __init__(self):
        super().__init__('ihm_node')
        self._init_subscibers()
        self.gui =  GUI()
        self.main()

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
            else:
                self.update_parameters()
            break
        self.gui.initialized = True
        self.gui.run_score()

    def update_parameters(self):
        client = self.create_client(Init, 'set_parameters')
        request = Init.Request()
        request.team_color = self.clr
        request.script_number = self.scr
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
    
    def _init_subscibers(self):
        self.get_logger().info(f"Hello§§§§§??????")

        self.sub_score_topic = self.create_subscription(
            Int32,
            'score',
            self.score_callback,
            10
        )
        self.sub_au_topic = self.create_subscription(
            Bool,
            'au',
            self.au_callback,
            10
        )
        self.sub_enable_timer_topic = self.create_subscription(
            Bool,
            'enable_timer',
            self.enable_timer_callback,
            10
        )

    def score_callback(self, msg):
        self.get_logger().info(f"Hello score: {msg.data}")
        if self.gui.initialized:
            self.get_logger().info(f"Score: {msg.data}")

    def au_callback(self, msg):
        if self.gui.initialized:
            self.gui.update_au(msg.data)
            self.get_logger().info(f"AU: {msg.data}")

    def enable_timer_callback(self, msg):
        if self.gui.initialized:
            self.gui.update_timer(msg.data)
            self.get_logger().info(f"Enable timer: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    #root = tk.Tk()
    node = IhmNode() #root)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()