#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from opossum_msgs.srv import Init
from opossum_ihm.interface import GUI
from opossum_msgs.msg import LidarLoc
from std_msgs.msg import Int32, Bool, String
import threading
import time


class IhmNode(Node):
    def __init__(self):
        super().__init__("ihm_node")
        self.get_logger().info("Initializing IHM Node")
        self._init_parameters()
        self._init_publishers()
        self._init_subscribers()

        # Initialisation de l'interface graphique
        self.gui = GUI(self.name)
        self.logic_thread = threading.Thread(target=self.main_logic,
                                             daemon=True)
        self.logic_thread.start()

    def _init_parameters(self):
        self.declare_parameters(
            namespace="",
            parameters=[
                ("color_topic", "init_team_color"),
                ("position_topic", "position_out"),
                ("score_topic", "score"),
                ("au_topic", "au"),
                ("enable_timer_topic", "enable_timer"),
                ("comm_state_topic", "comm_state"),
            ]
        )
        self.current_score = 0
        self.clr = 'blue'
        self.scr = 0
        self.ready_plot_pos = False
        self.score = 0
        self.debug = False
        self.au = False
        self.comm_state = True
        self.x, self.y, self.t = None, None, None
        name = self.get_namespace()
        self.name = name[1:] if name[0] == "/" else name 

    def _init_publishers(self):
        color_topic = (
            self.get_parameter("color_topic").get_parameter_value().string_value
        )
        self.pub_color_topic = self.create_publisher(String, color_topic, 10)

    def _init_subscribers(self):
        """Initialise les abonnements aux topics ROS 2."""
        self.get_logger().info("Setting up subscribers...")
        
        score_topic = (
            self.get_parameter("score_topic").get_parameter_value().string_value
        )
        
        au_topic = (
            self.get_parameter("au_topic").get_parameter_value().string_value
        )

        enable_timer_topic = (
            self.get_parameter("enable_timer_topic").get_parameter_value().string_value
        )

        comm_state_topic = (
            self.get_parameter("comm_state_topic").get_parameter_value().string_value
        )

        position_topic = (
            self.get_parameter("position_topic").get_parameter_value().string_value
        )

        self.sub_score = self.create_subscription(
            Int32,
            score_topic,
            self.score_callback,
            10
        )
        self.sub_au = self.create_subscription(
            Bool,
            au_topic,
            self.au_callback,
            10
        )
        self.sub_enable_timer = self.create_subscription(
            Bool,
            enable_timer_topic,
            self.enable_timer_callback,
            10
        )
        self.sub_comm_state = self.create_subscription(
            Bool,
            comm_state_topic,
            self.comm_state_callback,
            10
        )
        self.lidar_loc_sub = self.create_subscription(
            LidarLoc,
            position_topic,
            self.lidar_loc_callback,
            1
        )

    def main_logic(self):
        """Logique principale pour interagir avec l'utilisateur."""
        while True:
            self.gui.reload = False
            self.update_parameters()
            time.sleep(0.2)
            self.gui.run_color(self.au)
            if self.gui.reload:
                continue

            self.clr = self.gui.get_color()
            self.pub_color_topic.publish(String(data=self.clr))
            self.get_logger().info(f"Color selected: {self.clr}")

            self.gui.run_script()
            if self.gui.reload:
                continue

            self.scr = self.gui.get_script()
            self.get_logger().info(f"Script selected: {self.scr}")

            self.update_parameters()

            self.gui.run_validation()
            if self.gui.reload:
                continue
            elif self.gui.launched_init:
                self.debug = self.gui.launched_init
                self.update_parameters()
                self.debug = False
                self.update_parameters()
                continue
            else:
                self.update_parameters()
                break

        self.gui.run_score()
        self.timer = self.create_timer(0.5, self.update_values)
        self.gui.score_app.update_gif()
        self.gui.score_app.root.mainloop()

    def update_values(self):
        self.gui.score_app.update_score(self.score)
        self.gui.score_app.update_au(self.au, self.comm_state)
        self.gui.score_app.update_position(self.x, self.y, self.t)

    def update_parameters(self):
        """Met à jour les paramètres via un service ROS 2."""
        client = self.create_client(Init, "set_parameters")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for 'set_parameters' service...")

        request = Init.Request()
        request.team_color = self.clr
        request.script_number = self.scr
        request.current_score = self.current_score
        request.debug_mode = self.debug

        future = client.call_async(request)
        future.add_done_callback(self.handle_service_response)

    def handle_service_response(self, future):
        """Gère la réponse du service asynchrone."""
        try:
            response = future.result()
            # self.get_logger().info(f"Parameters updated successfully: "
            #                        f"{response}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def score_callback(self, msg):
        """Callback pour le topic 'score'."""
        self.get_logger().info(f"Score received: {msg.data}")
        self.score += msg.data
        self.get_logger().info(
            f"Score received: {msg.data}, "
            f"current_score={self.score}"
        )

    def au_callback(self, msg):
        """Callback pour le topic 'au'."""
        self.au = msg.data
        # self.get_logger().info(f"AU received: {msg.data}")

    def enable_timer_callback(self, msg):
        """Callback pour le topic 'enable_timer'."""
        if self.gui.initialized:
            self.gui.score_app.is_match = msg.data
            self.get_logger().info(f"Enable Timer received: {msg.data}")

    def comm_state_callback(self, msg):
        """Callback pour le topic 'comm_state'."""
        self.comm_state = msg.data
        self.get_logger().info(f"Comm State received: {msg.data}")

    def lidar_loc_callback(self, msg: LidarLoc):
        """Receive Lidar location."""
        self.x = msg.robot_position.x
        self.y = msg.robot_position.y
        self.t = msg.robot_position.z


def main(args=None):
    rclpy.init(args=args)
    node = IhmNode()

    try:
        rclpy.spin(node)  # Gestion des messages ROS 2
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        node.get_logger().info("Shutting down IHM Node")


if __name__ == "__main__":
    main()
