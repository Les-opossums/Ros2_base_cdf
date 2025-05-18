#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from opossum_msgs.srv import Init
from opossum_ihm.interface import GUI
from opossum_msgs.msg import LidarLoc
from std_msgs.msg import Int32, Bool
import threading


class IhmNode(Node):
    def __init__(self):
        super().__init__("ihm_node")
        self.get_logger().info("Initializing IHM Node")

        # Initialisation des abonnements
        self._init_subscribers()
        self.current_score = 0
        self.clr = 'blue'
        self.scr = 0
        self.ready_plot_pos = False

        # Initialisation de l'interface graphique
        self.gui = GUI()

        # Démarrage de la logique principale
        self.logic_thread = threading.Thread(target=self.main_logic,
                                             daemon=True)
        self.logic_thread.start()

    def main_logic(self):
        """Logique principale pour interagir avec l'utilisateur."""
        while True:
            self.gui.reload = False
            self.update_parameters()
            self.gui.run_color()
            if self.gui.reload:
                continue

            self.clr = self.gui.get_color()
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
            else:
                self.update_parameters()
            break

        self.gui.run_score()
        self.gui.score_app.update_score()
        self.gui.score_app.update_au()
        # self.gui.score_app.update_position()
        self.gui.score_app.root.mainloop()

    def update_parameters(self):
        """Met à jour les paramètres via un service ROS 2."""
        client = self.create_client(Init, "set_parameters")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for 'set_parameters' service...")

        request = Init.Request()
        request.team_color = self.clr
        request.script_number = self.scr
        request.current_score = self.current_score

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

    def _init_subscribers(self):
        """Initialise les abonnements aux topics ROS 2."""
        self.get_logger().info("Setting up subscribers...")

        self.sub_score_topic = self.create_subscription(
            Int32,
            "score",
            self.score_callback,
            10
        )
        self.sub_au_topic = self.create_subscription(
            Bool,
            "au",
            self.au_callback,
            10
        )
        self.sub_enable_timer_topic = self.create_subscription(
            Bool,
            "enable_timer",
            self.enable_timer_callback,
            10
        )
        self.sub_comm_state_topic = self.create_subscription(
            Bool,
            "/main_robot/comm_state",
            self.comm_state_callback,
            10
        )
        self.lidar_loc_sub = self.create_subscription(
            LidarLoc,
            "/main_robot/position_out",
            self.lidar_loc_callback,
            1
        )

    def score_callback(self, msg):
        """Callback pour le topic 'score'."""
        self.get_logger().info(f"Score received: {msg.data}")
        if self.gui.initialized:
            self.gui.score_app.score += msg.data
            self.get_logger().info(
                f"Score received: {msg.data}, "
                f"current_score={self.gui.score_app.score}"
            )

    def au_callback(self, msg):
        """Callback pour le topic 'au'."""
        if self.gui.initialized:
            self.gui.score_app.is_au = msg.data
            self.get_logger().info(f"AU received: {msg.data}")

    def enable_timer_callback(self, msg):
        """Callback pour le topic 'enable_timer'."""
        if self.gui.initialized:
            self.gui.score_app.is_match = msg.data
            self.get_logger().info(f"Enable Timer received: {msg.data}")

    def comm_state_callback(self, msg):
        """Callback pour le topic 'comm_state'."""
        if self.gui.initialized:
            self.gui.score_app.comm_state = msg.data
            self.get_logger().info(f"Comm State received: {msg.data}")

    def lidar_loc_callback(self, msg: LidarLoc):
        """Receive Lidar location."""
        if self.gui.initialized:
            self.gui.score_app.lidar_pos_x = msg.robot_position.x
            self.gui.score_app.lidar_pos_y = msg.robot_position.y
            self.gui.score_app.lidar_pos_t = msg.robot_position.z


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
