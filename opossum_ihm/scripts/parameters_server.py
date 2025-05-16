#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from cdf_msgs.srv import Init


class ParametersServer(Node):
    def __init__(self):
        super().__init__("parameters_server")

        # Initialisation des paramètres
        self.team_color = "blue"
        self.script_number = 0
        self.debug_mode = False
        self.current_score = 0

        # Déclarer les paramètres avec des valeurs par défaut
        self.declare_parameter("team_color", self.team_color)
        self.declare_parameter("script_number", self.script_number)
        self.declare_parameter("debug_mode", self.debug_mode)
        self.declare_parameter("current_score", self.current_score)

        # Lire les paramètres
        # self.team_color = self.get_parameter("team_color").value
        # self.script_number = self.get_parameter("script_number").value
        # self.debug_mode = self.get_parameter("debug_mode").value
        # self.current_score = self.get_parameter("current_score").value

        # Création du service
        self.srv = self.create_service(
            Init,
            "set_parameters",
            self.set_parameters_callback
        )
        self.get_logger().info("Parameters service is ready.")

    def set_parameters_callback(self, request, response):
        # self.get_logger().info(
        #     f"Received request: team_color={request.team_color}, "
        #     f"script_number={request.script_number}, "
        #     f"debug_mode={request.debug_mode}"
        # )

        # Validation des valeurs
        if request.team_color not in ["blue", "yellow"]:
            response.success = False
            self.get_logger().error("Invalid team_color.")
            return response

        # Mise à jour des paramètres
        self.team_color = request.team_color
        self.script_number = request.script_number
        self.debug_mode = request.debug_mode
        self.current_score = request.current_score

        # Enregistrement des nouveaux paramètres
        self.set_parameters([
            rclpy.parameter.Parameter(
                "team_color",
                rclpy.Parameter.Type.STRING,
                self.team_color
            ),
            rclpy.parameter.Parameter(
                "script_number",
                rclpy.Parameter.Type.INTEGER,
                self.script_number
            ),
            rclpy.parameter.Parameter(
                "debug_mode",
                rclpy.Parameter.Type.BOOL,
                self.debug_mode
            ),
            rclpy.parameter.Parameter(
                "current_score",
                rclpy.Parameter.Type.INTEGER,
                self.current_score
            ),
        ])

        self.get_logger().info(
            f"Updated parameters: team_color={self.team_color}, "
            f"script_number={self.script_number}, debug_mode={self.debug_mode}"
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = ParametersServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
