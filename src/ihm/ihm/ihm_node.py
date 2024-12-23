import rclpy
from rclpy.node import Node
from cdf_msgs.srv import Init

class ParametersServer(Node):
    def __init__(self):
        super().__init__('parameters_server')

        # Initialisation des paramètres
        self.team_color = "blue"
        self.script_number = 1
        self.debug_mode = False

        # Création du service
        self.srv = self.create_service(Init, 'set_parameters', self.set_parameters_callback)
        self.get_logger().info("Parameters service is ready.")

    def set_parameters_callback(self, request, response):
        self.get_logger().info(f"Received request: team_color={request.team_color}, "
                               f"script_number={request.script_number}, debug_mode={request.debug_mode}")

        # Validation des valeurs
        if request.team_color not in ['blue', 'yellow']:
            response.success = False
            self.get_logger().error("Invalid team_color. Must be 'blue' or 'yellow'.")
            return response

        # Mise à jour des paramètres
        self.team_color = request.team_color
        self.script_number = request.script_number
        self.debug_mode = request.debug_mode

        self.get_logger().info(f"Updated parameters: team_color={self.team_color}, "
                               f"script_number={self.script_number}, debug_mode={self.debug_mode}")

        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ParametersServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()