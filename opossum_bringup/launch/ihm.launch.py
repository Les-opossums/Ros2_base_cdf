"""Launch the IHM node for multiple robots."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    """Launch the setup."""
    nodes = []
    
    # Récupération et découpage de la chaîne de caractères des robots
    robot_names_str = LaunchConfiguration("robot_names").perform(context)
    robot_names_list = [name.strip() for name in robot_names_str.split(",")]

    # Boucle pour instancier l'IHM et le serveur de paramètres pour CHAQUE robot
    for robot in robot_names_list:
        node_ihm = Node(
            package="opossum_ihm",
            namespace=robot,  # Utilisation du nom dynamique
            executable="ihm_node.py",
            name="node_ihm",
            parameters=[],
        )

        param_server = Node(
            package="opossum_ihm",
            namespace=robot,  # Utilisation du nom dynamique
            executable="parameters_server.py",
            name="param_server",
            parameters=[],
        )

        nodes.append(node_ihm)
        nodes.append(param_server)

    return nodes


def generate_launch_description():
    """Generate launch description."""
    robot_names_arg = DeclareLaunchArgument(
        "robot_names",
        default_value="main_robot",
        description="Set all the simulated robots",
    )

    return LaunchDescription(
        [robot_names_arg, OpaqueFunction(function=launch_setup)]
    )