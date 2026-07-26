"""Setup « connexion IHM web » pour le robot.

Lance en un seul launch tout ce dont la page web de debug a besoin pour se
connecter au robot :

  * ``rosbridge_server`` (pont WebSocket, port 9090) ;
  * ``tag_fusion_node`` (opossum_vision) -> topics ``aruco_world`` /
    ``aruco_world_fused`` affichés sur la carte ;
  * ``calibration_manager`` (opossum_dev_gui) -> calibration caméra pilotable
    depuis la page web.

Ce launch est piloté par le bouton « Connexion Web » de l'IHM Qt
(opossum_ihm/interface.py). On le coupe pendant un vrai match pour libérer du
CPU : tous les nœuds ci-dessus sont alors arrêtés proprement.

Le namespace par défaut est ``main_robot`` (surchargable via l'argument
``namespace``) pour rester cohérent avec les bringups.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    port = LaunchConfiguration("port")

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="main_robot",
        description="Namespace ROS du robot (doit matcher les bringups).",
    )
    declare_port = DeclareLaunchArgument(
        "port",
        default_value="9090",
        description="Port WebSocket exposé par rosbridge.",
    )

    # --- Pont WebSocket rosbridge (page web / Foxglove) ---
    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rosbridge_server"),
                    "launch",
                    "rosbridge_websocket_launch.xml",
                ]
            )
        ),
        launch_arguments={"port": port}.items(),
    )

    # --- Fusion des tags en coordonnées monde (carte web) ---
    param_tag_fusion = PathJoinSubstitution(
        [FindPackageShare("opossum_vision"), "config", "tag_fusion_params.yaml"]
    )
    node_tag_fusion = Node(
        namespace=namespace,
        package="opossum_vision",
        executable="tag_fusion_node.py",
        name="tag_fusion_node",
        output="screen",
        parameters=[param_tag_fusion],
    )

    # --- Gestionnaire de calibration caméra (piloté depuis la page web) ---
    node_calibration = Node(
        namespace=namespace,
        package="opossum_dev_gui",
        executable="calibration_manager.py",
        name="calibration_manager",
        output="screen",
    )

    # --- Moniteur systeme (CPU/RAM) pour la page « Systeme » de l'IHM web ---
    node_system_monitor = Node(
        namespace=namespace,
        package="opossum_dev_gui",
        executable="system_monitor.py",
        name="system_monitor",
        output="screen",
    )

    # --- Superviseur de noeuds (allumer/couper + statut) ---
    node_manager_params = PathJoinSubstitution(
        [FindPackageShare("opossum_dev_gui"), "config", "node_manager.yaml"]
    )
    node_manager = Node(
        namespace=namespace,
        package="opossum_dev_gui",
        executable="node_manager.py",
        name="node_manager",
        output="screen",
        parameters=[node_manager_params],
    )

    return LaunchDescription(
        [
            declare_namespace,
            declare_port,
            rosbridge,
            node_tag_fusion,
            node_calibration,
            node_system_monitor,
            node_manager,
        ]
    )
