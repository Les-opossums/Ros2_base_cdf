#!/usr/bin/env python3
"""Simulate the lidar sensor, outputs the beacons in the robot frame."""


# Issue when sending 2 goal and the first has not ended, it fails, to publish idk why.... Need to check on actions
# Import des librairies
import rclpy
from rclpy.node import Node
import numpy as np

# Import des messages
from rclpy.executors import ExternalShutdownException
from opossum_msgs.msg import GlobalView, VisionDataFrame, VisionData

class HazCrate:
    id: int
    x: float
    y: float
    theta: float
    state: int
    color: int # 37 yellow, 43 blue, 21 rot

    def __init__(self, msg):
        self.id = msg.id
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        self.state = 1 if msg.state.split("*")[0] == 'free' else 0
        if msg.state.split("*")[-1] == "yellow":
            self.color = 37
        elif msg.state.split("*")[-1] == "blue":
            self.color = 43
        elif msg.state.split("*")[-1] == "rot":
            self.color = 21
        else:
            self.color = -1

class CameraSimulation(Node):
    """Simulate the lidar sensor by returning points corresponding to beacons."""

    def __init__(self):
        super().__init__("camera_simulation_node")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("camera_angles", rclpy.Parameter.Type.DOUBLE_ARRAY),
                ("radius", rclpy.Parameter.Type.DOUBLE_ARRAY),
                ("cone_range", rclpy.Parameter.Type.DOUBLE_ARRAY),
            ],
        )
        self._init_main_parameters()

    def _init_main_parameters(self: Node) -> None:
        self.radius = (
            self.get_parameter("radius")
            .get_parameter_value()
            .double_array_value
        )
        self.camera_angles = (
            self.get_parameter("camera_angles")
            .get_parameter_value()
            .double_array_value
        )
        self.cone_range = (
            self.get_parameter("cone_range")
            .get_parameter_value()
            .double_array_value
        )
        self.haz_crates = {}
        self._init_publishers()
        self._init_subscribers()

    def _init_publishers(self) -> None:
        """Initialize publishers."""
        self.pub_cam = self.create_publisher(VisionDataFrame, "aruco_loc", 10)

    def _init_subscribers(self) -> None:
        """Initialize subscribers."""
        self.sub_global_view = self.create_subscription(
            GlobalView, "/global_view", self._publish_objects, 10
        )

    def _update(self, msg) -> None:
        """Update the position and transform fixed beacons and enemy positions to the robot frame."""
        ns = self.get_namespace()[1:]
        for robot in msg.robots:
            if robot.name == ns:
                x, y, theta = robot.x, robot.y, robot.theta
                self.position = np.array([[x], [y], [1]])
                self.angle = theta

        # Homogeneous transformation matrix: world → robot frame
        cos_a = np.cos(theta)
        sin_a = np.sin(theta)
        self.OtoR = np.array([
            [cos_a, -sin_a, x],
            [sin_a,  cos_a, y],
            [0, 0, 1]
        ])
        for obj in msg.objects:
            if obj.type == "haz_crate":
                self.haz_crates[obj.id] = HazCrate(obj)

    def _publish_objects(self, msg) -> None:
        """Simule la détection des caméras et publie les objets dans le repère Robot."""
        self._update(msg)
        global_msg = VisionDataFrame()
        
        # Position et angle actuel du robot (World)
        rx, ry = self.position[0][0], self.position[1][0]
        r_theta = self.angle

        # Parcourir chaque caméra définie par tes paramètres
        for cam_idx, cam_angle_robot in enumerate(self.camera_angles):
            radius_max = self.radius[cam_idx]
            half_cone = self.cone_range[cam_idx] / 2.0
            
            # Angle absolu de la caméra dans le monde
            # cam_angle_robot est l'angle de la caméra PAR RAPPORT au robot
            cam_world_angle = r_theta + cam_angle_robot

            for obj_id, crate in self.haz_crates.items():
                
                if crate.state != 1:
                    continue

                # 1. Vecteur Robot -> Objet (World)
                dx = crate.x - rx
                dy = crate.y - ry
                dist = np.sqrt(dx**2 + dy**2)

                # Vérification de la distance
                if dist > radius_max:
                    continue

                # 2. Angle de l'objet dans le monde par rapport au robot
                obj_world_angle = np.arctan2(dy, dx)

                # 3. Écart angulaire entre l'objet et l'axe de la caméra
                # On normalise l'angle entre -pi et pi
                angle_diff = (obj_world_angle - cam_world_angle + np.pi) % (2 * np.pi) - np.pi

                if abs(angle_diff) <= half_cone:
                    # L'objet est dans le cône !
                    obj_msg = VisionData()
                    
                    # Transformer les coordonnées dans le repère ROBOT
                    # On utilise la rotation inverse de l'angle du robot
                    cos_r = np.cos(-r_theta)
                    sin_r = np.sin(-r_theta)
                    
                    # Position relative (Robot Frame)
                    obj_msg.x = dx * cos_r - dy * sin_r
                    obj_msg.y = dx * sin_r + dy * cos_r
                    
                    # Angle relatif de l'objet (Robot Frame)
                    obj_msg.theta = (crate.theta - r_theta + np.pi) % (2 * np.pi) - np.pi
                    
                    obj_msg.id = crate.color
                    global_msg.object.append(obj_msg)

        self.pub_cam.publish(global_msg)
        


def main(args=None):
    """Run main loop."""
    rclpy.init(args=args)
    camerasimu_node = CameraSimulation()
    try:
        rclpy.spin(camerasimu_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        camerasimu_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
