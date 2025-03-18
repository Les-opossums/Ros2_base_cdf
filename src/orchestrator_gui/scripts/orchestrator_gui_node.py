#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from PyQt5 import QtWidgets, QtCore
from orchestrator_gui.qt_interface import MyWidget, MyPoorWidget
from cdf_msgs.msg import LidarLoc
from PyQt5 import QtWidgets, QtGui, QtCore
from ament_index_python.packages import get_package_share_directory  # type: ignore
import os
import numpy as np
from rclpy.action import ActionClient
from cdf_msgs.action import MoveTo


class MyROSNode(Node):
    def __init__(self, parent=None):
        super().__init__('gui_node')
        self.parent = parent
        self._init_parameters()
        self._init_subscriber()
        self._init_client_action()
        self.get_logger().info("Orchestrator GUI node initialized.")
        
    def _init_parameters(self) -> None:
        self.declare_parameters(
            namespace="",
            parameters=[
                ("position_topic", rclpy.Parameter.Type.STRING),
                ("moveto_action", rclpy.Parameter.Type.STRING),
                ("robot_names", rclpy.Parameter.Type.STRING_ARRAY),
            ],
        )
        self.robot_names = self.get_parameter("robot_names").get_parameter_value().string_array_value

    def _init_subscriber(self):
        self.position_topic = (
            self.get_parameter("position_topic").get_parameter_value().string_value
        )
        for name in self.robot_names:
            self.create_subscription(
                LidarLoc,
                name + '/' + self.position_topic,
                lambda msg: self.position_callback(msg, name),
                10
            )

    def _init_client_action(self):
        self.moveto_action = (
            self.get_parameter("moveto_action").get_parameter_value().string_value
        )
        self.get_logger().info(f"Im sending information to {'/' + self.robot_names[0] + '/' + self.moveto_action} ")

        self.moveto_client = {name: ActionClient(self, MoveTo, name + '/' + self.moveto_action) for name in self.robot_names}

    def send_goal(self, x, y, name):
        goal_msg = MoveTo.Goal()
        goal_msg.goal = Point()
        goal_msg.goal.x = x
        goal_msg.goal.y = y
        goal_msg.goal.z = 0.
        self.get_logger().info(f"sending to {name}")
        self.moveto_client[name].wait_for_server()

        return self.moveto_client[name].send_goal_async(goal_msg)

    def send_future_position(self, x, y, name):
        self.get_logger().info(f"Im sending information")
        future = self.send_goal(x, y, name)

        rclpy.spin_until_future_complete(self, future)

    def position_callback(self, msg, name):
        self.parent.update_robot_position(msg.robot_position, name)

class MapViewRosConnected(QtWidgets.QGraphicsView):
    def __init__(self, name, parent=None):
        super().__init__()
        self.parent = parent
        self.name = name
        image_path = os.path.join(get_package_share_directory("orchestrator_gui"), "images")
        map = os.path.join(image_path, "plateau.png")
        icon = os.path.join(image_path, "robot.png")
        # Création de la scène
        self.scene = QtWidgets.QGraphicsScene(self)
        self.setScene(self.scene)
        
        # Chargement de l'image de la carte
        self.map_pixmap = QtGui.QPixmap(map).scaled(800, 800, QtCore.Qt.KeepAspectRatio)
        self.map_item = QtWidgets.QGraphicsPixmapItem(self.map_pixmap)
        self.map_item.mousePressEvent = lambda event: self.send_pos_goal(event, self.name)
        self.scene.addItem(self.map_item)
        
        # Chargement de l'icône et ajout d'un item déplaçable
        self.icon_pixmap = QtGui.QPixmap(icon).scaled(100, 100, QtCore.Qt.KeepAspectRatio)
        self.icon_item = DraggablePixmapItem(self.icon_pixmap)
        icon_rect = self.icon_item.boundingRect()
        self.icon_width = icon_rect.width()
        self.icon_height = icon_rect.height()
        self.icon_item.setPos(5, 5)  # Position initiale
        self.scene.addItem(self.icon_item)
        self.fitInView(self.scene.sceneRect(), QtCore.Qt.KeepAspectRatio)

    def send_pos_goal(self, event, name):
        x, y = self.get_real_pos(event.pos().x(), event.pos().y())
        self.parent.send_future_position(x, y, name)

    def get_real_pos(self, x, y):
        map_rect = self.map_item.boundingRect()
        width = map_rect.width()
        height = map_rect.height()
        # return 3 * x / width, 2 * (y / height)
        return 3 * x / width, 2 * (1 - y / height)
    
    @QtCore.pyqtSlot(Point)
    def update_robot_position(self, msg):
        map_rect = self.map_item.boundingRect()
        width = map_rect.width()
        height = map_rect.height()
        posx = width * msg.x / 3 - self.icon_width / 2
        posy =  height *(1 - msg.y / 2)  -  self.icon_height / 2
        new_pos = QtCore.QPointF(posx, posy)
        self.icon_item.setPos(new_pos)
        print(f"Icône déplacée vers {new_pos}")
        new_rotation = msg.z * 180 / np.pi
        self.icon_item.setRotation(new_rotation)

class DraggablePixmapItem(QtWidgets.QGraphicsPixmapItem):
    def __init__(self, pixmap, parent=None):
        super().__init__(pixmap, parent)
        # Permettre à cet item d'être déplacé et sélectionné
        self.setFlags(QtWidgets.QGraphicsItem.ItemIsMovable |
                      QtWidgets.QGraphicsItem.ItemIsSelectable)
        self.setTransformOriginPoint(pixmap.width() / 2, pixmap.height() / 2)

# Fenêtre Qt avec un label à mettre à jour
class MyWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.p2 = {}
        self.p2["main_robot"] = MapViewRosConnected("main_robot", self)
        self.ros_node = MyROSNode(self)

        self.setWindowTitle("Subscriber ROS2 - Mise à jour d'un texte")
        self.setGeometry(0, 0, 800, 500)
        # self.setGeometry(0, 0, 1900, 1040)

        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        self.layout = QtWidgets.QVBoxLayout(central_widget)
        self.stackedWidgets = QtWidgets.QStackedWidget()

        # Add Pages for need
        self.comboBox = QtWidgets.QComboBox()
        self.pageComboBox = QtWidgets.QComboBox()
        self.comboBox.addItems(["Page 1", "Page 2"])
        self.comboBox.currentIndexChanged.connect(self.change_page)

        p1 = MyWidget('Heyooo')
        self.pages = [p1, self.p2["main_robot"]]

        self.label = QtWidgets.QLabel("En attente de message...", self)

        self.layout.addWidget(self.label)
        self.layout.addWidget(self.comboBox)
        self.layout.addWidget(self.stackedWidgets)

        self.stackedWidgets.addWidget(p1)
        self.stackedWidgets.addWidget(self.p2["main_robot"])
        self._connect_to_ros()
    
    def update_robot_position(self, msg, name):
        self.p2[name].update_robot_position(msg)

    def send_future_position(self, x, y, name):
        self.ros_node.send_future_position(x, y, name)

    def _connect_to_ros(self):
        # QTimer pour intégrer le traitement ROS2 dans la boucle Qt
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(10)  # Vérifie ROS2 toutes les 10 ms

    def change_page(self, index):
        self.stackedWidgets.setCurrentIndex(index)
        
    def spin_ros(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0.001)

def main():
    rclpy.init(args=None)
    app = QtWidgets.QApplication(sys.argv)

    # ROS2 Node created inside it
    window = MyWindow()
    window.show()

    #Start the app
    app.exec_()

    # Claening, but if bugs, get the Ros2 node outside
    # ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



