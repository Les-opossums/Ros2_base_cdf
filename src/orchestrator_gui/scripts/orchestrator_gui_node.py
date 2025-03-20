#!/usr/bin/env python3
"""Display all the information gathered by captors."""

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from orchestrator_gui.qt_interface import MyWidget
from PyQt5 import QtWidgets, QtGui, QtCore
from ament_index_python.packages import get_package_share_directory
import os
import numpy as np
from rclpy.action import ActionClient
from cdf_msgs.action import MoveTo
from cdf_msgs.msg import LidarLoc
import functools


class MyROSNode(Node):
    """Make the link between interface and ROS messages."""

    def __init__(self, parent=None):
        super().__init__("gui_node")
        self.parent = parent
        self._init_parameters()
        self._init_subscriber()
        self._init_client_action()
        self.get_logger().info("Orchestrator GUI node initialized.")

    def _init_parameters(self) -> None:
        """Initialize parameters of the node."""
        self.declare_parameters(
            namespace="",
            parameters=[
                ("position_topic", rclpy.Parameter.Type.STRING),
                ("moveto_action", rclpy.Parameter.Type.STRING),
                ("robot_names", rclpy.Parameter.Type.STRING_ARRAY),
            ],
        )
        self.robot_names = (
            self.get_parameter("robot_names").get_parameter_value().string_array_value
        )

    def _init_subscriber(self):
        """Initialize subscribers of the node."""
        self.position_topic = (
            self.get_parameter("position_topic").get_parameter_value().string_value
        )
        for name in self.robot_names:
            self.create_subscription(
                LidarLoc,
                name + "/" + self.position_topic,
                functools.partial(self.position_callback, name=name),
                10,
            )

    def _init_client_action(self):
        """Initialize client of the node."""
        self.moveto_action = (
            self.get_parameter("moveto_action").get_parameter_value().string_value
        )
        self.moveto_client = {
            name: ActionClient(self, MoveTo, name + "/" + self.moveto_action)
            for name in self.robot_names
        }

    def send_goal(self, x, y, name):
        """Send goal to the action server."""
        goal_msg = MoveTo.Goal()
        goal_msg.goal = Point()
        goal_msg.goal.x = x
        goal_msg.goal.y = y
        goal_msg.goal.z = 0.0
        self.moveto_client[name].wait_for_server()

        return self.moveto_client[name].send_goal_async(goal_msg)

    def send_future_position(self, x, y, name):
        """Send future position."""
        future = self.send_goal(x, y, name)
        rclpy.spin_until_future_complete(self, future)

    def position_callback(self, msg, name):
        """Receive the last known information and display it."""
        self.parent.update_robot_position(msg, name)


class MapViewRosConnected(QtWidgets.QGraphicsView):
    """Connect to ROS and display information of a robot."""

    def __init__(self, name, parent=None):
        super().__init__()
        self.parent = parent
        self.name = name
        image_path = os.path.join(
            get_package_share_directory("orchestrator_gui"), "images"
        )
        map = os.path.join(image_path, "plateau.png")
        icon = os.path.join(image_path, "robot.png")
        self.mad_icon = os.path.join(image_path, "mad_robot.png")
        # Création de la scène
        self.scene = QtWidgets.QGraphicsScene(self)
        self.setScene(self.scene)

        # Chargement de l'image de la carte
        self.map_pixmap = QtGui.QPixmap(map).scaled(800, 800, QtCore.Qt.KeepAspectRatio)
        self.map_item = QtWidgets.QGraphicsPixmapItem(self.map_pixmap)
        self.map_item.mousePressEvent = lambda event: self.send_pos_goal(
            event, self.name
        )
        self.scene.addItem(self.map_item)

        # Chargement de l'icône et ajout d'un item déplaçable
        self.icon_pixmap = QtGui.QPixmap(icon).scaled(
            100, 100, QtCore.Qt.KeepAspectRatio
        )
        self.icon_item = DraggablePixmapItem(self.icon_pixmap)
        icon_rect = self.icon_item.boundingRect()
        self.icon_width = icon_rect.width()
        self.icon_height = icon_rect.height()
        self.icon_item.setPos(5, 5)  # Position initiale
        m_pixmap = QtGui.QPixmap(self.mad_icon).scaled(
            100, 100, QtCore.Qt.KeepAspectRatio
        )
        dragmap = DraggablePixmapItem(m_pixmap)
        m_icon_rect = dragmap.boundingRect()
        self.m_icon_width = m_icon_rect.width()
        self.m_icon_height = m_icon_rect.height()
        self.ennemis_items = []
        self.scene.addItem(self.icon_item)
        self.fitInView(self.scene.sceneRect(), QtCore.Qt.KeepAspectRatio)

    def send_pos_goal(self, event, name):
        """Send the goal position on the button click."""
        x, y = self.get_real_pos(event.pos().x(), event.pos().y())
        self.parent.send_future_position(x, y, name)

    def get_real_pos(self, x, y):
        """Convert the position on the map to real world."""
        map_rect = self.map_item.boundingRect()
        width = map_rect.width()
        height = map_rect.height()
        return 3 * x / width, 2 * (1 - y / height)

    @QtCore.pyqtSlot(LidarLoc)
    def update_robot_position(self, msg):
        """Update the robot position."""
        map_rect = self.map_item.boundingRect()
        width = map_rect.width()
        height = map_rect.height()
        posx = width * msg.robot_position.x / 3 - self.icon_width / 2
        posy = height * (1 - msg.robot_position.y / 2) - self.icon_height / 2
        new_pos = QtCore.QPointF(posx, posy)
        self.icon_item.setPos(new_pos)
        new_rotation = msg.robot_position.z * 180 / np.pi
        self.icon_item.setRotation(new_rotation)
        index = 0
        for rob in msg.other_robot_position:
            if len(self.ennemis_items) > index:
                e_x = width * rob.x / 3 - self.m_icon_width / 2
                e_y = height * (1 - rob.y / 2) - self.m_icon_height / 2
                e_pos = QtCore.QPointF(e_x, e_y)
                self.ennemis_items[index].setPos(e_pos)
            else:
                e_pix = QtGui.QPixmap(self.mad_icon).scaled(
                    100, 100, QtCore.Qt.KeepAspectRatio
                )
                e_item = DraggablePixmapItem(e_pix)
                self.ennemis_items.append(e_item)
                e_x = width * rob.x / 3 - self.m_icon_width / 2
                e_y = height * (1 - rob.y / 2) - self.m_icon_height / 2
                e_pos = QtCore.QPointF(e_x, e_y)
                self.ennemis_items[index].setPos(e_pos)
                self.scene.addItem(self.ennemis_items[index])
            index += 1
        if index < len(self.ennemis_items) - 1:
            self.ennemis_items = self.ennemis_items[:index]


class DraggablePixmapItem(QtWidgets.QGraphicsPixmapItem):
    """Drag items on the Map."""

    def __init__(self, pixmap, parent=None):
        super().__init__(pixmap, parent)
        # Permettre à cet item d'être déplacé et sélectionné
        self.setFlags(
            QtWidgets.QGraphicsItem.ItemIsMovable
            | QtWidgets.QGraphicsItem.ItemIsSelectable
        )
        self.setTransformOriginPoint(pixmap.width() / 2, pixmap.height() / 2)


# Fenêtre Qt avec un label à mettre à jour
class MyWindow(QtWidgets.QMainWindow):
    """Display all the information and master interface."""

    def __init__(self):
        super().__init__()
        self.p2 = {}
        self.ros_node = MyROSNode(self)
        self.p2 = {
            name: MapViewRosConnected(name, self) for name in self.ros_node.robot_names
        }

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
        self.comboBox.addItems(["Page 1"] + [key for key in self.p2.keys()])
        self.comboBox.currentIndexChanged.connect(self.change_page)

        p1 = MyWidget("Heyooo")
        self.pages = [p1] + [val for val in self.p2.values()]

        self.label = QtWidgets.QLabel("En attente de message...", self)

        self.layout.addWidget(self.label)
        self.layout.addWidget(self.comboBox)
        self.layout.addWidget(self.stackedWidgets)

        self.stackedWidgets.addWidget(p1)
        for val in self.p2.values():
            self.stackedWidgets.addWidget(val)
        self._connect_to_ros()

    def update_robot_position(self, msg, name):
        """Update the position of the robot."""
        self.p2[name].update_robot_position(msg)

    def send_future_position(self, x, y, name):
        """Send the position request to node ROS."""
        self.ros_node.send_future_position(x, y, name)

    def _connect_to_ros(self):
        """Connect ROS to interface."""
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(10)

    def change_page(self, index):
        """Change the page."""
        self.stackedWidgets.setCurrentIndex(index)

    def spin_ros(self):
        """Spin the ROS node."""
        rclpy.spin_once(self.ros_node, timeout_sec=0.001)


def main():
    """Run main loop."""
    rclpy.init(args=None)
    app = QtWidgets.QApplication(sys.argv)
    window = MyWindow()
    window.show()
    app.exec_()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
