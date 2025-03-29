#!/usr/bin/env python3
"""Display all the information gathered by captors."""

import sys
import rclpy
from rclpy.node import Node
from PyQt5 import QtWidgets, QtGui, QtCore
from ament_index_python.packages import get_package_share_directory
import os
import numpy as np
from rclpy.action import ActionClient
from cdf_msgs.action import MoveTo
from cdf_msgs.msg import LidarLoc
from std_msgs.msg import String
import functools


class NodeGUI(Node):
    """Make the link between interface and ROS messages."""

    def __init__(self, parent=None):
        super().__init__("gui_node")
        self.parent = parent
        self._init_parameters()
        self._init_publishers()
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
                ("command_topic", rclpy.Parameter.Type.STRING),
                ("robot_names", rclpy.Parameter.Type.STRING_ARRAY),
            ],
        )
        self.robot_names = (
            self.get_parameter("robot_names").get_parameter_value().string_array_value
        )

    def _init_publishers(self):
        """Initialize subscribers of the node."""
        self.command_topic = (
            self.get_parameter("command_topic").get_parameter_value().string_value
        )
        self.pub_command = {
            name: self.create_publisher(String, name + "/" + self.command_topic, 10)
            for name in self.robot_names
        }

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

    def publish_command(self, x, y, theta, name):
        """Publish the command for the robot."""
        command_msg = String()
        command = "MOVE " + str(x) + " " + str(y) + " " + str(theta)
        command_msg.data = command
        self.pub_command[name].publish(command_msg)

    def position_callback(self, msg, name):
        """Receive the last known information and display it."""
        self.parent.update_robot_position(msg, name)


class MapScene(QtWidgets.QGraphicsView):
    """Connect to ROS and display information of a robot."""

    def __init__(self, name, parent=None, use_map=True):
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
        if use_map:
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
        self.parent.send_future_position(x, y, 0.0, name)

    def get_real_pos(self, x, y):
        """Convert the position on the map to real world."""
        map_rect = self.map_item.boundingRect()
        width = map_rect.width()
        height = map_rect.height()
        return 3 * x / width, 2 * (1 - y / height)

    @QtCore.pyqtSlot(LidarLoc)
    def update_map_position(self, msg):
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


class GlobalViewPage(QtWidgets.QWidget):
    """Display Global State."""

    def __init__(self, name, parent=None):
        super().__init__()
        self.parent = parent
        self.name = name
        main_layout = QtWidgets.QVBoxLayout(self)

        self.map_scene = MapScene(name, self.parent)
        form_layout = QtWidgets.QFormLayout()

        # Create QLabel objects for the static labels (the keys)
        # and QLabel objects for the dynamic values.
        self.x_value_label = QtWidgets.QLabel("0.0")
        self.y_value_label = QtWidgets.QLabel("0.0")
        self.t_value_label = QtWidgets.QLabel("0.0")

        # Add rows to the layout: each row has a static label and a value label.
        form_layout.addRow("x:", self.x_value_label)
        form_layout.addRow("y:", self.y_value_label)
        form_layout.addRow("theta:", self.t_value_label)
        main_layout.addWidget(self.map_scene)
        main_layout.addLayout(form_layout)
        # self.setLayout(main_layout)

    def update_text_position(self, msg):
        """Update the text of the dynamic labels."""
        self.x_value_label.setText(f"{msg.robot_position.x:.2f}")
        self.y_value_label.setText(f"{msg.robot_position.y:.2f}")
        self.t_value_label.setText(f"{msg.robot_position.z:.2f}")

    def update_map_position(self, msg):
        """Update the map of the robots."""
        self.map_scene.update_map_position(msg)


class MotorsPage(QtWidgets.QWidget):
    """Page dedicated for Motors."""

    def __init__(self, name, parent=None):
        super().__init__()
        self.parent = parent
        self.name = name
        main_layout = QtWidgets.QVBoxLayout(self)

        self.map_scene = MapScene(name, self.parent)
        form_layout = QtWidgets.QFormLayout()
        grid_layout = QtWidgets.QGridLayout()
        self.button = QtWidgets.QPushButton("Send Command")

        # Create QLabel objects for the static labels (the keys)
        # and QLabel objects for the dynamic values.
        self.x_value_label = QtWidgets.QLabel("0.0")
        self.y_value_label = QtWidgets.QLabel("0.0")
        self.t_value_label = QtWidgets.QLabel("0.0")

        # Create labels for each parameter
        x_label = QtWidgets.QLabel("X (m):")
        y_label = QtWidgets.QLabel("Y (m):")
        theta_label = QtWidgets.QLabel("Theta (deg):")

        # Create line edits for each parameter
        self.x_edit = QtWidgets.QLineEdit(self)
        self.x_edit.setPlaceholderText("Enter x value")
        self.y_edit = QtWidgets.QLineEdit(self)
        self.y_edit.setPlaceholderText("Enter y value")
        self.theta_edit = QtWidgets.QLineEdit(self)
        self.theta_edit.setPlaceholderText("Enter theta value")

        # Place widgets in the grid
        grid_layout.addWidget(x_label, 0, 0)
        grid_layout.addWidget(self.x_edit, 0, 1)
        grid_layout.addWidget(y_label, 1, 0)
        grid_layout.addWidget(self.y_edit, 1, 1)
        grid_layout.addWidget(theta_label, 2, 0)
        grid_layout.addWidget(self.theta_edit, 2, 1)

        self.button.clicked.connect(self.send_command)
        self.x_edit.returnPressed.connect(self.y_edit.setFocus)
        self.y_edit.returnPressed.connect(self.theta_edit.setFocus)
        # For the last QLineEdit, trigger the button click when Enter is pressed.
        self.theta_edit.returnPressed.connect(self.button.setFocus)

        # Add rows to the layout: each row has a static label and a value label.
        form_layout.addRow("X (m):", self.x_value_label)
        form_layout.addRow("Y (m):", self.y_value_label)
        form_layout.addRow("Theta (deg):", self.t_value_label)
        main_layout.addWidget(self.map_scene)
        main_layout.addLayout(form_layout)
        main_layout.addLayout(grid_layout)
        main_layout.addWidget(self.button)

    def send_command(self):
        """Send command to ROS."""
        x = float(self.x_edit.text())
        y = float(self.y_edit.text())
        theta = float(self.theta_edit.text()) * np.pi / 180
        self.parent.send_future_position(x, y, theta, self.name)

    def update_text_position(self, msg):
        """Update the text of the dynamic labels."""
        self.x_value_label.setText(f"{msg.robot_position.x:.2f}")
        self.y_value_label.setText(f"{msg.robot_position.y:.2f}")
        self.t_value_label.setText(f"{msg.robot_position.z * 180 / np.pi:.2f}")

    def update_map_position(self, msg):
        """Update the map of the robots."""
        self.map_scene.update_map_position(msg)


class MainRobotPage(QtWidgets.QWidget):
    """Page for ecah robot."""

    def __init__(self, name, parent=None):
        super().__init__()
        self.parent = parent
        self.name = name
        self.components_names = QtWidgets.QComboBox()
        self.components_names.addItems(
            ["Global View", "Motors", "Servo", "Create Script"]
        )
        self.component_pages = {
            "GlobalView": GlobalViewPage(name, self.parent),
            "Motors": MotorsPage(name, self.parent),
            "Servo": MapScene(name, self.parent),
            "Create Script": MapScene(name, self.parent),
        }
        self.stackedWidgets = QtWidgets.QStackedWidget()
        for val in self.component_pages.values():
            self.stackedWidgets.addWidget(val)

        self.components_names.currentIndexChanged.connect(self.change_page)
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.addWidget(self.components_names)
        self.layout.addWidget(self.stackedWidgets)

    def change_page(self, index):
        """Change the page."""
        self.stackedWidgets.setCurrentIndex(index)

    def update_robot_position(self, msg):
        """Update robot position in every layout."""
        self.component_pages["GlobalView"].update_map_position(msg)
        self.component_pages["GlobalView"].update_text_position(msg)
        self.component_pages["Motors"].update_map_position(msg)
        self.component_pages["Motors"].update_text_position(msg)


# Fenêtre Qt avec un label à mettre à jour
class OrchestratorGUI(QtWidgets.QMainWindow):
    """Display all the information and master interface."""

    def __init__(self):
        super().__init__()
        # Set main characteristics
        self.setWindowTitle("Orchestrator GUI")
        self.setGeometry(0, 0, 1000, 800)

        # ROS connection
        self.gui_node = NodeGUI(self)

        # GUI for each robot
        self.page_name_box = QtWidgets.QComboBox()
        self.page_name_box.addItems([name for name in self.gui_node.robot_names])
        self.robot_pages = {
            name: MainRobotPage(name, self) for name in self.gui_node.robot_names
        }
        self.stackedWidgets = QtWidgets.QStackedWidget()
        for val in self.robot_pages.values():
            self.stackedWidgets.addWidget(val)

        self.page_name_box.currentIndexChanged.connect(self.change_page)

        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        self.layout = QtWidgets.QVBoxLayout(central_widget)
        self.layout.addWidget(self.page_name_box)
        self.layout.addWidget(self.stackedWidgets)

        self._connect_to_ros()

    def update_robot_position(self, msg, name):
        """Update the position of the robot."""
        self.robot_pages[name].update_robot_position(msg)

    def send_future_position(self, x, y, theta, name):
        """Send the position request to node ROS."""
        self.gui_node.publish_command(x, y, theta, name)

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
        rclpy.spin_once(self.gui_node, timeout_sec=0.001)


def main():
    """Run main loop."""
    rclpy.init(args=None)
    app = QtWidgets.QApplication(sys.argv)
    window = OrchestratorGUI()
    window.show()
    app.exec_()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
