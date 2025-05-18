#!/usr/bin/env python3
"""Display all the information gathered by captors."""

import sys
import random

# from PyQt5.QtChart import QChart, QChartView, QLineSeries
from PyQt5 import QtWidgets, QtGui, QtCore
from ament_index_python.packages import get_package_share_directory
import os
import numpy as np
from opossum_msgs.msg import LidarLoc
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import websocket
import threading
import json
import yaml


def load_config(file_path):
    """Load the YAML config."""
    with open(file_path, "r") as f:
        config = yaml.safe_load(f)
    return config


class RosbridgeClient:
    """Connect GUI to ROS through rosbridge."""

    def __init__(self, parent):
        config_params_path = os.path.join(
            get_package_share_directory("opossum_dev_gui"),
            "config",
            "dev_gui_params.yaml",
        )
        config_params = load_config(config_params_path)
        config = config_params["dev_gui_node"]["ros__parameters"]

        # Access settings
        self.position_topic = config["position_topic"]
        self.command_topic = config["command_topic"]
        self.rosbridge_ip = config["rosbridge_ip"]

        self.parent = parent
        self.robot_names = ["main_robot"]  # We will fill it when needed

        self.ws = websocket.WebSocketApp(
            f"ws://{self.rosbridge_ip}:9090",
            on_message=self.on_message,
            on_open=self.on_open,
            on_error=self.on_error,
            on_close=self.on_close,
        )

        self.thread = threading.Thread(target=self.ws.run_forever)
        self.thread.daemon = True
        self.thread.start()

        self.connected = False

    def on_open(self, ws):
        """Open the rosbridge connection."""
        print("[RosbridgeClient] Connected to rosbridge!")
        self.connected = True
        # Subscribe to topics once connection is open
        self._subscribe_to_all()

    def on_error(self, ws, error):
        """Ouput the error fo Rosbridge."""
        print(f"[RosbridgeClient] Error: {error}")

    def on_close(self, ws, close_status_code, close_msg):
        """Advert the closing connection."""
        print("[RosbridgeClient] Disconnected from rosbridge.")

    def _subscribe_to_all(self):
        """Subscribe to all robot position topics."""
        for name in self.robot_names:
            topic = f"/{name}/{self.position_topic}"
            msg = {"op": "subscribe", "topic": topic, "type": "opossum_msgs/msg/LidarLoc"}
            self.ws.send(json.dumps(msg))
            print(f"[RosbridgeClient] Subscribed to {topic}")

    def configure_robots(self):
        """Set robot names and subscribe when ready."""
        if self.connected:
            self._subscribe_to_all()

    def publish_command(self, name, command_name, args):
        """Publish a command message over rosbridge."""
        topic = f"/{name}/{self.command_topic}"
        command = command_name.upper() + " " + " ".join(map(str, args))
        msg = {"op": "publish", "topic": topic, "msg": {"data": command}}
        self.ws.send(json.dumps(msg))
        print(f"[RosbridgeClient] Published to {topic}: {command}")

    def on_message(self, ws, message):
        """Handle incoming messages."""
        data = json.loads(message)
        if data.get("op") == "publish":
            topic = data["topic"]
            msg = data["msg"]
            # Extract robot name from topic
            parts = topic.split("/")
            if len(parts) >= 2:
                robot_name = parts[1]
                # Update the GUI
                QtCore.QMetaObject.invokeMethod(
                    self.parent,
                    "update_robot_position_from_json",
                    QtCore.Qt.QueuedConnection,
                    QtCore.Q_ARG(dict, msg),
                    QtCore.Q_ARG(str, robot_name),
                )


class MapScene(QtWidgets.QGraphicsView):
    """Connect to ROS and display information of a robot."""

    def __init__(self, name, parent=None, use_map=True):
        super().__init__()
        self.parent = parent
        self.name = name
        image_path = os.path.join(
            get_package_share_directory("opossum_dev_gui"), "images"
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
            self.map_item.mousePressEvent = lambda event: self.send_pos_goal(event)
        self.scene.addItem(self.map_item)

        # Chargement de l'icône et ajout d'un item déplaçable
        self.icon_pixmap = QtGui.QPixmap(icon).scaled(50, 50, QtCore.Qt.KeepAspectRatio)
        self.icon_item = DraggablePixmapItem(self.icon_pixmap)
        icon_rect = self.icon_item.boundingRect()
        self.icon_width = icon_rect.width()
        self.icon_height = icon_rect.height()
        self.icon_item.setPos(5, 5)  # Position initiale
        m_pixmap = QtGui.QPixmap(self.mad_icon).scaled(
            50, 50, QtCore.Qt.KeepAspectRatio
        )
        dragmap = DraggablePixmapItem(m_pixmap)
        m_icon_rect = dragmap.boundingRect()
        self.m_icon_width = m_icon_rect.width()
        self.m_icon_height = m_icon_rect.height()
        self.ennemis_items = []
        self.scene.addItem(self.icon_item)
        self.fitInView(self.scene.sceneRect(), QtCore.Qt.KeepAspectRatio)

    def send_pos_goal(self, event):
        """Send the goal position on the button click."""
        x, y = self.get_real_pos(event.pos().x(), event.pos().y())
        command_name = "MOVE"
        args = [x, y, 0.0]
        self.parent.send_cmd(self.name, command_name, args)

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
        new_rotation = -msg.robot_position.z * 180 / np.pi
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
                # self.scene.addItem(self.ennemis_items[index])
            index += 1
        # logger = get_logger("HEY")
        # logger.info(f"NUM robo: {len(self.ennemis_items)}")
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

        self.map_scene = MapScene(name, self.parent, use_map=False)
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
        grid_layout = QtWidgets.QGridLayout()
        self.button = QtWidgets.QPushButton("Send Command")

        # Create QLabel objects for the static labels (the keys)
        # and QLabel objects for the dynamic values.
        self.x_value_label = QtWidgets.QLabel("--.--")
        self.y_value_label = QtWidgets.QLabel("--.--")
        self.t_value_label = QtWidgets.QLabel("--.--")
        self.lin_vel_value_label = QtWidgets.QLabel("--.--")
        self.ang_vel_value_label = QtWidgets.QLabel("--.--")

        # Create labels for each parameter
        current_value_label_x = QtWidgets.QLabel("Current Value X:")
        current_value_label_y = QtWidgets.QLabel("Current Value Y:")
        current_value_label_t = QtWidgets.QLabel("Current Value Theta:")
        current_value_label_lin_vel = QtWidgets.QLabel("Current Value Linear Vel:")
        current_value_label_ang_vel = QtWidgets.QLabel("Current Value Angle Vel:")
        x_label = QtWidgets.QLabel("X (m):")
        y_label = QtWidgets.QLabel("Y (m):")
        theta_label = QtWidgets.QLabel("Theta (deg):")
        lin_vel_label = QtWidgets.QLabel("Linear Vel (m/s):")
        ang_vel_label = QtWidgets.QLabel("Angular Vel (deg/s):")

        # Create line edits for each parameter
        self.x_edit = QtWidgets.QLineEdit(self)
        self.x_edit.setPlaceholderText("Enter x value")
        self.y_edit = QtWidgets.QLineEdit(self)
        self.y_edit.setPlaceholderText("Enter y value")
        self.theta_edit = QtWidgets.QLineEdit(self)
        self.theta_edit.setPlaceholderText("Enter theta value")
        self.lin_vel_edit = QtWidgets.QLineEdit(self)
        self.lin_vel_edit.setPlaceholderText("Enter linear velocity value")
        self.ang_vel_edit = QtWidgets.QLineEdit(self)
        self.ang_vel_edit.setPlaceholderText("Enter angle velocity value")

        # Place widgets in the grid
        grid_layout.addWidget(x_label, 0, 0)
        grid_layout.addWidget(self.x_edit, 0, 1)
        grid_layout.addWidget(current_value_label_x, 0, 2)
        grid_layout.addWidget(self.x_value_label, 0, 3)
        grid_layout.addWidget(y_label, 1, 0)
        grid_layout.addWidget(self.y_edit, 1, 1)
        grid_layout.addWidget(current_value_label_y, 1, 2)
        grid_layout.addWidget(self.y_value_label, 1, 3)
        grid_layout.addWidget(theta_label, 2, 0)
        grid_layout.addWidget(self.theta_edit, 2, 1)
        grid_layout.addWidget(current_value_label_t, 2, 2)
        grid_layout.addWidget(self.t_value_label, 2, 3)
        grid_layout.addWidget(lin_vel_label, 3, 0)
        grid_layout.addWidget(self.lin_vel_edit, 3, 1)
        grid_layout.addWidget(current_value_label_lin_vel, 3, 2)
        grid_layout.addWidget(self.lin_vel_value_label, 3, 3)
        grid_layout.addWidget(ang_vel_label, 4, 0)
        grid_layout.addWidget(self.ang_vel_edit, 4, 1)
        grid_layout.addWidget(current_value_label_ang_vel, 4, 2)
        grid_layout.addWidget(self.ang_vel_value_label, 4, 3)

        self.button.clicked.connect(self.send_motor_command)
        self.x_edit.returnPressed.connect(self.y_edit.setFocus)
        self.y_edit.returnPressed.connect(self.theta_edit.setFocus)
        self.theta_edit.returnPressed.connect(self.lin_vel_edit.setFocus)
        self.lin_vel_edit.returnPressed.connect(self.ang_vel_edit.setFocus)
        self.ang_vel_edit.returnPressed.connect(self.button.setFocus)

        main_layout.addWidget(self.map_scene)
        main_layout.addLayout(grid_layout)
        main_layout.addWidget(self.button)

    def send_motor_command(self):
        """Send command to ROS."""
        x = self.x_edit.text()
        y = self.y_edit.text()
        theta = self.theta_edit.text()
        x = float(x) if x != "" else -1
        y = float(y) if y != "" else -1
        theta = float(theta) * np.pi / 180 if theta != "" else -1
        if x != -1 or y != -1 or theta != -1:
            command_name = "MOVE"
            args = [x, y, theta]
            self.parent.send_cmd(self.name, command_name, args)
        lin_vel = self.lin_vel_edit.text()
        ang_vel = self.ang_vel_edit.text()
        lin_vel = float(lin_vel) if lin_vel != "" else -1
        ang_vel = float(ang_vel) if ang_vel != "" else -1
        if lin_vel != -1 or ang_vel != -1:
            command_name = "SPEED"
            args = [lin_vel, lin_vel, ang_vel]
            self.parent.send_cmd(self.name, command_name, args)
            self.lin_vel_value_label.setText(f"{lin_vel:.2f}")
            self.ang_vel_value_label.setText(f"{ang_vel:.2f}")

    def update_text_position(self, msg):
        """Update the text of the dynamic labels."""
        self.x_value_label.setText(f"{msg.robot_position.x:.2f}")
        self.y_value_label.setText(f"{msg.robot_position.y:.2f}")
        self.t_value_label.setText(f"{msg.robot_position.z * 180 / np.pi:.2f}")

    def update_map_position(self, msg):
        """Update the map of the robots."""
        self.map_scene.update_map_position(msg)


class AsservPage(QtWidgets.QWidget):
    """Page dedicated for Motors."""

    def __init__(self, name, parent=None):
        super().__init__()
        self.parent = parent
        self.name = name
        main_layout = QtWidgets.QVBoxLayout(self)

        grid_layout = QtWidgets.QGridLayout()
        self.button = QtWidgets.QPushButton("Send Command")
        cmd_label = QtWidgets.QLabel("Command: ")
        args_label = QtWidgets.QLabel("Args:")

        # Create line edits for each parameter
        self.cmd_edit = QtWidgets.QLineEdit(self)
        self.cmd_edit.setPlaceholderText("Enter command")
        self.args_edit = QtWidgets.QLineEdit(self)
        self.args_edit.setPlaceholderText("Enter Arguments")

        # Place widgets in the grid
        grid_layout.addWidget(cmd_label, 0, 0)
        grid_layout.addWidget(self.cmd_edit, 0, 1)
        grid_layout.addWidget(args_label, 1, 0)
        grid_layout.addWidget(self.args_edit, 1, 1)

        self.button.clicked.connect(self.send_command)
        self.cmd_edit.returnPressed.connect(self.args_edit.setFocus)
        self.args_edit.returnPressed.connect(self.button.setFocus)

        main_layout.addLayout(grid_layout)
        main_layout.addWidget(self.button)
        # Create the main layout

        # Create the plot canvas and add it to the layout
        self.canvas = PlotCanvas(self, width=5, height=4, dpi=100)
        main_layout.addWidget(self.canvas)

        self.button2 = QtWidgets.QPushButton("Update Graph", self)
        self.button2.clicked.connect(self.canvas.update_plot)
        main_layout.addWidget(self.button2)

    def send_command(self):
        """Send command to ROS node."""
        command_name = self.cmd_edit.text()
        args = self.args_edit.text().split()
        self.parent.send_cmd(self.name, command_name, args)


class PlotCanvas(FigureCanvas):
    """Plot in a graph the result of asserv (Not implemented yet well)."""

    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.ax = self.fig.add_subplot(111)
        super().__init__(self.fig)
        self.setParent(parent)
        self.plot_initial()

    def plot_initial(self):
        """Plot randomly to init."""
        self.ax.clear()
        self.ax.plot([i for i in range(10)], "r-")
        self.ax.set_title("Initial Plot")
        self.draw()

    def update_plot(self):
        """Update the plot."""
        self.ax.clear()
        data = [random.randint(0, 10) for _ in range(10)]
        self.ax.plot(data, "b-")
        self.ax.set_title("Updated Plot")
        self.draw()


class MainRobotPage(QtWidgets.QWidget):
    """Page for ecah robot."""

    def __init__(self, name, parent=None):
        super().__init__()
        self.parent = parent
        self.name = name
        self.components_names = QtWidgets.QComboBox()
        self.components_names.addItems(
            [
                "Global View",
                "Motors",
                "Servo",
                "Create Script",
                "Asserv",
                "Asserv Dynamic",
            ]
        )
        self.component_pages = {
            "GlobalView": GlobalViewPage(name, self.parent),
            "Motors": MotorsPage(name, self.parent),
            "Servo": MapScene(name, self.parent),
            "Create Script": MapScene(name, self.parent),
            "Asserv": AsservPage(name, self.parent),
            # "AsservDynamic": ChartWidget(name, self.parent),
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
        self.setGeometry(0, 0, 600, 800)

        # ROS connection
        self.gui_node = RosbridgeClient(self)
        self.gui_node.configure_robots()

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

    @QtCore.pyqtSlot(dict, str)
    def update_robot_position_from_json(self, msg, name):
        """Update the robot position from JSON message."""

        class RobotPos:
            def __init__(self, pos):
                self.x = pos.get("x", 0.0)
                self.y = pos.get("y", 0.0)
                self.z = pos.get("z", 0.0)

        class LidarLocMsg:
            def __init__(self, data):
                self.robot_position = RobotPos(data.get("robot_position", {}))
                self.other_robot_position = [
                    RobotPos(p) for p in data.get("other_robot_position", [])
                ]

        lidar_loc_msg = LidarLocMsg(msg)
        self.update_robot_position(lidar_loc_msg, name)

    def update_robot_position(self, msg, name):
        """Update the position of the robot."""
        self.robot_pages[name].update_robot_position(msg)

    def send_cmd(self, name, command_name, args):
        """Send the command request to node ROS."""
        self.gui_node.publish_command(name, command_name, args)

    def change_page(self, index):
        """Change the page."""
        self.stackedWidgets.setCurrentIndex(index)


def main():
    """Run main loop."""
    app = QtWidgets.QApplication(sys.argv)
    window = OrchestratorGUI()
    window.show()
    app.exec_()


if __name__ == "__main__":
    main()
