#!/usr/bin/env python3
"""Display all the information gathered by captors."""

import signal
import sys
import random
import rclpy
from rclpy.node import Node
import json
import math

# from PyQt5.QtChart import QChart, QChartView, QLineSeries
from PyQt5 import QtWidgets, QtGui, QtCore
from ament_index_python.packages import get_package_share_directory
import os
from opossum_msgs.msg import LidarLoc, GlobalView
from std_msgs.msg import String
from std_srvs.srv import Trigger
import functools
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from math import pi
from rclpy.logging import get_logger


class NodeGUI(Node):
    """Make the link between interface and ROS messages."""

    def __init__(self, parent=None):
        super().__init__("gui_node")
        self.parent = parent
        self._init_parameters()
        self._init_publishers()
        self._init_subscribers()
        self._init_clients()

    def _init_parameters(self) -> None:
        """Initialize parameters of the node."""
        self.declare_parameters(
            namespace="",
            parameters=[
                ("set_asserv", rclpy.Parameter.Type.BOOL),
                ("position_topic", rclpy.Parameter.Type.STRING),
                ("command_topic", rclpy.Parameter.Type.STRING),
                ("feedback_command_topic", rclpy.Parameter.Type.STRING),
                ("robot_names", rclpy.Parameter.Type.STRING_ARRAY),
                ("simulation", True)
            ],
        )
        self.robot_names = (
            self.get_parameter("robot_names").get_parameter_value().string_array_value
        )
        self.set_asserv = (
            self.get_parameter("set_asserv").get_parameter_value().bool_value
        )
        self.simulation = (
            self.get_parameter("simulation").get_parameter_value().bool_value
        )

    def _init_publishers(self):
        """Initialize subscribers of the node."""
        if self.simulation:
            self.command_topic = (
                self.get_parameter("command_topic").get_parameter_value().string_value
            )
            self.pub_command = {
                name: self.create_publisher(String, name + "/" + self.command_topic, 10)
                for name in self.robot_names
            }

    def _init_clients(self):
        if self.simulation:
            self.client_global_data = self.create_client(Trigger, 'send_global_data')
            while not self.client_global_data.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
        self.full_state_clients = {}
        for name in self.robot_names:
            self.full_state_clients[name] = self.create_client(Trigger, f"{name}/get_full_board_state")

    def _init_subscribers(self):
        """Initialize subscribers of the node."""
        if self.simulation:
            self.create_subscription(GlobalView, 'global_view', self.global_view_callback, 10)
        position_topic = (
            self.get_parameter("position_topic").get_parameter_value().string_value
        )
        for name in self.robot_names:
            self.create_subscription(
                LidarLoc,
                name + "/" + position_topic,
                functools.partial(self.position_callback, name=name),
                10,
            )
            self.create_subscription(
                String,
                f"{name}/board_state_updates",
                functools.partial(self.board_state_callback, name=name),
                10
            )
        if self.set_asserv:
            asserv_topic = (
                self.get_parameter("asserv_topic").get_parameter_value().string_value
            )
            for name in self.robot_names:
                self.create_subscription(
                    String,
                    name + "/" + asserv_topic + "/pos",
                    functools.partial(self.check_asserv_callback, name=name),
                    10,
                )
                self.create_subscription(
                    String,
                    name + "/" + asserv_topic + "/vel",
                    functools.partial(self.check_asserv_callback, name=name),
                    10,
                )

    def board_state_callback(self, msg, name):
        """Pass the delta JSON to the interface."""
        self.parent.update_robot_board_state(msg.data, name)

    def call_send_global_data(self):
        req = Trigger.Request()
        future = self.client_global_data.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def publish_command(self, name, command_name, args):
        """Publish the command for the robot."""
        command_msg = String()
        command = command_name.upper()
        for arg in args:
            command += " " + str(arg)
        command_msg.data = command
        self.pub_command[name].publish(command_msg)

    def global_view_callback(self, msg):
        self.parent.update_global_view(msg)

    def position_callback(self, msg, name):
        """Receive the last known information and display it."""
        self.parent.update_robot_position(msg, name)

    def check_asserv_callback(self, msg, name):
        """Check if an interesting data is received."""
        self.parent.update_robot_position(msg, name)

    def request_full_board_state(self, name):
        """Async request for the initial massive map dump."""
        client = self.full_state_clients[name]
        if client.wait_for_service(timeout_sec=0.5):
            req = Trigger.Request()
            future = client.call_async(req)
            future.add_done_callback(functools.partial(self.full_state_done_callback, name=name))
        else:
            self.get_logger().warn(f"Service {name}/get_full_board_state not available yet.")

    def full_state_done_callback(self, future, name):
        try:
            response = future.result()
            if response.success:
                self.parent.init_robot_board_state(response.message, name)
        except Exception as e:
            self.get_logger().error(f"Failed to get full state for {name}: {e}")


class MapScene(QtWidgets.QGraphicsView):
    """Connect to ROS and display information of a robot."""

    def __init__(self, name, parent=None, use_map=True):
        super().__init__()
        self.parent = parent
        self.name = name
        image_path = os.path.join(
            get_package_share_directory("opossum_dev_gui"), "images", "2026"
        )
        map = os.path.join(image_path, "map.png")
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
        self.crate_items = {}
        self.plier_items = {}
        self.robot_global_x = 0.0
        self.robot_global_y = 0.0
        self.robot_global_theta = 0.0
        self.plier_local_states = {}
        self.crate_local_states = {}

    def local_to_global(self, local_x, local_y, local_t):
        """Converts robot-frame coordinates to world-frame coordinates."""
        cos_t = math.cos(self.robot_global_theta)
        sin_t = math.sin(self.robot_global_theta)
        
        global_x = self.robot_global_x + (local_x * cos_t) - (local_y * sin_t)
        global_y = self.robot_global_y + (local_x * sin_t) + (local_y * cos_t)
        global_t = self.robot_global_theta + local_t
        
        return global_x, global_y, global_t

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

    def _redraw_pliers(self):
        """Redraw all pliers based on their saved local states and the robot's current pose."""
        for pid, plier_data in self.plier_local_states.items():
            
            # 1. Transform from local Robot frame to global Map frame
            gx, gy, gt = self.local_to_global(
                plier_data['x'], plier_data['y'], plier_data['theta']
            )

            if pid not in self.plier_items:
                # Create the 3cm circle if it doesn't exist
                r_px, _ = self.size_to_scene(0.03, 0.03)
                circle = QtWidgets.QGraphicsEllipseItem(-r_px/2, -r_px/2, r_px, r_px)
                circle.setPen(QtGui.QPen(QtCore.Qt.black))
                circle.setZValue(2.0)
                self.scene.addItem(circle)
                self.plier_items[pid] = circle

            item = self.plier_items[pid]
            px, py = self.pos_to_scene(gx, gy)
            
            item.setPos(px, py)
            item.setRotation(90 - (gt * 180 / math.pi)) 

            # Update Color (Green if holding something, White if empty)
            state = plier_data.get('state', -1)
            brush_color = QtGui.QColor(0, 255, 0) if state >= 0 else QtGui.QColor(255, 255, 255)
            item.setBrush(QtGui.QBrush(brush_color))

    def _redraw_crates(self):
        """Continuously update the position of crates being carried by the robot."""
        # Make sure the dictionaries exist before looping
        if not hasattr(self, 'crate_local_states') or not hasattr(self, 'plier_local_states'):
            return

        for cid, crate_data in self.crate_local_states.items():
            plier_id = crate_data.get('state', -1)
            
            # If the crate is held by a plier (state is the plier's ID)
            if plier_id != -1 and plier_id in self.plier_local_states:
                plier_data = self.plier_local_states[plier_id]
                
                if cid in self.crate_items:
                    # 1. Get the global position of the PLIER holding this crate
                    gx, gy, gt = self.local_to_global(
                        plier_data['x'], plier_data['y'], plier_data['theta']
                    )
                    
                    # 2. Move the crate to exactly match the plier's position and rotation
                    item = self.crate_items[cid]
                    px, py = self.pos_to_scene(gx, gy)
                    
                    item.setPos(px, py)
                    item.setRotation(90 - (gt * 180 / math.pi))

    @QtCore.pyqtSlot(LidarLoc)
    def update_map_position(self, msg):
        self.robot_global_x = msg.robot_position.x
        self.robot_global_y = msg.robot_position.y
        self.robot_global_theta = msg.robot_position.z
        """Update the robot position."""
        map_rect = self.map_item.boundingRect()
        width = map_rect.width()
        height = map_rect.height()
        posx = width * msg.robot_position.x / 3 - self.icon_width / 2
        posy = height * (1 - msg.robot_position.y / 2) - self.icon_height / 2
        new_pos = QtCore.QPointF(posx, posy)
        self.icon_item.setPos(new_pos)
        new_rotation = -msg.robot_position.z * 180 / pi
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
                    50, 50, QtCore.Qt.KeepAspectRatio
                )
                e_item = DraggablePixmapItem(e_pix)
                self.ennemis_items.append(e_item)
                e_x = width * rob.x / 3 - self.m_icon_width / 2
                e_y = height * (1 - rob.y / 2) - self.m_icon_height / 2
                e_pos = QtCore.QPointF(e_x, e_y)
                self.ennemis_items[index].setPos(e_pos)
                self.scene.addItem(self.ennemis_items[index])
            index += 1
        while len(self.ennemis_items) > index:
            old_item = self.ennemis_items.pop()
            self.scene.removeItem(old_item)
        self._redraw_pliers()
        self._redraw_crates()

    def pos_to_scene(self, x, y):
        """Convert real (x,y) meters to scene pixels."""
        map_rect = self.map_item.boundingRect()
        return map_rect.width() * x / 3.0, map_rect.height() * (1 - y / 2.0)

    def size_to_scene(self, w_m, h_m):
        """Convert real-world dimensions in meters to scene pixels."""
        map_rect = self.map_item.boundingRect()
        # Map is 3.0m wide by 2.0m high
        w_px = map_rect.width() * (w_m / 3.0)
        h_px = map_rect.height() * (h_m / 2.0)
        return w_px, h_px

    # # --- NEW: Coordinate helper ---
    # def pos_to_scene(self, x, y):
    #     """Convert real (x,y) meters to scene pixels."""
    #     map_rect = self.map_item.boundingRect()
    #     width = map_rect.width()
    #     height = map_rect.height()
    #     return width * x / 3.0, height * (1 - y / 2.0)

    # --- NEW: State Applicators ---
    def apply_full_state(self, json_str):
        """Initialize the whole board."""
        try:
            full_state = json.loads(json_str)
            for crate in full_state.get('crates', []):
                # --- NEW: Save the raw crate data ---
                self.crate_local_states[crate['id']] = crate 
                self._update_crate_visual(crate)
            
            for plier in full_state.get('pliers', []):
                self.plier_local_states[plier['id']] = plier
            self._redraw_pliers()
            # --- NEW: Trigger crate redraw too ---
            self._redraw_crates() 
            
        except Exception as e:
            print(f"Error parsing full state: {e}")

    def apply_update_state(self, json_str):
        """Update only the changed elements."""
        try:
            updates = json.loads(json_str)
            for crate in updates.get('crates', []):
                # --- NEW: Save the raw crate data ---
                self.crate_local_states[crate['id']] = crate
                self._update_crate_visual(crate)
                
            for plier in updates.get('pliers', []):
                self.plier_local_states[plier['id']] = plier
            self._redraw_pliers()
            # --- NEW: Trigger crate redraw too ---
            self._redraw_crates()
            
        except Exception as e:
            print(f"Error parsing update state: {e}")

    def _update_crate_visual(self, crate_data):
        # Crates are now correctly World Frame! No math needed here.
        cid = crate_data['id']
        
        if cid not in self.crate_items:
            w_px, h_px = self.size_to_scene(0.05, 0.15)
            rect = QtWidgets.QGraphicsRectItem(-w_px/2, -h_px/2, w_px, h_px) 
            rect.setPen(QtGui.QPen(QtCore.Qt.black))
            rect.setZValue(1.0)
            self.scene.addItem(rect)
            self.crate_items[cid] = rect

        item = self.crate_items[cid]
        px, py = self.pos_to_scene(crate_data['x'], crate_data['y'])
        
        item.setPos(px, py)
        item.setRotation(90 - (crate_data['theta'] * 180 / math.pi))

        # Color mapping: 0=Yellow, 1=Blue, 2=Red
        color_val = crate_data.get('color', -1)
        if color_val == 0: color = QtGui.QColor(255, 255, 0)
        elif color_val == 1: color = QtGui.QColor(0, 0, 255)
        elif color_val == 2: color = QtGui.QColor(255, 0, 0)
        else: color = QtGui.QColor(150, 150, 150)
        item.setBrush(QtGui.QBrush(color))

    def _update_plier_visual(self, plier_data):
        pid = plier_data['id']
        
        # 1. Transform from local Robot frame to global Map frame
        gx, gy, gt = self.local_to_global(
            plier_data['x'], plier_data['y'], plier_data['theta']
        )

        if pid not in self.plier_items:
            # Pliers are small, let's say 3cm (0.03m) circles
            r_px, _ = self.size_to_scene(0.03, 0.03)
            circle = QtWidgets.QGraphicsEllipseItem(-r_px/2, -r_px/2, r_px, r_px)
            circle.setPen(QtGui.QPen(QtCore.Qt.black))
            self.scene.addItem(circle)
            self.plier_items[pid] = circle

        item = self.plier_items[pid]
        px, py = self.pos_to_scene(gx, gy)
        
        item.setPos(px, py)
        
        # --- THE FIX ---
        item.setRotation(90 - (gt * 180 / math.pi))

        state = plier_data.get('state', -1)
        brush_color = QtGui.QColor(0, 255, 0) if state >= 0 else QtGui.QColor(255, 255, 255)
        item.setBrush(QtGui.QBrush(brush_color))

class DraggablePixmapItem(QtWidgets.QGraphicsPixmapItem):
    """Drag items on the Map."""

    def __init__(self, pixmap, parent=None, is_movable=True):
        super().__init__(pixmap, parent)
        # Permettre à cet item d'être déplacé et sélectionné
        if is_movable:
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
        self.button_leash = QtWidgets.QPushButton("REALEASE LEASH")
        self.button_au = QtWidgets.QPushButton("AUUUUUUUUU")
        self.map_scene = MapScene(name, self.parent, use_map=False)
        form_layout = QtWidgets.QFormLayout()
        self.button_leash.clicked.connect(functools.partial(self.send_command, command="LEASH"))
        self.button_au.clicked.connect(functools.partial(self.send_command, command="AU 1"))

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
        main_layout.addWidget(self.button_au)
        main_layout.addWidget(self.button_leash)
        
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

    def send_command(self, command):
        """Send command to ROS."""
        command_name = command
        args = []
        self.parent.send_cmd(self.name, command_name, args)

class MotorsPage(QtWidgets.QWidget):
    """Page dedicated for Motors."""

    def __init__(self, name, parent=None):
        super().__init__()
        self.parent = parent
        self.name = name
        main_layout = QtWidgets.QVBoxLayout(self)

        self.map_scene = MapScene(name, self.parent)
        grid_layout = QtWidgets.QGridLayout()
        self.button_cmd = QtWidgets.QPushButton("Send Command")
        self.button_update_lidar = QtWidgets.QPushButton("Update Value with lidar")

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
        theta_label = QtWidgets.QLabel("Theta (rad):")
        lin_vel_label = QtWidgets.QLabel("Linear Vel (m/s):")
        ang_vel_label = QtWidgets.QLabel("Angular Vel (rad/s):")

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

        self.button_cmd.clicked.connect(self.send_motor_command)
        self.button_update_lidar.clicked.connect(self.send_update_lidar)
        self.x_edit.returnPressed.connect(self.y_edit.setFocus)
        self.y_edit.returnPressed.connect(self.theta_edit.setFocus)
        self.theta_edit.returnPressed.connect(self.lin_vel_edit.setFocus)
        self.lin_vel_edit.returnPressed.connect(self.ang_vel_edit.setFocus)
        self.ang_vel_edit.returnPressed.connect(self.button_cmd.setFocus)

        main_layout.addWidget(self.map_scene)
        main_layout.addLayout(grid_layout)
        main_layout.addWidget(self.button_cmd)
        main_layout.addWidget(self.button_update_lidar)

    def send_update_lidar(self):
        """Send the command to update odom with lidar."""
        self.x_value_label.text()
        if self.x_value_label.text() != "--.--":
            self.parent.send_cmd(
                self.name,
                "SYNCHROLIDAR",
                [
                    float(self.x_value_label.text()),
                    float(self.y_value_label.text()),
                    float(self.t_value_label.text()),
                ],
            )

    def send_motor_command(self):
        """Send command to ROS."""
        x = self.x_edit.text()
        y = self.y_edit.text()
        theta = self.theta_edit.text()
        x = float(x) if x != "" else -1
        y = float(y) if y != "" else -1
        theta = float(theta) if theta != "" else -1
        if x != -1 or y != -1 or theta != -1:
            if x == -1:
                x = float(self.x_value_label.text())
            if y == -1:
                y = float(self.y_value_label.text())
            if theta == -1:
                theta = float(self.t_value_label.text())
            command_name = "MOVE"
            args = [x, y, theta]
            self.parent.send_cmd(self.name, command_name, args)
        lin_vel = self.lin_vel_edit.text()
        ang_vel = self.ang_vel_edit.text()
        lin_vel = float(lin_vel) if lin_vel != "" else -1
        ang_vel = float(ang_vel) if ang_vel != "" else -1
        if lin_vel != -1: 
            command_name = "VMAX"
            args = [lin_vel]
            self.parent.send_cmd(self.name, command_name, args)
            self.lin_vel_value_label.setText(f"{lin_vel:.2f}")
        if ang_vel != -1:
            command_name = "VTMAX"
            args = [ang_vel]
            self.parent.send_cmd(self.name, command_name, args)
            self.ang_vel_value_label.setText(f"{ang_vel:.2f}")

    def update_text_position(self, msg):
        """Update the text of the dynamic labels."""
        self.x_value_label.setText(f"{msg.robot_position.x:.2f}")
        self.y_value_label.setText(f"{msg.robot_position.y:.2f}")
        self.t_value_label.setText(f"{msg.robot_position.z:.2f}")

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
                # "Servo",
                # "Create Script",
                # "Asserv",
                # "Asserv Dynamic",
            ]
        )
        self.component_pages = {
            "GlobalView": GlobalViewPage(name, self.parent),
            "Motors": MotorsPage(name, self.parent),
            # "Servo": MapScene(name, self.parent),
            # "Create Script": MapScene(name, self.parent),
            # "Asserv": AsservPage(name, self.parent),
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

    # Inside MainRobotPage class
    def update_board_state(self, json_str):
        self.component_pages["GlobalView"].map_scene.apply_update_state(json_str)
        
    def init_board_state(self, json_str):
        self.component_pages["GlobalView"].map_scene.apply_full_state(json_str)

class GeneralViewPage(QtWidgets.QGraphicsView):
    def __init__(self, robot_names, parent):
        super().__init__()
        self.parent = parent
        self.robot_names = robot_names

        self.image_path = os.path.join(
            get_package_share_directory("opossum_dev_gui"), "images", "2026"
        )

        # Real sizes (meters)
        self.elements = {
            "map": (3.0, 2.0),
            "haz_crate*blue": (0.05, 0.15),
            "haz_crate*yellow": (0.05, 0.15),
            "haz_crate*rot": (0.05, 0.15),
            "cursor": (0.1, 0.045),
            "vaccum_gripper*free": (0.03, 0.01),
            "vaccum_gripper*pick": (0.03, 0.01),
            "vaccum_gripper*drop": (0.03, 0.01),
            "vaccum_gripper*revdrop": (0.03, 0.01),
            "vaccum_gripper*keeping": (0.03, 0.01),
            
        } | {name: (0.35, 0.35) for name in self.robot_names}

        self.scene = QtWidgets.QGraphicsScene(self)
        self.setScene(self.scene)

        # Load pixmaps
        self.pixmaps = {
            name: QtGui.QPixmap(os.path.join(self.image_path, f"{name.replace('*', '_')}.png"))
            for name in self.elements
        }

        # Add map
        self.map_item = QtWidgets.QGraphicsPixmapItem(self.pixmaps["map"])
        self.scene.addItem(self.map_item)
        self.scene.setSceneRect(QtCore.QRectF(self.pixmaps["map"].rect()))

        # Add robot items
        self.icons = {}
        for name in self.robot_names:
            item = QtWidgets.QGraphicsPixmapItem()
            self._scale_and_center_item(item, name, 1.0, "free")  # default scale
            item.setPos(5, 5)
            self.icons[name] = {"item": item, "state": "free", "type": name}
            self.scene.addItem(item)

        self.fitInView(self.scene.sceneRect(), QtCore.Qt.KeepAspectRatio)
        self.compute_initial_scale()   

    @QtCore.pyqtSlot(GlobalView)
    def update_global_view(self, msg):
        map_w, map_h = self.elements["map"]
        map_rect = self.map_item.boundingRect()
        width = map_rect.width()
        height = map_rect.height()

        for robot in msg.robots:
            posx = width * robot.x / map_w
            posy = height * (1 - robot.y / map_h)
            item = self.icons[robot.name]["item"]
            item.setPos(posx, posy)
            item.setRotation(-robot.theta * 180 / pi)
        
        to_update = []
        for elem in msg.objects:
            key = f"{elem.type}-{elem.id}"
            if key not in self.icons:
                item = QtWidgets.QGraphicsPixmapItem()
                if "vaccum_gripper" in elem.type:
                    item.setZValue(2.0)  # Actuators on top of crates
                elif "haz_crate" in elem.type:
                    item.setZValue(1.0)  # Crates below actuators
                else:
                    item.setZValue(1.5)  # Fallback for anything else
                self.icons[key] = {"item": item, "state": elem.state, "type": elem.type}
                self.scene.addItem(item)
                to_update.append(key)
            if elem.state != self.icons[key]["state"]:
                self.icons[key]["state"] = elem.state
                to_update.append(key)
            item = self.icons[key]["item"]
            if True:
                posx = width * elem.x / map_w
                posy = height * (1 - elem.y / map_h)
                item.setPos(posx, posy)
                item.setRotation(90 - elem.theta * 180 / pi)
        if to_update != []:
            self.compute_initial_scale(to_update)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.fitInView(self.map_item, QtCore.Qt.KeepAspectRatio)
        self.compute_initial_scale(self.icons.keys())

    def compute_initial_scale(self, to_update = []):
        map_w, map_h = self.elements["map"]
        item_rect = self.map_item.mapRectToScene(self.map_item.boundingRect())
        pixels_per_meter_x = item_rect.width() / map_w
        pixels_per_meter_y = item_rect.height() / map_h
        scale = min(pixels_per_meter_x, pixels_per_meter_y)
        for id in to_update:
            self._scale_and_center_item(self.icons[id]["item"], self.icons[id]["type"], scale, self.icons[id]["state"])
            
    def _scale_and_center_item(self, item, elem_type, scale, state):
        if elem_type in ("haz_crate", "vaccum_gripper"):
            mod_elem_type = elem_type + "*" + state.split("*")[-1]
        else:
            mod_elem_type = elem_type
        base_pixmap = self.pixmaps.get(mod_elem_type)
        if base_pixmap and not base_pixmap.isNull():
            real_w, real_h = self.elements[mod_elem_type]
            width_px = int(scale * real_w)
            height_px = int(scale * real_h)

            scaled_pixmap = base_pixmap.scaled(
                width_px, height_px,
                QtCore.Qt.IgnoreAspectRatio,  # Real-world size
                QtCore.Qt.SmoothTransformation
            )
            item.setPixmap(scaled_pixmap)

            w = scaled_pixmap.width()
            h = scaled_pixmap.height()
            item.setOffset(-w / 2, -h / 2)

# Fenêtre Qt avec un label à mettre à jour
class OrchestratorGUI(QtWidgets.QMainWindow):
    """Display all the information and master interface."""

    def __init__(self):
        super().__init__()
        # Set main characteristics
        self.setWindowTitle("Orchestrator GUI")
        self.setGeometry(0, 0, 600, 800)

        # ROS connection
        self.gui_node = NodeGUI(self)
        self.leash_button = QtWidgets.QPushButton("Send all LEASH")
        self.leash_button.clicked.connect(self.send_leashes)

        # GUI for each robot
        self.page_name_box = QtWidgets.QComboBox()
        self.page_name_box.addItems(["General View"] + [name for name in self.gui_node.robot_names])
        self.robot_pages = {
            name: MainRobotPage(name, self) for name in self.gui_node.robot_names
        }
        self.robot_pages['General View'] = GeneralViewPage(self.gui_node.robot_names, self)
        self.stackedWidgets = QtWidgets.QStackedWidget()
        for page_name in [self.page_name_box.itemText(i) for i in range(self.page_name_box.count())]:
            self.stackedWidgets.addWidget(self.robot_pages[page_name])
        
        self.page_name_box.currentIndexChanged.connect(self.change_page)

        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        self.layout = QtWidgets.QVBoxLayout(central_widget)
        self.layout.addWidget(self.leash_button)
        self.layout.addWidget(self.page_name_box)
        self.layout.addWidget(self.stackedWidgets)

        self._connect_to_ros()
        self.call_update_global()
        self.fetch_initial_states()

    def update_robot_position(self, msg, name):
        """Update the position of the robot."""
        self.robot_pages[name].update_robot_position(msg)

    def update_global_view(self, msg):
        self.robot_pages['General View'].update_global_view(msg)

    def call_update_global(self):
        if self.gui_node.simulation:
            self.gui_node.call_send_global_data()

    def send_cmd(self, name, command_name, args):
        """Send the command request to node ROS."""
        if self.gui_node.simulation:
            self.gui_node.publish_command(name, command_name, args)

    def send_leashes(self):
        if self.gui_node.simulation:
            for name in self.gui_node.robot_names:
                self.send_cmd(name, "LEASH", [])

    def update_robot_board_state(self, json_str, name):
        self.robot_pages[name].update_board_state(json_str)

    def init_robot_board_state(self, json_str, name):
        self.robot_pages[name].init_board_state(json_str)
        
    def fetch_initial_states(self):
        for name in self.gui_node.robot_names:
            self.gui_node.request_full_board_state(name)
   
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
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    app = QtWidgets.QApplication(sys.argv)
    window = OrchestratorGUI()
    window.show()
    sys.exit(app.exec_())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
