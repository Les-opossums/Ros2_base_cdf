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
from opossum_msgs.msg import LidarLoc, GlobalView, RobotData
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
        self.msg_lidar = None

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
                functools.partial(self.update_lidar_data, name=name),
                10,
            )
            if not self.parent.use_only_lidar_points:
                self.create_subscription(
                    RobotData,
                    name + "/" + "robot_data",
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

    def update_lidar_data(self, msg, name):
        """Update the lidar data."""
        self.msg_lidar = msg
        if self.parent.use_only_lidar_points:
            self.parent.update_robot_position(msg, name)
        if self.parent.use_ghost_lidar_points:
            self.parent.update_ghost_position(msg, name)

    def position_callback(self, msg, name):
        """Receive the last known information and display it."""
        new_msg = LidarLoc()
        new_msg.robot_position.x = msg.x
        new_msg.robot_position.y = msg.y
        new_msg.robot_position.z = msg.theta
        if self.msg_lidar is not None:  
            new_msg.other_robot_position = self.msg_lidar.other_robot_position
            new_msg.balises = self.msg_lidar.balises

        self.parent.update_robot_position(new_msg, name)

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
        self.icon_item.setZValue(0.2) # Ensure main robot is always on top

        # --- NEW: GHOST ROBOT ADDITION ---
        # is_movable=False so you don't accidentally drag the ghost!
        self.ghost_item = DraggablePixmapItem(self.icon_pixmap, is_movable=False)
        self.ghost_item.setScale(1.15)   # <-- NEW: Makes it exactly 15% larger!
        self.ghost_item.setOpacity(0.4)  # 40% opaque (0.0 is invisible, 1.0 is solid)
        self.ghost_item.setZValue(0.1)   # Sit just below the main robot
        self.scene.addItem(self.ghost_item)
        # ---------------------------------
        
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
        
        # --- NEW: Zone variables and Path selector ---
        self.zone_local_states = {}
        self.zone_items = {}
        self.path_items = []
        self.max_paths_to_show = 3
        self.morbidity_data = {}
        self.morbidity_items = []

        # Create a dropdown menu floating on the View
        self.path_selector = QtWidgets.QComboBox(self)
        self.path_selector.addItems(["0 paths", "1 path", "2 paths", "3 paths"])
        self.path_selector.setCurrentIndex(3) # Default to showing 3
        self.path_selector.move(10, 10) # Position in the top-left corner
        self.path_selector.setStyleSheet("background-color: white; font-weight: bold;")
        self.path_selector.currentIndexChanged.connect(self.set_max_paths)

    # --- NEW METHOD: Triggered by the dropdown ---
    def set_max_paths(self, index):
        """Update how many paths to show based on dropdown selection."""
        self.max_paths_to_show = index
        self._redraw_paths()

    # --- ADD THIS NEW METHOD ANYWHERE INSIDE MapScene ---
    def set_ghost_visibility(self, visible: bool):
        """Instantly hide or show the ghost item on the map."""
        if hasattr(self, 'ghost_item'):
            self.ghost_item.setVisible(visible)

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

            # Update Color: Purple if running, Green if holding, White if free
            is_running = plier_data.get('is_running', False)
            state = plier_data.get('state', -1)
            
            if is_running:
                brush_color = QtGui.QColor(170, 0, 255) # Purple
            elif state >= 0:
                brush_color = QtGui.QColor(0, 255, 0)   # Green
            else:
                brush_color = QtGui.QColor(255, 255, 255) # White
                
            item.setBrush(QtGui.QBrush(brush_color))

    def _update_plier_visual(self, plier_data):
        pid = plier_data['id']
        gx, gy, gt = self.local_to_global(
            plier_data['x'], plier_data['y'], plier_data['theta']
        )

        if pid not in self.plier_items:
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

        # Update Color: Purple if running, Green if holding, White if free
        is_running = plier_data.get('is_running', False)
        state = plier_data.get('state', -1)
        
        if is_running:
            brush_color = QtGui.QColor(170, 0, 255) # Purple
        elif state >= 0:
            brush_color = QtGui.QColor(0, 255, 0)   # Green
        else:
            brush_color = QtGui.QColor(255, 255, 255) # White
            
        item.setBrush(QtGui.QBrush(brush_color))

    def _redraw_crates(self):
        """Continuously update the position and visuals of crates being carried or targeted."""
        if not hasattr(self, 'crate_local_states') or not hasattr(self, 'plier_local_states'):
            return

        for cid, crate_data in self.crate_local_states.items():
            plier_id = crate_data.get('state', -1)
            item = self.crate_items.get(cid)
            
            if not item:
                continue

            # If the crate is linked to a plier
            if plier_id != -1 and plier_id in self.plier_local_states:
                plier_data = self.plier_local_states[plier_id]
                
                # 1. Move the crate to perfectly track the plier
                gx, gy, gt = self.local_to_global(
                    plier_data['x'], plier_data['y'], plier_data['theta']
                )
                px, py = self.pos_to_scene(gx, gy)
                item.setPos(px, py)
                item.setRotation(90 - (gt * 180 / math.pi))

                # 2. Apply "Held" Visuals
                item.setOpacity(0.5)  # 50% transparent as long as it is linked to a plier
                item.setPen(QtGui.QPen(QtCore.Qt.black)) # Keep the normal solid black outline
                    
            else:
                # 3. Apply "Free" Visuals (Crate is sitting on the board)
                item.setOpacity(1.0) # Full opacity
                item.setPen(QtGui.QPen(QtCore.Qt.black)) # Normal black outline

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
        self._redraw_paths()

    @QtCore.pyqtSlot(LidarLoc)
    def update_ghost_position(self, msg):
        """Update the position of the ghost in the map."""
        # Only update if the item exists
        if not hasattr(self, 'ghost_item'):
            return

        map_rect = self.map_item.boundingRect()
        width = map_rect.width()
        height = map_rect.height()
        
        # Calculate position using the exact same math as the main robot
        posx = width * msg.robot_position.x / 3.0 - self.icon_width / 2.0
        posy = height * (1.0 - msg.robot_position.y / 2.0) - self.icon_height / 2.0
        
        new_pos = QtCore.QPointF(posx, posy)
        self.ghost_item.setPos(new_pos)
        
        # Calculate and set rotation
        new_rotation = -msg.robot_position.z * 180.0 / pi
        self.ghost_item.setRotation(new_rotation)

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

    def resizeEvent(self, event):
        """Make the map dynamically adjust when the window is resized."""
        super().resizeEvent(event)
        # Force the view to keep the map item fully visible and scaled
        if hasattr(self, 'map_item'):
            self.fitInView(self.map_item, QtCore.Qt.KeepAspectRatio)

    def apply_full_state(self, json_str):
        """Initialize the whole board."""
        try:
            full_state = json.loads(json_str)
            
            # --- CLEAR EXISTING CRATES TO AVOID GHOSTS ---
            for cid, item in self.crate_items.items():
                self.scene.removeItem(item)
            self.crate_items.clear()
            self.crate_local_states.clear()
            
            if 'morbidity' in full_state: # or full_state
                self.morbidity_data = full_state['morbidity']
                self._redraw_morbidity()

            for crate in full_state.get('crates', []):
                self.crate_local_states[crate['id']] = crate 
                self._update_crate_visual(crate)
                
            for plier in full_state.get('pliers', []):
                self.plier_local_states[plier['id']] = plier
                
            # --- NEW: Process Zones ---
            for zone in full_state.get('zones', []):
                self.zone_local_states[zone['id']] = zone
                self._update_zone_visual(zone)

            self._redraw_pliers()
            self._redraw_crates()
            self._redraw_paths()
            
        except Exception as e:
            print(f"Error parsing full state: {e}")

    def apply_update_state(self, json_str):
        """Update only the changed elements."""
        try:
            updates = json.loads(json_str)
            
            if 'morbidity' in updates:
                self.morbidity_data = updates['morbidity']
                self._redraw_morbidity()

            # ==========================================
            # 1. PROCESS DELETED CRATES (NEW LOGIC)
            # ==========================================
            for cid in updates.get('deleted_crates', []):
                # Remove from data tracking
                if cid in self.crate_local_states:
                    del self.crate_local_states[cid]
                
                # Remove from the visual map!
                if cid in self.crate_items:
                    item = self.crate_items.pop(cid)
                    self.scene.removeItem(item)

            # ==========================================
            # 2. Process New/Updated Elements
            # ==========================================
            for crate in updates.get('crates', []):
                self.crate_local_states[crate['id']] = crate
                self._update_crate_visual(crate)
                
            for plier in updates.get('pliers', []):
                self.plier_local_states[plier['id']] = plier
                
            for zone in updates.get('zones', []):
                self.zone_local_states[zone['id']] = zone
                self._update_zone_visual(zone)

            self._redraw_pliers()
            self._redraw_crates()
            self._redraw_paths()
            
        except Exception as e:
            print(f"Error parsing update state: {e}")

    def _update_crate_visual(self, crate_data):
        cid = crate_data['id']
        reward = crate_data.get('reward', float('-inf'))

        # 1. Format the text to show ONLY the Score
        if reward > -99999:
            display_text = f"{reward:.2f}"
        else:
            display_text = "-"

        # 2. Create the crate if it doesn't exist yet
        if cid not in self.crate_items:
            w_px, h_px = self.size_to_scene(0.05, 0.15)
            rect = QtWidgets.QGraphicsRectItem(-w_px/2, -h_px/2, w_px, h_px) 
            rect.setPen(QtGui.QPen(QtCore.Qt.black))
            rect.setZValue(1.0)
            
            # Add the Text Item
            text_item = QtWidgets.QGraphicsSimpleTextItem(display_text, rect)
            font = QtGui.QFont("Arial", 10, QtGui.QFont.Bold) # Larger font since it's just one line
            text_item.setFont(font)
            
            self.scene.addItem(rect)
            self.crate_items[cid] = rect

        # 3. Update Position and Rotation
        item = self.crate_items[cid]
        px, py = self.pos_to_scene(crate_data['x'], crate_data['y'])
        item.setPos(px, py)
        item.setRotation(90 - (crate_data['theta'] * 180 / math.pi))

        # 4. Color mapping
        color_val = crate_data.get('color', -1)
        text_color = QtCore.Qt.white # Default text color

        if color_val == 0: 
            color = QtGui.QColor(255, 255, 0)
            text_color = QtCore.Qt.black # Yellow needs black text to be readable
        elif color_val == 1: 
            color = QtGui.QColor(0, 0, 255)
        elif color_val == 2: 
            color = QtGui.QColor(255, 0, 0)
        else: 
            color = QtGui.QColor(150, 150, 150)
            text_color = QtCore.Qt.black
            
        item.setBrush(QtGui.QBrush(color))
        
        # 5. --- THE FIX: Dynamically update, ROTATE, and Center the Text ---
        for child in item.childItems():
            if isinstance(child, QtWidgets.QGraphicsSimpleTextItem):
                child.setText(display_text)
                child.setBrush(QtGui.QBrush(text_color))
                
                # Setup rotation around the exact center of the text bounding box
                text_rect = child.boundingRect()
                child.setTransformOriginPoint(text_rect.width() / 2, text_rect.height() / 2)
                
                # Rotate it 90 degrees!
                child.setRotation(90) 
                
                # Center the rotated text item perfectly inside the crate
                child.setPos(-text_rect.width() / 2, -text_rect.height() / 2)

    def _update_zone_visual(self, zone_data):
        """Draw or update the release zones with their rewards."""
        zid = zone_data['id']
        reward = zone_data.get('reward', float('-inf'))
        
        if reward > -99999:
            display_text = f"{reward:.3f}"
            # display_text = f"Zone {zid}\n{reward:.3f}"
        else:
            display_text = f"---"
            # display_text = f"Zone {zid}\n-"
            
        # Create the zone if it doesn't exist
        if zid not in self.zone_items:
            # Assume zone is roughly 30x30 cm. Adjust 0.3 if needed.
            w_px, h_px = self.size_to_scene(0.2, 0.2)
            rect = QtWidgets.QGraphicsRectItem(-w_px/2, -h_px/2, w_px, h_px)
            
            # Semi-transparent green background with dashed border
            rect.setBrush(QtGui.QBrush(QtGui.QColor(0, 255, 0, 40))) 
            rect.setPen(QtGui.QPen(QtCore.Qt.darkGreen, 2, QtCore.Qt.DashLine))
            rect.setZValue(0.1) # Keep it on the floor
            
            # Add reward text
            text_item = QtWidgets.QGraphicsSimpleTextItem(display_text, rect)
            text_item.setFont(QtGui.QFont("Arial", 9, QtGui.QFont.Bold))
            text_item.setBrush(QtGui.QBrush(QtCore.Qt.black))
            
            self.scene.addItem(rect)
            self.zone_items[zid] = rect

        item = self.zone_items[zid]
        px, py = self.pos_to_scene(zone_data['x'], zone_data['y'])
        item.setPos(px, py)
        
        # Update text and keep it centered
        for child in item.childItems():
            if isinstance(child, QtWidgets.QGraphicsSimpleTextItem):
                child.setText(display_text)
                r = child.boundingRect()
                child.setPos(-r.width()/2, -r.height()/2)

    def _redraw_paths(self):
        """Draw top N paths for both Picking (Crates) and Releasing (Zones)."""
        # 1. Clear old paths from the screen
        for item in getattr(self, 'path_items', []):
            if item in self.scene.items():
                self.scene.removeItem(item)
        self.path_items = []

        # If user selected "0 paths", stop here.
        if self.max_paths_to_show == 0:
            return

        # 2. Get valid crates and zones, and sort them highest to lowest score
        valid_crates = [c for c in self.crate_local_states.values() if c.get('reward', float('-inf')) > -99999 and 'path' in c]
        valid_crates.sort(key=lambda c: c['reward'], reverse=True)

        valid_zones = [z for z in getattr(self, 'zone_local_states', {}).values() if z.get('reward', float('-inf')) > -99999 and 'path' in z]
        valid_zones.sort(key=lambda z: z['reward'], reverse=True)

        # 3. Helper function to draw a line
        def draw_trajectory(path_pts, color, is_first):
            if not path_pts or len(path_pts) < 2: 
                return
            qpath = QtGui.QPainterPath()
            sx, sy = self.pos_to_scene(path_pts[0][0], path_pts[0][1])
            qpath.moveTo(sx, sy)
            for pt in path_pts[1:]:
                px, py = self.pos_to_scene(pt[0], pt[1])
                qpath.lineTo(px, py)
            
            item = QtWidgets.QGraphicsPathItem(qpath)
            pen = QtGui.QPen(color)
            # Make the best path 6px thick, others 3px thick
            pen.setWidth(6 if is_first else 3)
            pen.setStyle(QtCore.Qt.DashLine)
            item.setPen(pen)
            item.setZValue(0.8) # Above floor, below crates
            self.scene.addItem(item)
            self.path_items.append(item)

        # 4. Draw Top N Picking Paths (Shades of Blue)
        blue_shades = [QtGui.QColor(0, 0, 255, 200), QtGui.QColor(0, 150, 255, 200), QtGui.QColor(100, 200, 255, 200)]
        drawn_crate_paths = []
        for crate in valid_crates:
            if len(drawn_crate_paths) >= self.max_paths_to_show: 
                break
            
            # Prevent drawing the exact same path twice for crates in the same stack
            if crate['path'] in drawn_crate_paths: 
                continue
            
            is_best = (len(drawn_crate_paths) == 0)
            draw_trajectory(crate['path'], blue_shades[len(drawn_crate_paths) % 3], is_best)
            drawn_crate_paths.append(crate['path'])

        # 5. Draw Top N Release Paths (Shades of Green)
        green_shades = [QtGui.QColor(0, 255, 0, 200), QtGui.QColor(150, 255, 0, 200), QtGui.QColor(200, 255, 100, 200)]
        drawn_zone_paths = []
        for zone in valid_zones:
            if len(drawn_zone_paths) >= self.max_paths_to_show: 
                break
            
            if zone['path'] in drawn_zone_paths: 
                continue
                
            is_best = (len(drawn_zone_paths) == 0)
            draw_trajectory(zone['path'], green_shades[len(drawn_zone_paths) % 3], is_best)
            drawn_zone_paths.append(zone['path'])

    def _redraw_morbidity(self):
        """Draw the 'Danger Map' showing why paths are refused."""
        # 1. Clear old morbidity items
        for item in self.morbidity_items:
            if item in self.scene.items():
                self.scene.removeItem(item)
        self.morbidity_items = []

        if not self.morbidity_data:
            return

        # Helper to convert meters to QRectF/Scene coordinates
        def get_qrect(x, y, w, h):
            px, py = self.pos_to_scene(x, y + h) # Qt draws from top-left, we calculate from bottom-left
            pw, ph = self.size_to_scene(w, h)
            return QtCore.QRectF(px, py, pw, ph)

        # 2. Draw HARD ZONES (Solid Red - Never Cross)
        red_brush = QtGui.QBrush(QtGui.QColor(255, 0, 0, 60)) # 60/255 opacity
        for z in self.morbidity_data.get('hard_zones', []):
            rect_item = QtWidgets.QGraphicsRectItem(get_qrect(z['x'], z['y'], z['w'], z['h']))
            rect_item.setBrush(red_brush)
            rect_item.setPen(QtGui.QPen(QtCore.Qt.red, 1))
            rect_item.setZValue(0.05) # Floor level
            self.scene.addItem(rect_item)
            self.morbidity_items.append(rect_item)

        # 3. Draw SAFETY MARGINS (Orange - The Expanded Robot Radius)
        orange_brush = QtGui.QBrush(QtGui.QColor(255, 165, 0, 40))
        for z in self.morbidity_data.get('safety_zones', []):
            rect_item = QtWidgets.QGraphicsRectItem(get_qrect(z['x'], z['y'], z['w'], z['h']))
            rect_item.setBrush(orange_brush)
            rect_item.setPen(QtGui.QPen(QtCore.Qt.darkYellow, 1, QtCore.Qt.DashLine))
            rect_item.setZValue(0.06)
            self.scene.addItem(rect_item)
            self.morbidity_items.append(rect_item)

        # 4. Draw CRATE BUBBLES (Blue Circles - Dynamic Obstacles)
        blue_brush = QtGui.QBrush(QtGui.QColor(0, 0, 255, 35))
        for b in self.morbidity_data.get('crate_bubbles', []):
            # pos_to_scene returns center, drawEllipse needs top-left
            cx, cy = self.pos_to_scene(b['x'], b['y'])
            r_px, _ = self.size_to_scene(b['radius'], b['radius']) # radius in pixels
            
            ellipse_item = QtWidgets.QGraphicsEllipseItem(cx - r_px, cy - r_px, r_px * 2, r_px * 2)
            ellipse_item.setBrush(blue_brush)
            ellipse_item.setPen(QtGui.QPen(QtCore.Qt.blue, 1, QtCore.Qt.DotLine))
            ellipse_item.setZValue(0.07)
            self.scene.addItem(ellipse_item)
            self.morbidity_items.append(ellipse_item)

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

    def update_ghost_position(self, msg):
        """Update the map of the robots."""
        self.map_scene.update_ghost_position(msg)

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
    
    def update_ghost_position(self, msg):
        """Update the map of the robots."""
        self.map_scene.update_ghost_position(msg)


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

    def update_ghost_position(self, msg):
        if self.parent.use_ghost_lidar_points:
            self.component_pages["GlobalView"].update_ghost_position(msg)
            self.component_pages["GlobalView"].update_ghost_position(msg)

    def update_robot_position(self, msg):
        """Update robot position in every layout."""
        self.component_pages["GlobalView"].update_map_position(msg)
        self.component_pages["GlobalView"].update_map_position(msg)
        self.component_pages["GlobalView"].update_text_position(msg)
        self.component_pages["Motors"].update_map_position(msg)
        self.component_pages["Motors"].update_map_position(msg)
        self.component_pages["Motors"].update_text_position(msg)

    # Inside MainRobotPage class
    def update_board_state(self, json_str):
        self.component_pages["GlobalView"].map_scene.apply_update_state(json_str)
        
    def init_board_state(self, json_str):
        self.component_pages["GlobalView"].map_scene.apply_full_state(json_str)

    # --- ADD THIS NEW METHOD ANYWHERE INSIDE MainRobotPage ---
    def set_ghost_visibility(self, visible: bool):
        """Pass the visibility toggle down to all map scenes."""
        if "GlobalView" in self.component_pages:
            self.component_pages["GlobalView"].map_scene.set_ghost_visibility(visible)
        if "Motors" in self.component_pages:
            self.component_pages["Motors"].map_scene.set_ghost_visibility(visible)

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
        self.setWindowTitle("Orchestrator GUI")
        self.setGeometry(0, 0, 600, 800)

        # 1. Initialize variables
        self.use_only_lidar_points = False
        self.use_ghost_lidar_points = True  # Default to ON
        if self.use_only_lidar_points:
            self.use_ghost_lidar_points = False

        self.gui_node = NodeGUI(self)
        self.leash_button = QtWidgets.QPushButton("Send all LEASH")
        self.leash_button.clicked.connect(self.send_leashes)

        # 2. --- NEW: Create the Checkbox ---
        self.ghost_checkbox = QtWidgets.QCheckBox("Show Lidar Ghost (Debug)")
        self.ghost_checkbox.setChecked(self.use_ghost_lidar_points)
        self.ghost_checkbox.stateChanged.connect(self.toggle_ghost)

        # GUI for each robot
        self.page_name_box = QtWidgets.QComboBox()
        self.page_name_box.addItems([name for name in self.gui_node.robot_names] + ["General View"])
        self.robot_pages = {
            name: MainRobotPage(name, self) for name in self.gui_node.robot_names
        }
        self.robot_pages['General View'] = GeneralViewPage(self.gui_node.robot_names, self)
        self.stackedWidgets = QtWidgets.QStackedWidget()
        for page_name in [self.page_name_box.itemText(i) for i in range(self.page_name_box.count())]:
            self.stackedWidgets.addWidget(self.robot_pages[page_name])
        
        self.page_name_box.currentIndexChanged.connect(self.change_page)

        # 3. Add it to your layout
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        self.layout = QtWidgets.QVBoxLayout(central_widget)
        self.layout.addWidget(self.leash_button)
        self.layout.addWidget(self.ghost_checkbox) # <-- NEW: Add to screen
        self.layout.addWidget(self.page_name_box)
        self.layout.addWidget(self.stackedWidgets)

        self._connect_to_ros()
        self.call_update_global()
        self.fetch_initial_states()

    def toggle_ghost(self, state):
        """Triggered when the user clicks the checkbox."""
        is_visible = (state == QtCore.Qt.Checked)
        
        # 1. Update the variable so NodeGUI starts/stops passing Lidar data
        self.use_ghost_lidar_points = is_visible
        
        # 2. Force the graphics items to hide/show immediately
        for name, page in self.robot_pages.items():
            if isinstance(page, MainRobotPage):
                page.set_ghost_visibility(is_visible)

    def update_robot_position(self, msg, name):
        """Update the position of the robot."""
        self.robot_pages[name].update_robot_position(msg)

    def update_ghost_position(self, msg, name):
        """Update the position of the ghost."""
        self.robot_pages[name].update_ghost_position(msg)

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
