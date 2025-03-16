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
    def __init__(self, update_callback):
        super().__init__('gui_node')
        self._init_parameters()
        self._init_subscriber()
        self._init_client_action()
        self.get_logger().info("Orchestrator GUI node initialized.")
        
        self.update_callback = update_callback

    def _init_parameters(self) -> None:
        self.declare_parameters(
            namespace="",
            parameters=[
                ("position_topic", rclpy.Parameter.Type.STRING),
                ("moveto_action", rclpy.Parameter.Type.STRING),
            ],
        )

    def _init_subscriber(self):
        self.position_topic = (
            self.get_parameter("position_topic").get_parameter_value().string_value
        )
        self.sub_position = self.create_subscription(
            LidarLoc,
            self.position_topic,
            self.position_callback,
            10
        )

    def _init_client_action(self):
        self.moveto_action = (
            self.get_parameter("moveto_action").get_parameter_value().string_value
        )
        self.moveto_client = ActionClient(self, MoveTo, self.moveto_action)

    def send_goal(self, x, y):
        goal_msg = MoveTo.Goal()
        goal_msg.goal = Point()
        goal_msg.goal.x = x
        goal_msg.goal.y = y
        goal_msg.goal.z = 0.

        self.moveto_client.wait_for_server()

        return self.moveto_client.send_goal_async(goal_msg)

    def send_future_position(self, x, y):
        future = self.send_goal(x, y)

        rclpy.spin_until_future_complete(self, future)

    def publish_message(self, message_text: str):
        msg = String()
        msg.data = message_text
        self.publisher_.publish(msg)

    def position_callback(self, msg):
        self.update_callback(msg.robot_position)

class MapView(QtWidgets.QGraphicsView):
    def __init__(self, parent=None):
        image_path = os.path.join(get_package_share_directory("orchestrator_gui"), "images")
        map = os.path.join(image_path, "plateau.png")
        icon = os.path.join(image_path, "robot.png")
        super().__init__(parent)
        # Création de la scène
        self.scene = QtWidgets.QGraphicsScene(self)
        self.setScene(self.scene)
        
        # Chargement de l'image de la carte
        self.map_pixmap = QtGui.QPixmap(map).scaled(1000, 1000, QtCore.Qt.KeepAspectRatio)
        self.map_item = QtWidgets.QGraphicsPixmapItem(self.map_pixmap)
        self.scene.addItem(self.map_item)
        
        # Chargement de l'icône et ajout d'un item déplaçable
        self.icon_pixmap = QtGui.QPixmap(icon).scaled(100, 100, QtCore.Qt.KeepAspectRatio)
        self.icon_item = DraggablePixmapItem(self.icon_pixmap)
        self.icon_item.setPos(5, 5)  # Position initiale
        self.scene.addItem(self.icon_item)
        
        # Timer pour déplacer l'icône toutes les 3 secondes
        self.move_timer = QtCore.QTimer(self)
        self.move_timer.timeout.connect(self.move_icon)
        self.move_timer.start(3000)  # 3000 millisecondes = 3 secondes
        
        self.fitInView(self.scene.sceneRect(), QtCore.Qt.KeepAspectRatio)

    def move_icon(self):
        # Récupération de la position actuelle de l'icône
        current_pos = self.icon_item.pos()
        # Création d'une nouvelle position en ajoutant 10 pixels à la coordonnée x
        new_pos = QtCore.QPointF(current_pos.x() + 10, current_pos.y())
        self.icon_item.setPos(new_pos)
        print(f"Icône déplacée vers {new_pos}")
        current_rotation = self.icon_item.rotation()
        new_rotation = current_rotation + 15
        self.icon_item.setRotation(new_rotation)
        print(f"Icône pivotée vers {new_rotation} degrés")

class MapViewRosConnected(QtWidgets.QGraphicsView):
    def __init__(self, parent=None):
        super().__init__()
        self.parent = parent
        image_path = os.path.join(get_package_share_directory("orchestrator_gui"), "images")
        map = os.path.join(image_path, "plateau.png")
        icon = os.path.join(image_path, "robot.png")
        # Création de la scène
        self.scene = QtWidgets.QGraphicsScene(self)
        self.setScene(self.scene)
        
        # Chargement de l'image de la carte
        self.map_pixmap = QtGui.QPixmap(map).scaled(800, 800, QtCore.Qt.KeepAspectRatio)
        self.map_item = QtWidgets.QGraphicsPixmapItem(self.map_pixmap)
        self.map_item.mousePressEvent = self.send_pos_goal
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

    def send_pos_goal(self, event):
        x, y = self.get_real_pos(event.pos().x(), event.pos().y())
        self.parent.send_future_position(x, y)

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
        p2 = MapViewRosConnected(self)
        self.ros_node = MyROSNode(p2.update_robot_position)

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
        self.comboBox.addItems(["Page 1", "Page 2", "Page 3"])
        self.comboBox.currentIndexChanged.connect(self.change_page)

        p1 = MyWidget('Heyooo')
        p3 = MapView()
        self.pages = [p1, p2, p3]

        self.label = QtWidgets.QLabel("En attente de message...", self)
        self.button = QtWidgets.QPushButton("Publier un message ROS")
        self.button.clicked.connect(self.on_button_clicked)

        self.layout.addWidget(self.button)
        self.layout.addWidget(self.label)
        self.layout.addWidget(self.comboBox)
        self.layout.addWidget(self.stackedWidgets)

        self.stackedWidgets.addWidget(p1)
        self.stackedWidgets.addWidget(p2)
        self.stackedWidgets.addWidget(p3)

        self._connect_to_ros()
        
    def send_future_position(self, x, y):
        self.ros_node.send_future_position(x, y)

    def _connect_to_ros(self):
        # QTimer pour intégrer le traitement ROS2 dans la boucle Qt
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(10)  # Vérifie ROS2 toutes les 10 ms

    def change_page(self, index):
        self.stackedWidgets.setCurrentIndex(index)
        
    def spin_ros(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0.001)

    def on_button_clicked(self):
        # Lors du clic, on publie un message sur le topic 'chatter'
        self.ros_node.publish_message("Hello ROS2!")

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



