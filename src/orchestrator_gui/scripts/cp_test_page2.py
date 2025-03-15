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

class MyROSNode(Node):
    def __init__(self, update_callback):
        super().__init__('gui_node')
        self.get_logger().info("GUI Node created")
        self._init_parameters()
        self._init_subscriber()
        self._init_client_action()
        
        self.update_callback = update_callback

    def _init_parameters(self) -> None:
        self.declare_parameters(
            namespace="",
            parameters=[
                ("position_topic", rclpy.Parameter.Type.STRING),
                ("moveto_action", rclpy.Parameter.Type.STRING),
            ],
        )
        
        self.moveto_action = (
            self.get_parameter("moveto_action").get_parameter_value().string_value
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
        pass

    def publish_message(self, message_text: str):
        msg = String()
        msg.data = message_text
        self.publisher_.publish(msg)
        # self.get_logger().info(f"Message publié : {message_text}")

    def position_callback(self, msg):
        # self.get_logger().info(f"Message reçu: {msg.robot_position.x}")
        # Appeler la fonction de mise à jour de l'interface
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

class EzNode(Node):
    def __init__(self):
        super().__init__('nam_she')

class MapViewRosConnected(QtWidgets.QGraphicsView):
    def __init__(self, parent=None):
        super().__init__()
        self.parent_window = parent
        self.node = EzNode()
        image_path = os.path.join(get_package_share_directory("orchestrator_gui"), "images")
        map = os.path.join(image_path, "plateau.png")
        icon = os.path.join(image_path, "robot.png")
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
        icon_rect = self.icon_item.boundingRect()
        self.icon_width = icon_rect.width()
        self.icon_height = icon_rect.height()
        self.icon_item.setPos(5, 5)  # Position initiale
        self.scene.addItem(self.icon_item)
        
        # Timer pour déplacer l'icône toutes les 3 secondes
        # self.move_timer = QtCore.QTimer(self)
        # self.move_timer.timeout.connect(self.move_icon)
        # self.move_timer.start(3000)  # 3000 millisecondes = 3 secondes
        
        self.fitInView(self.scene.sceneRect(), QtCore.Qt.KeepAspectRatio)

    def update_robot_position(self, msg):
        map_rect = self.map_item.boundingRect()
        width = map_rect.width()
        height = map_rect.height()
        posx = int(width * msg.x / 2 - self.icon_width / 2) 
        posy = int(height * msg.y / 3 - self.icon_height / 2) 
        # Récupération de la position actuelle de l'icône
        current_pos = self.icon_item.pos()
        # Création d'une nouvelle position en ajoutant 10 pixels à la coordonnée x
        new_pos = QtCore.QPointF(posx, posy)
        # new_pos = QtCore.QPointF(current_pos.x() + 10, current_pos.y())
        self.icon_item.setPos(new_pos)
        print(f"Icône déplacée vers {new_pos}")
        current_rotation = self.icon_item.rotation()
        # new_rotation = current_rotation + 15
        new_rotation = msg.z * 180 / np.pi
        self.icon_item.setRotation(new_rotation)
        # print(f"Icône pivotée vers {new_rotation} degrés")

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

    # @QtCore.pyqtSlot(float)
    def update_label(self, text: float):
        # Mise à jour du label avec le texte reçu
        self.label.setText(str(text))

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
