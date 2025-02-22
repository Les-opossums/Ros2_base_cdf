from ament_index_python.packages import get_package_share_directory  # type: ignore
from PyQt5 import QtWidgets, QtGui, QtCore
import os

map = os.path.join(
            get_package_share_directory("orchestrator_gui"),
                                    "images",
                                    "plateau.png",
            )

icon = os.path.join(
            get_package_share_directory("orchestrator_gui"),
                                    "images",
                                    "robot.png",
            )


class MapView(QtWidgets.QGraphicsView):
    def __init__(self, parent=None):
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

class DraggablePixmapItem(QtWidgets.QGraphicsPixmapItem):
    def __init__(self, pixmap, parent=None):
        super().__init__(pixmap, parent)
        # Permettre à cet item d'être déplacé et sélectionné
        self.setFlags(QtWidgets.QGraphicsItem.ItemIsMovable |
                      QtWidgets.QGraphicsItem.ItemIsSelectable)
        self.setTransformOriginPoint(pixmap.width() / 2, pixmap.height() / 2)

if __name__ == '__main__':
    import sys
    app = QtWidgets.QApplication(sys.argv)
    view = MapView()
    view.setWindowTitle("Carte avec icône déplaçable")
    view.show()
    sys.exit(app.exec_())