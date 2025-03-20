import sys
import random
from PyQt5 import QtCore, QtWidgets, QtGui
from ament_index_python.packages import get_package_share_directory  # type: ignore
import os


BSIZE_X = 3
BSIZE_Y = 2

class MyPoorWidget(QtWidgets.QWidget):
    def __init__(self, text='Hey'):
        super().__init__()
        self.hello = text
        self.layout2 = QtWidgets.QVBoxLayout(self)
        self.button = QtWidgets.QPushButton("Click me!")
        self.text = QtWidgets.QLabel(self.hello,
                                     alignment=QtCore.Qt.AlignCenter)
        self.layout2.addWidget(self.text)
        self.layout2.addWidget(self.button)
        

class MyWidget(QtWidgets.QWidget):
    def __init__(self, text='Hey'):
        super().__init__()
        self.x, self.y, self.theta = None, None, None
        self.hello = text

        self.button = QtWidgets.QPushButton("Click me!")
        self.text = QtWidgets.QLabel(self.hello,
                                     alignment=QtCore.Qt.AlignCenter)
        self.pos0 = QtWidgets.QLabel("--:--:--",
                                     alignment=QtCore.Qt.AlignCenter)
        config = os.path.join(
            get_package_share_directory("orchestrator_gui"),
                                    "images",
                                    "plateau.png",
            )
        self.image = QtWidgets.QLabel()
        self.image.setPixmap(QtGui.QPixmap(config).scaled(640, 480, QtCore.Qt.KeepAspectRatio))
        self.img_width = self.image.width()
        self.img_height = self.image.height()
        self.image.setObjectName("image")
        self.image.mousePressEvent = self.getPos
        self.x_edit = QtWidgets.QLineEdit(parent=self)
        self.y_edit = QtWidgets.QLineEdit(parent=self)
        self.theta_edit = QtWidgets.QLineEdit(parent=self)
         
        self.layout2 = QtWidgets.QVBoxLayout(self)
        self.layout2.addWidget(self.text)
        self.layout2.addWidget(self.button)
        self.layout2.addWidget(self.pos0)
        self.layout2.addWidget(self.image)
        self.layout2.addWidget(self.x_edit)
        self.layout2.addWidget(self.y_edit)
        self.layout2.addWidget(self.theta_edit)
        self.button.clicked.connect(self.magic)
        self.x_edit.returnPressed.connect(self.update_label_x)
        self.y_edit.returnPressed.connect(self.update_label_y)
        self.theta_edit.returnPressed.connect(self.update_label_theta)

    # @QtCore.Slot()
    def magic(self):
        self.text.setText(random.choice(self.hello))

    def update_label_x(self):
        self.x = self.x_edit.text()

    def update_label_y(self):
        self.y = self.y_edit.text()

    def update_label_theta(self):
        self.theta = self.theta_edit.text()
        
    def getPos(self, event):
        self.x, self.y = self.get_real_pos(event.pos().x(), event.pos().y())
        self.theta = self.theta if self.theta is not None else 0
        self.pos0.setText(f"x = {self.x}, y = {self.y}, theta = {self.theta}")
        self.x_edit.setText(f"{self.x}")
        self.y_edit.setText(f"{self.y}")
        self.theta_edit.setText(f"{self.theta}")
    
    def get_real_pos(self, x, y):
        return BSIZE_X * x / self.img_width, BSIZE_Y * (1 - y / self.img_height)

class MyStackWidgetMain(QtWidgets.QWidget):
    def __init__(self):
        
        super().__init__()
        self.layout2 = QtWidgets.QVBoxLayout(self)
        self.comboBox = QtWidgets.QComboBox()
        self.pageComboBox = QtWidgets.QComboBox()
        self.comboBox.addItems(["Page 1", "Page 2", "Page 3"])
        self.layout2.addWidget(self.comboBox)
        self.setWindowTitle("BLABLABLA")

        self.stackedWidgets = QtWidgets.QStackedWidget()
        self.layout2.addWidget(self.stackedWidgets)
        p1 = MyWidget('Heyooo')
        # p2 = MyWidget('Hello')
        # p3 = MyWidget('Hi')
        self.stackedWidgets.addWidget(p1)
        # self.stackedWidgets.addWidget(p2)
        # self.stackedWidgets.addWidget(p3)
        # self.pages = [p1, p2, p3]

        self.comboBox.currentIndexChanged.connect(self.change_page)

    def change_page(self, index):
        self.stackedWidgets.setCurrentIndex(index)

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    widget = MyStackWidgetMain()
    widget.show()
    sys.exit(app.exec_())