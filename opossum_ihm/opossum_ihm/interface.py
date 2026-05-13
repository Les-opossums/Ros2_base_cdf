import os
import math
import subprocess
from ament_index_python.packages import get_package_share_directory

from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QPushButton, QLabel, QMessageBox, 
                             QStackedWidget, QDialog, QSizePolicy)
from PyQt5.QtCore import pyqtSignal, Qt, QTimer
from PyQt5.QtGui import QFont, QMovie

class GifPopup(QDialog):
    def __init__(self, gif_path, parent=None):
        super().__init__(parent)
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.Dialog | Qt.WindowStaysOnTopHint)
        self.setStyleSheet("background-color: black;") 
        layout = QVBoxLayout(self); layout.setContentsMargins(0, 0, 0, 0)
        self.label = QLabel(self); self.label.setAlignment(Qt.AlignCenter); layout.addWidget(self.label)
        self.movie = QMovie(gif_path); self.label.setMovie(self.movie); self.movie.start()
        QTimer.singleShot(3000, self.accept)

class ConfigPage(QWidget):
    request_param_update = pyqtSignal(str, int)
    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self); layout.setSpacing(15); layout.setContentsMargins(20, 20, 20, 20)
        title = QLabel("Choix de la Stratégie"); title.setFont(QFont("Arial", 22, QFont.Bold)); title.setAlignment(Qt.AlignCenter); layout.addWidget(title)
        
        font_btn = QFont("Arial", 18, QFont.Bold)

        # Liste des boutons à générer en "Grands"
        buttons_data = [
            ("MATCH JAUNE", "#FFD700", "black", "yellow", 11),
            ("MATCH BLEU", "#0000FF", "white", "blue", 12),
            ("HOMOLOGATION JAUNE", "#B8860B", "white", "yellow", 1),
            ("HOMOLOGATION BLEU", "#000080", "white", "blue", 2)
        ]

        for text, bg, fg, color, script in buttons_data:
            btn = QPushButton(text)
            btn.setFont(font_btn)
            btn.setStyleSheet(f"background-color: {bg}; color: {fg}; border: 3px solid black; border-radius: 10px;")
            # Cette ligne rend les boutons GRANDS (remplissent l'espace vertical)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            btn.clicked.connect(lambda checked, c=color, s=script, n=text: self.launch(c, s, n))
            layout.addWidget(btn)

    def launch(self, color, script, name):
        reply = QMessageBox.question(self, 'Confirmation', f"Valider : {name} ?", QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.request_param_update.emit(color, script)

class MatchPage(QWidget):
    request_restart_match = pyqtSignal()
    def __init__(self):
        super().__init__()
        self.team_color = "lightgray"; self.is_au = False; self.comm_state = True; self.is_match = False
        self.position_mismatch = False; self.positions = {}; self.current_score = 0; self.match_time = 0
        
        self.buf_zynq = "ZYNQ - X: --.-- Y: --.-- T: --.--"
        self.buf_lidar = "LIDAR - X: --.-- Y: --.-- T: --.--"
        self.buf_cams = {1: "X: --.-- Y: --.--", 2: "X: --.-- Y: --.--", 3: "X: --.-- Y: --.--"}

        self.init_ui()

        # Rafraîchissement GUI bridé à 5 Hz (200ms) pour économiser le CPU
        self.gui_timer = QTimer(self); self.gui_timer.timeout.connect(self.refresh_gui_elements); self.gui_timer.start(200) 
        self.chrono_timer = QTimer(self); self.chrono_timer.timeout.connect(self.tick_chrono)

    def init_ui(self):
        l = QVBoxLayout(self)
        self.lbl_score = QLabel("0"); self.lbl_score.setFont(QFont("Arial", 90, QFont.Bold)); self.lbl_score.setAlignment(Qt.AlignCenter); l.addWidget(self.lbl_score, stretch=1)
        f_main = QFont("Arial", 18, QFont.Bold)
        self.lbl_zynq = QLabel(); self.lbl_zynq.setFont(f_main); self.lbl_zynq.setAlignment(Qt.AlignCenter); l.addWidget(self.lbl_zynq)
        self.lbl_lidar = QLabel(); self.lbl_lidar.setFont(f_main); self.lbl_lidar.setAlignment(Qt.AlignCenter); l.addWidget(self.lbl_lidar)
        f_cam = QFont("Arial", 15, QFont.Bold)
        self.cam_labels = {}
        for i in range(1, 4):
            lbl = QLabel(); lbl.setFont(f_cam); lbl.setAlignment(Qt.AlignCenter); l.addWidget(lbl); self.cam_labels[i] = lbl
        l.addSpacing(20)
        btn_l = QHBoxLayout(); btn_l.setSpacing(10)
        b_res = QPushButton("Restart\nMatch"); b_res.setStyleSheet("background-color: #3498db; color: white; font-weight: bold; height: 80px; border-radius: 10px;"); b_res.clicked.connect(self.trigger_restart_match)
        b_srv = QPushButton("Restart\nService"); b_srv.setStyleSheet("background-color: orange; color: black; font-weight: bold; height: 80px; border-radius: 10px;"); b_srv.clicked.connect(self.restart_service)
        btn_l.addWidget(b_res); btn_l.addWidget(b_srv); l.addLayout(btn_l)

    def refresh_gui_elements(self):
        if self.is_match:
            self.lbl_score.setText(f"{self.match_time} s\n")
        else:
            self.lbl_score.setText(str(self.current_score))
            self.lbl_zynq.setText(self.buf_zynq); self.lbl_lidar.setText(self.buf_lidar)
            for i in range(1, 4): self.cam_labels[i].setText(f"CAM {i} - {self.buf_cams[i]}")
            self.check_positions()

    def tick_chrono(self): self.match_time += 1

    def set_match_state(self, command):
        if command == "LEASH":
            self.is_match = True; self.match_time = 0
            if not self.chrono_timer.isActive(): self.chrono_timer.start(1000) 
            self.update_background()
        elif command == "STOP": self.chrono_timer.stop()

    def update_score(self, s): self.current_score = s
    def update_lidar(self, x, y, t):
        if not self.is_match:
            self.buf_lidar = f"LIDAR - X: {x:.2f}  Y: {y:.2f}  T: {t:.2f}"
            self.positions['lidar'] = (x, y)

    def update_zynq(self, x, y, t):
        if not self.is_match:
            self.buf_zynq = f"ZYNQ - X: {x:.2f}  Y: {y:.2f}  T: {t:.2f}"
            self.positions['zynq'] = (x, y)

    def update_camera(self, i, x, y, t):
        if not self.is_match:
            self.buf_cams[i] = f"X: {x:.2f}  Y: {y:.2f}  T: {t:.2f}"
            self.positions[f'cam{i}'] = (x, y)

    def check_positions(self):
        v = list(self.positions.values())
        if len(v) < 2 or self.is_match: return
        mm = any(math.hypot(v[i][0]-v[j][0], v[i][1]-v[j][1]) > 0.10 for i in range(len(v)) for j in range(i+1, len(v)))
        if self.position_mismatch != mm: self.position_mismatch = mm; self.update_background()

    def update_background(self):
        if self.is_au: self.setStyleSheet("background-color: red;")
        elif not self.comm_state or (self.position_mismatch and not self.is_match): self.setStyleSheet("background-color: orange;")
        else: self.setStyleSheet(f"background-color: {self.team_color};")

    def set_au_state(self, au):
        if au and not self.is_au:
            gif = os.path.join(get_package_share_directory("opossum_ihm"), "images", "boulette.gif")
            if os.path.exists(gif): GifPopup(gif, self).show()
        self.is_au = au; self.update_background()

    def set_comm_state(self, s): self.comm_state = s; self.update_background()

    def trigger_restart_match(self):
        if QMessageBox.question(self, 'Confirmation', "Reset le Match ?", QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
            self.request_restart_match.emit(); self.chrono_timer.stop(); self.match_time = 0; self.is_match = False 

    def restart_service(self):
        if QMessageBox.question(self, 'Confirmation', "Restart ROS 2 ?", QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
            try: subprocess.run(['systemctl', '--user', 'restart', 'launch.service'], check=True)
            except: pass

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__(); self.resize(480, 800)
        self.sw = QStackedWidget(); self.setCentralWidget(self.sw)
        self.page_config = ConfigPage(); self.page_match = MatchPage()
        self.sw.addWidget(self.page_config); self.sw.addWidget(self.page_match)
    def go_to_match_page(self, color):
        self.page_match.team_color = color; self.page_match.update_background(); self.sw.setCurrentIndex(1)