import os
import math
import signal
import subprocess
from ament_index_python.packages import get_package_share_directory

from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QPushButton, QLabel, QMessageBox, QComboBox,
                             QTextEdit, QStackedWidget, QDialog, QSizePolicy)
from PyQt5.QtCore import pyqtSignal, Qt, QTimer, QProcess
from PyQt5.QtGui import QFont, QMovie, QTextCursor

# --- Chemins sur le robot (workspace ROS 2 + depot git) ---
# Le workspace colcon est /home/opossum/robot_ws et le depot git (toutes les
# packages) est dans son sous-dossier src/. ROS 2 distro : humble.
WS_DIR = "/home/opossum/robot_ws"
REPO_DIR = WS_DIR + "/src"
ROS_SETUP = "/opt/ros/humble/setup.bash"

class GifPopup(QDialog):
    def __init__(self, gif_path, parent=None):
        super().__init__(parent)
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.Dialog | Qt.WindowStaysOnTopHint)
        self.setStyleSheet("background-color: black;") 
        layout = QVBoxLayout(self); layout.setContentsMargins(0, 0, 0, 0)
        self.label = QLabel(self); self.label.setAlignment(Qt.AlignCenter); layout.addWidget(self.label)
        self.movie = QMovie(gif_path); self.label.setMovie(self.movie); self.movie.start()
        QTimer.singleShot(3000, self.accept)

class HomePage(QWidget):
    """Page d'accueil : point d'entree de l'IHM.

    Permet d'aller vers la selection de match ou vers la page de mise a jour
    du code (git + colcon build), pour ne jamais avoir a passer par SSH.
    """
    request_match = pyqtSignal()
    request_update = pyqtSignal()

    def __init__(self):
        super().__init__()
        # Setup « connexion IHM web » : rosbridge + tag_fusion + calibration,
        # lances/coupes d'un seul bouton (couper allege le CPU en match).
        self.rosbridge_proc = None            # process du launch web_bridge
        self.rb_state = "off"                 # 'off' | 'starting' | 'on'

        l = QVBoxLayout(self); l.setSpacing(12); l.setContentsMargins(14, 14, 14, 14)
        title = QLabel("Opossum"); title.setFont(QFont("Arial", 20, QFont.Bold))
        title.setAlignment(Qt.AlignCenter); l.addWidget(title)
        l.addStretch(1)

        b_match = QPushButton("SÉLECTION MATCH"); b_match.setFont(QFont("Arial", 15, QFont.Bold))
        b_match.setStyleSheet("background-color: #27ae60; color: white; border: 2px solid black; border-radius: 10px;")
        b_match.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed); b_match.setMinimumHeight(90)
        b_match.clicked.connect(self.request_match.emit)
        l.addWidget(b_match)

        b_upd = QPushButton("MISE À JOUR DU CODE"); b_upd.setFont(QFont("Arial", 15, QFont.Bold))
        b_upd.setStyleSheet("background-color: #2980b9; color: white; border: 2px solid black; border-radius: 10px;")
        b_upd.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed); b_upd.setMinimumHeight(90)
        b_upd.clicked.connect(self.request_update.emit)
        l.addWidget(b_upd)

        # Bouton bridge (connexion IHM web) au meme niveau que les 2 ci-dessus.
        self.b_rb = QPushButton(); self.b_rb.setFont(QFont("Arial", 15, QFont.Bold))
        self.b_rb.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed); self.b_rb.setMinimumHeight(90)
        self.b_rb.clicked.connect(self.toggle_rosbridge)
        self._style_rosbridge_btn()
        l.addWidget(self.b_rb)

        # Redemarrage du service ROS (meme niveau que les autres boutons).
        b_ros = QPushButton("RESTART ROS"); b_ros.setFont(QFont("Arial", 15, QFont.Bold))
        b_ros.setStyleSheet("background-color: #c0392b; color: white; border: 2px solid black; border-radius: 10px;")
        b_ros.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed); b_ros.setMinimumHeight(90)
        b_ros.clicked.connect(self.restart_ros)
        l.addWidget(b_ros)
        l.addStretch(1)

    def restart_ros(self):
        # Redemarre le service ROS 2 (launch.service) sans passer par SSH.
        if QMessageBox.question(self, 'Confirmation', "Redémarrer le service ROS 2 ?",
                                QMessageBox.Yes | QMessageBox.No) != QMessageBox.Yes:
            return
        try:
            subprocess.run(['systemctl', '--user', 'restart', 'launch.service'], check=True)
        except Exception as e:
            QMessageBox.warning(self, "Restart ROS", f"Echec du redémarrage :\n{e}")

    # ---------- Setup « connexion IHM web » (rosbridge + support debug) ----------
    def _style_rosbridge_btn(self):
        # Couleur / libelle selon l'etat du setup web (off / starting / on).
        styles = {
            "off":      ("#7f8c8d", "CONNEXION WEB\nOFF"),
            "starting": ("#e67e22", "CONNEXION WEB\ndémarrage…"),
            "on":       ("#2ecc71", "CONNEXION WEB\nON"),
        }
        bg, text = styles.get(self.rb_state, styles["off"])
        self.b_rb.setText(text)
        self.b_rb.setStyleSheet(
            f"background-color: {bg}; color: white; border: 2px solid black; border-radius: 10px;"
        )

    def toggle_rosbridge(self):
        # Lance / coupe le setup complet « connexion IHM web » :
        #   rosbridge_server + tag_fusion_node + calibration_manager
        # (launch opossum_ihm/web_bridge.launch.py). Couper libere du CPU en match.
        if self.rb_state == "starting":
            return  # anti double-clic pendant le demarrage
        if self.rosbridge_proc is None:
            try:
                self.rosbridge_proc = subprocess.Popen(
                    ["ros2", "launch", "opossum_ihm", "web_bridge.launch.py"],
                    start_new_session=True,  # groupe de process propre -> kill fiable
                )
                self.rb_state = "starting"
                QTimer.singleShot(3500, self._confirm_rosbridge)
            except Exception as e:
                self.rosbridge_proc = None
                self.rb_state = "off"
                QMessageBox.warning(self, "Connexion Web", f"Echec du lancement :\n{e}")
        else:
            self.stop_rosbridge()
        self._style_rosbridge_btn()

    def _confirm_rosbridge(self):
        if self.rosbridge_proc is None:
            self.rb_state = "off"
        elif self.rosbridge_proc.poll() is None:
            self.rb_state = "on"
        else:
            self.rosbridge_proc = None
            self.rb_state = "off"
            QMessageBox.warning(
                self, "Connexion Web",
                "Le setup web s'est arrêté au démarrage.\n"
                "Vérifier rosbridge_server / tag_fusion / calibration.",
            )
        self._style_rosbridge_btn()

    def stop_rosbridge(self):
        if self.rosbridge_proc is None:
            self.rb_state = "off"
            return
        try:
            os.killpg(os.getpgid(self.rosbridge_proc.pid), signal.SIGINT)
            self.rosbridge_proc.wait(timeout=5)
        except Exception:
            try:
                os.killpg(os.getpgid(self.rosbridge_proc.pid), signal.SIGKILL)
            except Exception:
                pass
        self.rosbridge_proc = None
        self.rb_state = "off"
        self._style_rosbridge_btn()


def _make_back_button(callback):
    """Petit bouton 'Retour' homogene pour toutes les pages."""
    b = QPushButton("← Retour")
    b.setStyleSheet("background-color: #7f8c8d; color: white; font-weight: bold; "
                    "font-size: 13px; border-radius: 7px; padding: 5px 10px;")
    b.setFixedWidth(96); b.setFixedHeight(38)
    b.clicked.connect(callback)
    return b


class ConfigPage(QWidget):
    request_param_update = pyqtSignal(str, int)
    request_back = pyqtSignal()
    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self); layout.setSpacing(8); layout.setContentsMargins(12, 12, 12, 12)
        top = QHBoxLayout()
        top.addWidget(_make_back_button(self.request_back.emit))
        title = QLabel("Choix de la Stratégie"); title.setFont(QFont("Arial", 16, QFont.Bold)); title.setAlignment(Qt.AlignCenter)
        top.addWidget(title, 1)
        top.addSpacing(96)  # equilibre visuel avec le bouton retour
        layout.addLayout(top)
        
        font_btn = QFont("Arial", 13, QFont.Bold)

        # Liste des boutons à générer en "Grands"
        buttons_data = [
            ("MATCH JAUNE", "#FFD700", "black", "yellow", 11),
            ("MATCH BLEU", "#0000FF", "white", "blue", 12),
            ("HOMOLOGATION JAUNE", "#B8860B", "white", "yellow", 1),
            ("HOMOLOGATION BLEU", "#000080", "white", "blue", 2),
            # Script de test : le robot suit en continu la position monde du
            # premier tag ArUco vu par les cameras (compensee du retard
            # camera), y compris pendant qu'il se deplace -- sert a verifier
            # que la detection reste juste en mouvement, pas seulement a
            # l'arret. Voir opossum_action_sequencer/match/follow_ennemi.py
            # (node.follow_tag_aruco()).
            ("TEST CAMERA JAUNE", "#808080", "white", "yellow", 9),
            ("TEST CAMERA BLEU", "#404040", "white", "blue", 10),
        ]

        for text, bg, fg, color, script in buttons_data:
            btn = QPushButton(text)
            btn.setFont(font_btn)
            btn.setStyleSheet(f"background-color: {bg}; color: {fg}; border: 2px solid black; border-radius: 8px;")
            # Boutons compacts adaptes au petit ecran 4,3" (480x800 portrait)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            btn.setMinimumHeight(56)
            btn.clicked.connect(lambda checked, c=color, s=script, n=text: self.launch(c, s, n))
            layout.addWidget(btn)

    def launch(self, color, script, name):
        reply = QMessageBox.question(self, 'Confirmation', f"Valider : {name} ?", QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.request_param_update.emit(color, script)


class UpdatePage(QWidget):
    """Mise a jour du code du robot sans SSH.

    Flux : 'git fetch --all' -> l'utilisateur choisit une branche -> checkout
    de la branche + reset sur origin/<branche> + 'colcon build'. La sortie des
    commandes est streamee dans une console. Un bouton permet ensuite de
    redemarrer le service ROS pour appliquer le nouveau build.
    """
    request_back = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.proc = None
        self.init_ui()

    def init_ui(self):
        l = QVBoxLayout(self); l.setContentsMargins(10, 8, 10, 10); l.setSpacing(7)
        top = QHBoxLayout()
        top.addWidget(_make_back_button(self.request_back.emit))
        title = QLabel("Mise à jour du code"); title.setFont(QFont("Arial", 15, QFont.Bold))
        title.setAlignment(Qt.AlignCenter); top.addWidget(title, 1); top.addSpacing(96)
        l.addLayout(top)

        row = QHBoxLayout()
        self.b_fetch = QPushButton("git fetch")
        self.b_fetch.setStyleSheet("background-color: #2980b9; color: white; font-weight: bold; font-size: 13px; min-height: 44px; border-radius: 8px;")
        self.b_fetch.clicked.connect(self.do_fetch)
        row.addWidget(self.b_fetch)
        self.combo = QComboBox(); self.combo.setMinimumHeight(44)
        self.combo.setStyleSheet("font-size: 14px; padding: 3px;")
        row.addWidget(self.combo, 1)
        l.addLayout(row)

        self.b_build = QPushButton("Checkout + colcon build")
        self.b_build.setStyleSheet("background-color: #e67e22; color: white; font-weight: bold; font-size: 13px; min-height: 48px; border-radius: 8px;")
        self.b_build.setEnabled(False); self.b_build.clicked.connect(self.do_update)
        l.addWidget(self.b_build)

        self.console = QTextEdit(); self.console.setReadOnly(True)
        self.console.setStyleSheet("background-color: #111; color: #33d17a; font-family: monospace; font-size: 11px;")
        l.addWidget(self.console, 1)

        bottom = QHBoxLayout()
        self.status = QLabel("Prêt."); self.status.setFont(QFont("Arial", 11, QFont.Bold)); self.status.setWordWrap(True)
        bottom.addWidget(self.status, 1)
        self.b_restart = QPushButton("Redémarrer ROS")
        self.b_restart.setStyleSheet("background-color: #c0392b; color: white; font-weight: bold; font-size: 12px; min-height: 40px; border-radius: 8px;")
        self.b_restart.setEnabled(False); self.b_restart.clicked.connect(self.do_restart)
        bottom.addWidget(self.b_restart)
        l.addLayout(bottom)

    # ---------- Helpers ----------
    def _append(self, text):
        self.console.moveCursor(QTextCursor.End)
        self.console.insertPlainText(text)
        self.console.moveCursor(QTextCursor.End)

    def _busy(self, busy):
        self.b_fetch.setEnabled(not busy)
        self.b_build.setEnabled(not busy and self.combo.count() > 0)
        self.b_restart.setEnabled(not busy and self.b_restart.property("ready") is True)

    def _run(self, shell_cmd, on_done):
        """Lance une commande shell en asynchrone (QProcess) et streame la sortie."""
        self._append("\n$ " + shell_cmd + "\n")
        self.proc = QProcess(self)
        self.proc.setProcessChannelMode(QProcess.MergedChannels)
        self.proc.readyRead.connect(
            lambda: self._append(bytes(self.proc.readAll()).decode(errors="replace"))
        )
        self.proc.finished.connect(lambda code, _st: self._finished(code, on_done))
        self.proc.start("bash", ["-lc", shell_cmd])

    def _finished(self, code, on_done):
        self.proc = None
        on_done(code)

    # ---------- Etape 1 : fetch + liste des branches ----------
    def do_fetch(self):
        self._busy(True); self.status.setText("git fetch --all…")
        self._run(f"cd {REPO_DIR} && git fetch --all --prune", self._after_fetch)

    def _after_fetch(self, code):
        if code != 0:
            self.status.setText("❌ Échec du git fetch (voir console)."); self._busy(False); return
        # Liste des branches distantes (lecture rapide, bloquante < 1 s)
        p = QProcess(self)
        p.start("bash", ["-lc",
                f"cd {REPO_DIR} && git for-each-ref --format='%(refname:short)' refs/remotes/origin"])
        p.waitForFinished(5000)
        out = bytes(p.readAllStandardOutput()).decode(errors="replace")
        branches = []
        for line in out.splitlines():
            name = line.strip()
            if name.startswith("origin/"):
                name = name[len("origin/"):]
            if name and name != "HEAD" and name not in branches:
                branches.append(name)
        self.combo.clear(); self.combo.addItems(branches)
        self.status.setText(f"{len(branches)} branche(s) trouvée(s). Sélectionne puis lance le build.")
        self._busy(False)

    # ---------- Etape 2 : checkout + build ----------
    def do_update(self):
        branch = self.combo.currentText().strip()
        if not branch:
            return
        if QMessageBox.question(
            self, 'Confirmation',
            f"Basculer sur '{branch}', écraser les modifs locales du robot\net relancer colcon build ?",
            QMessageBox.Yes | QMessageBox.No) != QMessageBox.Yes:
            return
        self.b_restart.setProperty("ready", False)
        self._busy(True); self.status.setText(f"Checkout {branch} + colcon build… (peut durer plusieurs minutes)")
        cmd = (
            f"set -e; source {ROS_SETUP}; "
            f"cd {REPO_DIR}; git checkout {branch}; git reset --hard origin/{branch}; "
            f"cd {WS_DIR}; colcon build"
        )
        self._run(cmd, self._after_build)

    def _after_build(self, code):
        if code == 0:
            self.status.setText("✅ Build terminé. « Redémarrer ROS » pour appliquer.")
            self.b_restart.setProperty("ready", True)
        else:
            self.status.setText(f"❌ Échec du build (code {code}). Voir la console.")
            self.b_restart.setProperty("ready", False)
        self._busy(False)

    # ---------- Redemarrage du service ROS ----------
    def do_restart(self):
        if QMessageBox.question(self, 'Confirmation', "Redémarrer le service ROS 2 ?",
                                QMessageBox.Yes | QMessageBox.No) != QMessageBox.Yes:
            return
        self.status.setText("Redémarrage du service…")
        try:
            subprocess.run(['systemctl', '--user', 'restart', 'launch.service'], check=True)
            self.status.setText("✅ Service redémarré.")
        except Exception as e:
            self.status.setText(f"❌ Échec du redémarrage : {e}")


class MatchPage(QWidget):
    request_restart_match = pyqtSignal()
    request_home = pyqtSignal()
    def __init__(self):
        super().__init__()
        self.team_color = "lightgray"; self.is_au = False; self.comm_state = True; self.is_match = False
        self.position_mismatch = False; self.positions = {}; self.current_score = 0; self.match_time = 0
        # (Le bouton « Connexion Web » / bridge est desormais sur la page d'accueil.)

        self.buf_zynq = "ZYNQ - X: --.-- Y: --.-- T: --.--"
        self.buf_lidar = "LIDAR - X: --.-- Y: --.-- T: --.--"
        self.buf_cams = {1: "X: --.-- Y: --.--", 2: "X: --.-- Y: --.--", 3: "X: --.-- Y: --.--"}

        self.init_ui()

        # Rafraîchissement GUI bridé à 5 Hz (200ms) pour économiser le CPU
        self.gui_timer = QTimer(self); self.gui_timer.timeout.connect(self.refresh_gui_elements); self.gui_timer.start(200) 
        self.chrono_timer = QTimer(self); self.chrono_timer.timeout.connect(self.tick_chrono)

    def init_ui(self):
        l = QVBoxLayout(self); l.setContentsMargins(8, 6, 8, 8); l.setSpacing(4)
        top = QHBoxLayout()
        top.addWidget(_make_back_button(self.request_home.emit)); top.addStretch(1)
        l.addLayout(top)
        self.lbl_score = QLabel("0"); self.lbl_score.setFont(QFont("Arial", 64, QFont.Bold)); self.lbl_score.setAlignment(Qt.AlignCenter); l.addWidget(self.lbl_score, stretch=1)
        f_main = QFont("Arial", 13, QFont.Bold)
        self.lbl_zynq = QLabel(); self.lbl_zynq.setFont(f_main); self.lbl_zynq.setAlignment(Qt.AlignCenter); l.addWidget(self.lbl_zynq)
        self.lbl_lidar = QLabel(); self.lbl_lidar.setFont(f_main); self.lbl_lidar.setAlignment(Qt.AlignCenter); l.addWidget(self.lbl_lidar)
        f_cam = QFont("Arial", 11, QFont.Bold)
        self.cam_labels = {}
        for i in range(1, 4):
            lbl = QLabel(); lbl.setFont(f_cam); lbl.setAlignment(Qt.AlignCenter); l.addWidget(lbl); self.cam_labels[i] = lbl
        l.addSpacing(8)
        btn_l = QHBoxLayout(); btn_l.setSpacing(6)
        b_res = QPushButton("Restart\nMatch"); b_res.setStyleSheet("background-color: #3498db; color: white; font-weight: bold; font-size: 13px; min-height: 52px; border-radius: 8px;"); b_res.clicked.connect(self.trigger_restart_match)
        b_srv = QPushButton("Restart\nService"); b_srv.setStyleSheet("background-color: orange; color: black; font-weight: bold; font-size: 13px; min-height: 52px; border-radius: 8px;"); b_srv.clicked.connect(self.restart_service)
        btn_l.addWidget(b_res); btn_l.addWidget(b_srv); l.addLayout(btn_l)

    def refresh_gui_elements(self):
        if self.is_match:
            self.lbl_score.setText(f"{self.match_time} s\n")
        else:
            self.lbl_score.setText(str(self.current_score))
            
            # On formatte les strings UNIQUEMENT 5 fois par seconde
            if hasattr(self, 'ros_node_ref'):
                # Lecture des variables thread-safe
                lx, ly, lt = self.ros_node_ref.latest_lidar
                zx, zy, zt = self.ros_node_ref.latest_zynq
                
                self.lbl_zynq.setText(f"ZYNQ - X: {zx:.2f}  Y: {zy:.2f}  T: {zt:.2f}")
                self.lbl_lidar.setText(f"LIDAR - X: {lx:.2f}  Y: {ly:.2f}  T: {lt:.2f}")
                
                for i in range(1, 4):
                    cx, cy, ct = self.ros_node_ref.latest_cams.get(i, (0,0,0))
                    self.cam_labels[i].setText(f"CAM {i} - X: {cx:.2f} Y: {cy:.2f} T: {ct:.2f}")
                    
                # Mise à jour des positions pour le check mismatch
                self.positions['lidar'] = (lx, ly)
                self.positions['zynq'] = (zx, zy)
            
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
        self.page_home = HomePage()
        self.page_config = ConfigPage()
        self.page_match = MatchPage()
        self.page_update = UpdatePage()
        for p in (self.page_home, self.page_config, self.page_match, self.page_update):
            self.sw.addWidget(p)

        # --- Navigation entre pages ---
        self.page_home.request_match.connect(lambda: self.sw.setCurrentWidget(self.page_config))
        self.page_home.request_update.connect(lambda: self.sw.setCurrentWidget(self.page_update))
        self.page_config.request_back.connect(lambda: self.sw.setCurrentWidget(self.page_home))
        self.page_update.request_back.connect(lambda: self.sw.setCurrentWidget(self.page_home))
        self.page_match.request_home.connect(lambda: self.sw.setCurrentWidget(self.page_home))

        # Page d'accueil affichee au demarrage
        self.sw.setCurrentWidget(self.page_home)

    def go_to_match_page(self, color):
        self.page_match.team_color = color; self.page_match.update_background()
        self.sw.setCurrentWidget(self.page_match)

    def go_home(self):
        self.sw.setCurrentWidget(self.page_home)