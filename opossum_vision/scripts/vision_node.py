#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import threading
import time
import math
import struct
from collections import deque

# Assurez-vous que l'import fonctionne selon votre structure
try:
    from opossum_msgs.msg import VisionData, VisionDataFrame, CameraLoc
    from std_msgs.msg import String
except ImportError:
    class VisionData: pass
    class VisionDataFrame: pass
    class CameraLoc: pass
    class String: pass

class JevoisClockSync:
    """Estime l'offset entre l'horloge locale (arbitraire, boot du module) du
    JeVois et l'horloge du Raspberry Pi, a partir des trames
    "HEARTBEAT <CAM_ID> <capture_us>" envoyees periodiquement par la camera.

    Principe (NTP simplifie, sens unique) : a chaque HEARTBEAT recu,
    offset_estimate = t_local_reception - capture_us/1e6. La seule chose qui
    peut faire varier cette estimation d'un heartbeat a l'autre est un DELAI
    de transmission (traitement JeVois + liaison serie + ordonnancement sur
    le Pi), qui ne peut qu'AJOUTER du retard, jamais en retirer. Donc la
    vraie valeur de l'offset est la valeur MINIMALE observee sur une fenetre
    glissante -- les echantillons plus grands ne sont que du jitter de
    transmission qu'on ne veut pas propager dans le calcul du retard camera.
    """

    def __init__(self, window=25):
        self._samples = deque(maxlen=window)

    def update(self, capture_us, t_local_recv):
        offset = t_local_recv - (capture_us / 1.0e6)
        self._samples.append(offset)

    @property
    def ready(self):
        return len(self._samples) > 0

    def to_local(self, capture_us):
        """Convertit un timestamp JeVois (us, horloge locale camera) en
        timestamp Pi (s, meme horloge que time.time()). Retourne None tant
        qu'aucun HEARTBEAT n'a ete recu."""
        if not self._samples:
            return None
        return (capture_us / 1.0e6) + min(self._samples)


class SingleVisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        # 1. Déclaration des paramètres pour UNE SEULE caméra
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("camera_id", 1)
        self.declare_parameter("simulation", False)
        # true = trames BINAIRES (doit matcher binary_output du module JeVois).
        self.declare_parameter("binary_input", True)

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self.camera_id = self.get_parameter("camera_id").get_parameter_value().integer_value
        self.simulation = self.get_parameter("simulation").get_parameter_value().bool_value
        self.binary = self.get_parameter("binary_input").get_parameter_value().bool_value
        
        # 2. Initialisation des Publishers
        self.aruco_pub = self.create_publisher(VisionDataFrame, 'aruco_loc', 10)
        self.pub_command = self.create_publisher(String, "command", 10)
        self.camera_loc_pub = self.create_publisher(CameraLoc, 'camera_loc', 10)

        # Calibration d'horloge JeVois -> Pi, alimentee par les trames HEARTBEAT.
        # Necessaire pour convertir le CAPTURE_US envoye dans les trames ARUCO
        # (horloge locale/arbitraire du JeVois) en un temps comparable a
        # time.time() sur le Pi -- indispensable pour compenser le retard
        # camera cote action_sequencer_node (historique de pose robot).
        self._clock_sync = JevoisClockSync()

        # 3. Démarrage
        self.is_running = True
        self.serial_card = None
        
        if not self.simulation:
            self.serial_card = self._init_cam()
            if self.serial_card:
                self.read_thread = threading.Thread(target=self._serial_read_worker)
                self.read_thread.start()
        else:
            self.get_logger().info(f"Caméra {self.camera_id} lancée en mode SIMULATION.")

    def _init_cam(self):
        """Initialise la connexion série de manière sécurisée pour ROS 2."""
        while self.is_running:
            # self.get_logger().info(f"Tentative de connexion sur {self.port} (Baudrate: {self.baudrate})...")
            try:
                tested_serial = serial.Serial(
                    port=self.port, 
                    baudrate=self.baudrate, 
                    timeout=1.0
                )
                time.sleep(0.2) # Courte pause pour laisser le temps au handshake
                all_data = tested_serial.read_until(b'\n').decode("utf-8", errors="ignore")
                
                self.get_logger().info(f"Caméra {self.camera_id} connectée ! Réponse: {all_data.strip()}")
                return tested_serial
                
            except serial.SerialException as e:
                # self.get_logger().warn(f"Échec sur {self.port}. Nouvel essai dans 1s... ({e})")
                time.sleep(1.0)
                
        return None

    def _serial_read_worker(self):
        """Thread de lecture serie.

        Lit par GROS BLOCS (un seul appel systeme draine tout le buffer serie)
        au lieu du read_until octet-par-octet de pyserial (1 syscall/octet).
        Puis decoupe soit des trames binaires (framing magic+crc), soit des
        lignes ASCII, selon binary_input."""
        self.get_logger().info(
            f"Thread d'ecoute demarre sur {self.port} "
            f"({'BINAIRE' if self.binary else 'ASCII'}).")

        buf = bytearray()
        ser = self.serial_card
        while rclpy.ok() and self.is_running:
            try:
                n = ser.in_waiting
                chunk = ser.read(n if n > 0 else 1)  # bloque sur 1 octet si idle
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f"Port série déconnecté sur {self.port}: {e}")
                break
            if not chunk:
                continue
            buf.extend(chunk)
            if self.binary:
                self._consume_binary(buf)
            else:
                self._consume_lines(buf)

    def _consume_lines(self, buf: bytearray):
        """Extrait les lignes ASCII completes du buffer."""
        while True:
            i = buf.find(b'\n')
            if i < 0:
                return
            raw = bytes(buf[:i]); del buf[:i + 1]
            line = raw.decode('utf-8', errors="ignore").strip()
            if line:
                self._handle_received_line(line)

    # --- Protocole binaire : 0xA5 0x5A | type | len | payload | crc(xor) ---
    def _consume_binary(self, buf: bytearray):
        while True:
            i = buf.find(b'\xA5\x5A')
            if i < 0:
                # pas de magic : ne garder que le dernier octet (magic coupe entre 2 lectures)
                if len(buf) > 1:
                    del buf[:-1]
                return
            if i > 0:
                del buf[:i]  # jette le bruit (ex: terminateur de ligne) avant le magic
            if len(buf) < 5:
                return       # header incomplet, on attend
            typ = buf[2]; ln = buf[3]
            total = 4 + ln + 1
            if len(buf) < total:
                return       # payload incomplet, on attend
            payload = bytes(buf[4:4 + ln]); crc = buf[4 + ln]
            calc = typ ^ ln
            for c in payload:
                calc ^= c
            if (calc & 0xFF) == crc:
                self._handle_binary(typ, payload)
                del buf[:total]
            else:
                del buf[:2]  # CRC faux : on saute le magic et on resynchronise

    def _handle_binary(self, typ, payload):
        try:
            if typ == 0x02:      # HEARTBEAT : cam_id(u8) capture_us(i64)
                _cam, cap = struct.unpack_from('<Bq', payload, 0)
                self._clock_sync.update(cap, time.time())
            elif typ == 0x01:    # ARUCO : cam_id(u8) capture_us(i64) count(u8) tags[]
                cam, cap, count = struct.unpack_from('<BqB', payload, 0)
                off = 10  # 1 + 8 + 1
                frame = VisionDataFrame()
                frame.id = int(cam)
                lt = self._clock_sync.to_local(cap)
                frame.capture_time = lt if lt is not None else 0.0
                frame.object = []
                for _ in range(count):
                    tid, x, y, z, yaw = struct.unpack_from('<hffff', payload, off)
                    off += 18  # 2 + 4*4
                    vd = VisionData()
                    vd.id = int(tid)
                    vd.x = x / 1000.0
                    vd.y = y / 1000.0
                    vd.z = z / 1000.0
                    vd.theta = yaw * math.pi / 180.0 + math.pi / 2
                    frame.object.append(vd)
                self.aruco_pub.publish(frame)
            # typ 0x03 (ROBOTPOS) : ignore cote Pi, comme en ASCII.
        except struct.error:
            pass  # trame tronquee/corrompue -> ignoree (le CRC a normalement filtre)

    def _handle_received_line(self, data):
        if data and data[0].isalpha():
            self.process_data_rcv(data)

    def process_data_rcv(self, data):
        """Traite les données brutes et publie les messages ROS.

        Le dispatch initial se fait sur le premier mot via partition(), pas
        via un split() complet de la ligne : une trame ARUCO avec plusieurs
        tags detectes peut contenir des dizaines de tokens, et un split()
        complet ne servirait qu'a lire l'element 0 avant d'etre jete -- le
        parsing detaille de chaque branche se charge ensuite de decouper ce
        dont elle a reellement besoin. A 30-60Hz par camera (x3 cameras),
        ca evite un aller-retour d'allocation Python inutile par ligne.
        """
        if not data:
            return

        head = data.partition(' ')[0]

        if head == "ARUCO":
            self._handle_aruco(data)
        elif head == "HEARTBEAT":
            self._handle_heartbeat(data)
        elif head == "ERROR":
            self.get_logger().error(f"Erreur de la carte: {data}")

    def _handle_aruco(self, data):
        parts = data.split(',')
        if len(parts) < 1:
            return

        header_tokens = parts[0].split()
        if len(header_tokens) < 2:
            return

        vision_frame_msg = VisionDataFrame()
        try:
            # Utilise l'ID envoyé par la carte, sinon l'ID du paramètre
            vision_frame_msg.id = int(header_tokens[1])
        except ValueError:
            vision_frame_msg.id = self.camera_id

        # Format JeVois : "ARUCO <CAM_ID> <CAPTURE_US>,..." -- convertit
        # l'instant de capture (horloge locale JeVois) en temps Pi via la
        # calibration HEARTBEAT. 0.0 si pas encore calibre : le
        # consommateur (action_sequencer_node) doit alors retomber sur la
        # pose robot courante plutot que sur l'historique.
        vision_frame_msg.capture_time = 0.0
        if len(header_tokens) >= 3:
            try:
                capture_us = int(header_tokens[2])
                local_t = self._clock_sync.to_local(capture_us)
                if local_t is not None:
                    vision_frame_msg.capture_time = local_t
            except ValueError:
                pass

        vision_frame_msg.object = []

        for part in parts[1:]:
            tag_tokens = part.split()
            if tag_tokens:
                obj = self.create_vision_data(tag_tokens)
                if obj:
                    vision_frame_msg.object.append(obj)

        self.aruco_pub.publish(vision_frame_msg)

    # ROBOTPOS desactive cote firmware/Pi pour le moment (cf historique du
    # depot) -- laisse volontairement de cote ici, rien a reactiver.

    def _handle_heartbeat(self, data):
        # Format JeVois : "HEARTBEAT <CAM_ID> <CAPTURE_US>" -- sert
        # uniquement a caler l'horloge locale du JeVois sur celle du Pi.
        tokens = data.split()
        if len(tokens) >= 3:
            try:
                capture_us = int(tokens[2])
                self._clock_sync.update(capture_us, time.time())
            except ValueError:
                pass

    def create_vision_data(self, tokens):
        """Convertit les tokens bruts en un objet VisionData."""
        if len(tokens) < 5: 
            return None

        vd = VisionData()
        
        try:
            vd.id = int(tokens[0])
            vd.x = float(tokens[1]) / 1000.0
            vd.y = float(tokens[2]) / 1000.0
            vd.z = float(tokens[3]) / 1000.0
            vd.theta = float(tokens[4]) * 3.14159265 / 180.0 + 3.14156 / 2 

        except ValueError as e:
            self.get_logger().warn(f"Erreur de conversion tag: {e}")
            return None
            
        return vd

    def destroy_node(self):
        """Surcharge pour fermer proprement le thread et le port série."""
        self.get_logger().info("Arrêt demandé. Fermeture du matériel...")
        self.is_running = False
        if hasattr(self, 'read_thread') and self.read_thread.is_alive():
            self.read_thread.join(timeout=2.0)
        
        if self.serial_card and self.serial_card.is_open:
            self.serial_card.close()
            self.get_logger().info(f"Port {self.port} fermé proprement.")
            
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SingleVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()