#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Moniteur systeme du Raspberry Pi.

Publie periodiquement (1 Hz par defaut) l'etat CPU / RAM / temperature du
Raspberry sur un topic JSON, pour la page « Systeme » de l'IHM web.

Topic (namespace du robot) : ``system_stats``  (std_msgs/String, payload JSON)

Payload JSON :
{
  "stamp": <epoch s>,
  "cpu_percent": 37.5,            # charge CPU globale (%)
  "cpu_per_core": [12.0, 55.0, ...],
  "cpu_count": 4,
  "mem_percent": 48.2,
  "mem_used_mb": 1930,
  "mem_total_mb": 4000,
  "temp_c": 51.3,                 # None si indisponible
  "load_avg": [0.8, 0.6, 0.5],    # moyennes de charge 1/5/15 min
  "uptime_s": 12345
}

Dependance : python3-psutil (``pip install psutil`` ou ``apt install python3-psutil``).
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import psutil
except ImportError:  # pragma: no cover - garde-fou si psutil manquant
    psutil = None


class SystemMonitor(Node):
    def __init__(self):
        super().__init__("system_monitor")
        self.declare_parameter("publish_period_s", 1.0)
        period = self.get_parameter("publish_period_s").get_parameter_value().double_value or 1.0

        self.pub = self.create_publisher(String, "system_stats", 10)
        self.timer = self.create_timer(period, self.publish_stats)

        if psutil is None:
            self.get_logger().error(
                "psutil introuvable : installe-le (pip install psutil). "
                "Le moniteur publiera des valeurs vides en attendant."
            )
        # Premier appel a cpu_percent pour amorcer la mesure (non bloquant ensuite)
        if psutil is not None:
            psutil.cpu_percent(interval=None)
            psutil.cpu_percent(interval=None, percpu=True)

    def _read_temp(self):
        """Temperature CPU en degres Celsius (None si indisponible)."""
        if psutil is not None:
            try:
                temps = psutil.sensors_temperatures()
                for key in ("cpu_thermal", "coretemp", "soc_thermal"):
                    if key in temps and temps[key]:
                        return round(temps[key][0].current, 1)
                # Sinon, prendre la premiere sonde dispo
                for entries in temps.values():
                    if entries:
                        return round(entries[0].current, 1)
            except Exception:
                pass
        # Fallback : lecture directe du thermal zone du Raspberry
        try:
            with open("/sys/class/thermal/thermal_zone0/temp") as f:
                return round(int(f.read().strip()) / 1000.0, 1)
        except Exception:
            return None

    def publish_stats(self):
        data = {"stamp": time.time()}
        if psutil is not None:
            vm = psutil.virtual_memory()
            data.update(
                cpu_percent=round(psutil.cpu_percent(interval=None), 1),
                cpu_per_core=[round(x, 1) for x in psutil.cpu_percent(interval=None, percpu=True)],
                cpu_count=psutil.cpu_count(),
                mem_percent=round(vm.percent, 1),
                mem_used_mb=round(vm.used / 1024 / 1024),
                mem_total_mb=round(vm.total / 1024 / 1024),
                temp_c=self._read_temp(),
            )
            try:
                import os
                la = os.getloadavg()
                data["load_avg"] = [round(x, 2) for x in la]
            except Exception:
                data["load_avg"] = None
            try:
                data["uptime_s"] = round(time.time() - psutil.boot_time())
            except Exception:
                data["uptime_s"] = None
        else:
            data.update(cpu_percent=None, cpu_per_core=[], cpu_count=None,
                        mem_percent=None, mem_used_mb=None, mem_total_mb=None,
                        temp_c=self._read_temp(), load_avg=None, uptime_s=None)

        msg = String()
        msg.data = json.dumps(data)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
