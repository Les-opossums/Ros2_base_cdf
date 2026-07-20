#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import bisect
import math


class Version:
    def __init__(self, version):
        self.version = version


class Position:
    def __init__(self, x=None, y=None, t=None):
        self.x = x
        self.y = y
        self.t = t


class Speed:
    def __init__(self, vx, vy, vt):
        self.vx = vx
        self.vy = vy
        self.vt = vt


class RobotPoseHistory:
    """Historique borne des poses/vitesses du robot (alimente depuis robot_data).

    But: retrouver la pose du robot au moment ou une frame camera a ete
    CAPTUREE (et non la pose courante au moment ou le message ROS est
    traite), afin de compenser le retard camera+liaison serie quand le
    robot est en mouvement. Sans ca, transformer une detection robot-frame
    en coordonnees monde avec la pose "maintenant" introduit une erreur qui
    grandit avec la vitesse du robot et le retard de la camera.

    vlin/vdir sont supposes exprimes dans le referentiel monde (vitesse
    lineaire du robot = vlin a l'angle vdir), coherent avec x/y/theta qui
    sont deja des poses absolues envoyees par le Zynq.
    """

    def __init__(self, max_age_s=2.0, max_samples=400):
        self.max_age_s = max_age_s
        self.max_samples = max_samples
        self._t = []
        self._x = []
        self._y = []
        self._theta = []
        self._vlin = []
        self._vdir = []
        self._vt = []

    def push(self, t, x, y, theta, vlin, vdir, vt):
        """Ajoute un echantillon. `t` doit etre croissant (meme horloge que
        celle utilisee ensuite dans get_pose_at, typiquement time.time())."""
        if self._t and t < self._t[-1]:
            # Horloge qui recule (NTP, redemarrage...) : on repart propre
            # plutot que de casser l'hypothese de liste triee.
            self.clear()

        self._t.append(t)
        self._x.append(x)
        self._y.append(y)
        self._theta.append(theta)
        self._vlin.append(vlin)
        self._vdir.append(vdir)
        self._vt.append(vt)

        cutoff = t - self.max_age_s
        while self._t and self._t[0] < cutoff:
            self._pop_front()
        while len(self._t) > self.max_samples:
            self._pop_front()

    def _pop_front(self):
        for lst in (self._t, self._x, self._y, self._theta, self._vlin, self._vdir, self._vt):
            lst.pop(0)

    def clear(self):
        for lst in (self._t, self._x, self._y, self._theta, self._vlin, self._vdir, self._vt):
            lst.clear()

    @staticmethod
    def _extrapolate(x0, y0, theta0, vlin0, vdir0, vt0, dt):
        x = x0 + vlin0 * math.cos(vdir0) * dt
        y = y0 + vlin0 * math.sin(vdir0) * dt
        theta = theta0 + vt0 * dt
        return x, y, theta

    @staticmethod
    def _lerp_angle(a0, a1, frac):
        d = a1 - a0
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return a0 + d * frac

    def get_pose_at(self, t_query):
        """Retourne (Position(x, y, t), extrapolated_by_s) pour l'instant
        t_query, ou None si l'historique est vide.

        extrapolated_by_s vaut 0.0 si t_query tombe entre deux echantillons
        reellement mesures (interpolation, cas normal). Sinon il vaut le
        nombre de secondes en dehors de l'historique disponible (positif si
        t_query est plus recent que le dernier echantillon, negatif si plus
        ancien que le premier) -- utile pour logger/detecter un probleme de
        synchronisation d'horloge cote camera.
        """
        n = len(self._t)
        if n == 0:
            return None, None

        if n == 1 or t_query <= self._t[0]:
            dt = t_query - self._t[0]
            x, y, theta = self._extrapolate(
                self._x[0], self._y[0], self._theta[0],
                self._vlin[0], self._vdir[0], self._vt[0], dt,
            )
            return Position(x=x, y=y, t=theta), dt

        if t_query >= self._t[-1]:
            dt = t_query - self._t[-1]
            x, y, theta = self._extrapolate(
                self._x[-1], self._y[-1], self._theta[-1],
                self._vlin[-1], self._vdir[-1], self._vt[-1], dt,
            )
            return Position(x=x, y=y, t=theta), dt

        # Interpolation entre les deux echantillons mesures qui encadrent t_query
        i = bisect.bisect_left(self._t, t_query)
        t0, t1 = self._t[i - 1], self._t[i]
        frac = (t_query - t0) / (t1 - t0) if t1 > t0 else 0.0
        x = self._x[i - 1] + frac * (self._x[i] - self._x[i - 1])
        y = self._y[i - 1] + frac * (self._y[i] - self._y[i - 1])
        theta = self._lerp_angle(self._theta[i - 1], self._theta[i], frac)
        return Position(x=x, y=y, t=theta), 0.0


class PUMP_struct:
    def __init__(self, pump_id, enable):
        self.pump_id = pump_id
        self.enable = enable

class VACCUMGRIPPER_struct:
    def __init__(self, id, mode, side):
        self.id = id
        self.mode = mode # 0: free, 1: picking, 2: dropping, 3: revert_dropping
        self.side = side # 0 left, 1 right, 2 both

class LED_struct:
    def __init__(self, red, green, blue):
        self.red = red
        self.green = green
        self.blue = blue


class ODOM_struct:
    def __init__(self, enable, freq):
        self.enable = enable
        self.freq = freq


class SERVO_struct:
    def __init__(self, servo_id, angle):
        self.servo_id = servo_id
        self.angle = angle


class STEPPER_struct:
    def __init__(self, mode):
        self.mode = mode


class VALVE_struct:
    def __init__(self, valve_id):
        self.valve_id = valve_id
