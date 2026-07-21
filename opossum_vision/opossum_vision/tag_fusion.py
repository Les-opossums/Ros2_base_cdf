"""Fusion temporelle des tags ArUco en coordonnees MONDE.

Bibliotheque PURE (aucune dependance ROS) pour :
  - etre testable unitairement hors ligne,
  - etre partagee entre le noeud temps-reel (tag_fusion_node) et les outils
    de calibration (Opossum_vision_calibration).

Contenu :
  - PoseHistory   : historique pose/vitesse robot, avec interpolation entre
    echantillons et extrapolation par la vitesse -> permet de retrouver la
    pose du robot AU MOMENT DE LA CAPTURE camera (compensation du retard).
  - transform_to_world : detection repere robot -> repere monde.
  - TagFuser      : fusion multi-observations d'une caisse statique :
      * association optimale (Hungarian), une caisse <-> une detection,
      * jamais entre couleurs ArUco differentes,
      * lissage EMA de la pose,
      * confirmation anti-fantome (min_hits),
      * dispersion (ecart-type Welford) mesuree a l'arret ET en mouvement.

Cette fusion n'est PAS que du debug : elle fournit une estimation stable et
filtree des objets de jeu, utilisable aussi en match.
"""

import math
import bisect

import numpy as np
from scipy.optimize import linear_sum_assignment


# --------------------------------------------------------------------------- #
#  Utilitaires
# --------------------------------------------------------------------------- #
def aruco_color_name(aruco_id):
    """Mappe un id ArUco vers un nom de couleur d'objet de jeu."""
    return {47: "yellow", 36: "blue", 41: "rot"}.get(int(aruco_id), "tag")


def transform_to_world(pose, dx, dy, dtheta):
    """Transforme une detection (dx, dy, dtheta) exprimee dans le repere robot
    en coordonnees monde, connaissant la pose robot `pose` (x, y, t)."""
    c = math.cos(pose.t)
    s = math.sin(pose.t)
    return (
        pose.x + dx * c - dy * s,
        pose.y + dx * s + dy * c,
        pose.t + dtheta,
    )


def ema_angle(a0, a1, alpha):
    """Moyenne exponentielle d'angles (gere le repliement +/- pi)."""
    d = a1 - a0
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return a0 + alpha * d


def _welford_add(acc, x, y):
    """Accumulateur de variance en ligne (Welford) sur x et y."""
    acc["n"] += 1
    n = acc["n"]
    dx = x - acc["mx"]; acc["mx"] += dx / n; acc["M2x"] += dx * (x - acc["mx"])
    dy = y - acc["my"]; acc["my"] += dy / n; acc["M2y"] += dy * (y - acc["my"])


def _welford_std(acc):
    """Ecart-type de position (m) de l'accumulateur, 0 si < 2 echantillons."""
    if acc["n"] < 2:
        return 0.0
    return math.sqrt((acc["M2x"] + acc["M2y"]) / acc["n"])


def _new_acc():
    return {"n": 0, "mx": 0.0, "my": 0.0, "M2x": 0.0, "M2y": 0.0}


class Pose:
    __slots__ = ("x", "y", "t")

    def __init__(self, x, y, t):
        self.x = x
        self.y = y
        self.t = t


# --------------------------------------------------------------------------- #
#  Historique de pose robot
# --------------------------------------------------------------------------- #
class PoseHistory:
    """Historique borne des poses/vitesses robot. `get_pose_at(t)` retrouve la
    pose a un instant passe (interpolation) ou l'extrapole via la vitesse."""

    def __init__(self, max_age_s=2.0, max_samples=400):
        self.max_age_s = max_age_s
        self.max_samples = max_samples
        self._t = []
        self._x = []
        self._y = []
        self._th = []
        self._vlin = []
        self._vdir = []
        self._vt = []

    def clear(self):
        for lst in (self._t, self._x, self._y, self._th,
                    self._vlin, self._vdir, self._vt):
            lst.clear()

    def _pop_front(self):
        for lst in (self._t, self._x, self._y, self._th,
                    self._vlin, self._vdir, self._vt):
            lst.pop(0)

    def push(self, t, x, y, theta, vlin, vdir, vt):
        if self._t and t < self._t[-1]:
            self.clear()  # horloge qui recule -> on repart propre
        self._t.append(t); self._x.append(x); self._y.append(y)
        self._th.append(theta); self._vlin.append(vlin)
        self._vdir.append(vdir); self._vt.append(vt)
        cutoff = t - self.max_age_s
        while self._t and self._t[0] < cutoff:
            self._pop_front()
        while len(self._t) > self.max_samples:
            self._pop_front()

    @staticmethod
    def _extrapolate(x0, y0, th0, vlin0, vdir0, vt0, dt):
        return (x0 + vlin0 * math.cos(vdir0) * dt,
                y0 + vlin0 * math.sin(vdir0) * dt,
                th0 + vt0 * dt)

    def get_pose_at(self, t_query):
        """Retourne (Pose, extrapolated_by_s) ou (None, None) si vide."""
        n = len(self._t)
        if n == 0:
            return None, None
        if n == 1 or t_query <= self._t[0]:
            dt = t_query - self._t[0]
            x, y, th = self._extrapolate(self._x[0], self._y[0], self._th[0],
                                         self._vlin[0], self._vdir[0], self._vt[0], dt)
            return Pose(x, y, th), dt
        if t_query >= self._t[-1]:
            dt = t_query - self._t[-1]
            x, y, th = self._extrapolate(self._x[-1], self._y[-1], self._th[-1],
                                         self._vlin[-1], self._vdir[-1], self._vt[-1], dt)
            return Pose(x, y, th), dt
        i = bisect.bisect_left(self._t, t_query)
        t0, t1 = self._t[i - 1], self._t[i]
        frac = (t_query - t0) / (t1 - t0) if t1 > t0 else 0.0
        x = self._x[i - 1] + frac * (self._x[i] - self._x[i - 1])
        y = self._y[i - 1] + frac * (self._y[i] - self._y[i - 1])
        th = ema_angle(self._th[i - 1], self._th[i], frac)
        return Pose(x, y, th), 0.0


# --------------------------------------------------------------------------- #
#  Fusion des tags
# --------------------------------------------------------------------------- #
class TagFuser:
    """Fusion temporelle des detections monde en estimations stables par tag."""

    def __init__(self, alpha=0.35, gate_m=0.12, match_m=0.15, min_hits=3,
                 static_vlin=0.03, static_vt=0.05, forget_s=1.0):
        self.alpha = alpha
        self.gate_m = gate_m
        self.match_m = match_m
        self.min_hits = min_hits
        self.static_vlin = static_vlin
        self.static_vt = static_vt
        self.forget_s = forget_s
        self._estimates = {}
        self._next_key = 1

    def set_config(self, **kw):
        """Met a jour les parametres fournis (ignore les cles inconnues)."""
        for k, v in kw.items():
            if hasattr(self, k) and v is not None:
                setattr(self, k, v)

    def is_static(self, vlin, vt):
        return abs(vlin) < self.static_vlin and abs(vt) < self.static_vt

    def _new_estimate(self, d, now):
        k = self._next_key
        self._next_key += 1
        self._estimates[k] = {
            "x": d["x"], "y": d["y"], "theta": d["theta"],
            "color": d.get("color", "tag"), "last_seen": now, "hits": 1,
            "s": _new_acc(), "m": _new_acc(),
        }

    def update(self, dets, now, static):
        """Integre les detections monde d'UNE trame.

        dets : liste de dicts {x, y, theta, color}. Association optimale
        (Hungarian) aux estimes existants, une detection par estime, jamais
        entre couleurs differentes ; mise a jour EMA + dispersion Welford."""
        if not dets:
            return
        keys = list(self._estimates.keys())
        if not keys:
            for d in dets:
                self._new_estimate(d, now)
            return

        BIG = 1e3
        cost = np.full((len(keys), len(dets)), BIG)
        for i, k in enumerate(keys):
            e = self._estimates[k]
            for j, d in enumerate(dets):
                if d.get("color", "tag") != e["color"]:
                    continue  # inter-couleur interdit
                cost[i, j] = math.hypot(e["x"] - d["x"], e["y"] - d["y"])

        rows, cols = linear_sum_assignment(cost)
        matched = set()
        a = self.alpha
        for i, j in zip(rows, cols):
            if cost[i, j] > self.match_m:
                continue  # trop loin (ou couleur differente) -> pas d'appariement
            e = self._estimates[keys[i]]
            d = dets[j]
            # gating d'outlier : saut brutal -> on ignore l'obs (garde l'estime)
            if math.hypot(e["x"] - d["x"], e["y"] - d["y"]) > self.gate_m:
                matched.add(j)
                continue
            e["x"] += a * (d["x"] - e["x"])
            e["y"] += a * (d["y"] - e["y"])
            e["theta"] = ema_angle(e["theta"], d["theta"], a)
            e["last_seen"] = now
            e["hits"] += 1
            _welford_add(e["s"] if static else e["m"], d["x"], d["y"])
            matched.add(j)

        for j, d in enumerate(dets):
            if j not in matched:
                self._new_estimate(d, now)

    def forget(self, now):
        stale = [k for k, e in self._estimates.items()
                 if now - e["last_seen"] > self.forget_s]
        for k in stale:
            del self._estimates[k]

    def snapshot(self, confirmed_only=True):
        """Liste des estimations : dict {key, color, x, y, theta, hits,
        std_static_m, n_static, std_moving_m, n_moving}."""
        out = []
        for k, e in self._estimates.items():
            if confirmed_only and e["hits"] < self.min_hits:
                continue
            out.append({
                "key": k, "color": e["color"],
                "x": e["x"], "y": e["y"], "theta": e["theta"], "hits": e["hits"],
                "std_static_m": _welford_std(e["s"]), "n_static": e["s"]["n"],
                "std_moving_m": _welford_std(e["m"]), "n_moving": e["m"]["n"],
            })
        return out
