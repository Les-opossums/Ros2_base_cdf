"""Tests unitaires de la fusion des tags (bibliotheque pure, sans ROS).

Lancement : pytest depuis le workspace source, ou `colcon test
--packages-select opossum_vision`."""

import math

from opossum_vision.tag_fusion import (
    Pose, PoseHistory, TagFuser, transform_to_world, aruco_color_name,
)


def test_transform_identity():
    # Robot a l'origine, oriente a 0 : le monde = le repere robot.
    p = Pose(0.0, 0.0, 0.0)
    wx, wy, wt = transform_to_world(p, 0.5, 0.2, 0.1)
    assert math.isclose(wx, 0.5, abs_tol=1e-9)
    assert math.isclose(wy, 0.2, abs_tol=1e-9)
    assert math.isclose(wt, 0.1, abs_tol=1e-9)


def test_transform_rotation():
    # Robot en (1,1), tourne de +90deg : un point devant (x=1) part vers +Y.
    p = Pose(1.0, 1.0, math.pi / 2)
    wx, wy, _ = transform_to_world(p, 1.0, 0.0, 0.0)
    assert math.isclose(wx, 1.0, abs_tol=1e-6)
    assert math.isclose(wy, 2.0, abs_tol=1e-6)


def test_pose_history_interpolation():
    h = PoseHistory()
    h.push(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    h.push(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    pose, extra = h.get_pose_at(0.5)
    assert math.isclose(pose.x, 0.5, abs_tol=1e-6)
    assert extra == 0.0  # entre deux echantillons -> interpolation


def test_pose_history_extrapolation_uses_velocity():
    h = PoseHistory()
    # push(t, x, y, theta, vlin, vdir, vt) : vitesse 1 m/s selon l'axe X (vdir=0)
    h.push(0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0)
    pose, extra = h.get_pose_at(0.5)  # au-dela du dernier echantillon
    assert math.isclose(pose.x, 0.5, abs_tol=1e-6)
    assert extra > 0.0


def test_fuser_two_tags_no_merge():
    f = TagFuser(min_hits=1, match_m=0.10)
    dets = [
        {"x": 0.5, "y": 0.5, "theta": 0.0, "color": "blue"},
        {"x": 1.5, "y": 0.5, "theta": 0.0, "color": "blue"},
    ]
    for _ in range(3):
        f.update([dict(d) for d in dets], now=0.0)
    snap = f.snapshot(now=0.0)
    assert len(snap) == 2  # deux estimes distincts, pas de fusion


def test_fuser_confirmation_min_hits():
    f = TagFuser(min_hits=3)
    d = {"x": 0.5, "y": 0.5, "theta": 0.0, "color": "blue"}
    f.update([dict(d)], now=0.0)
    assert f.snapshot(now=0.0) == []            # 1 hit -> pas encore confirme
    f.update([dict(d)], now=0.0)
    f.update([dict(d)], now=0.0)
    assert len(f.snapshot(now=0.0)) == 1        # >= min_hits -> confirme


def test_fuser_no_cross_color_merge():
    f = TagFuser(min_hits=1, match_m=0.30)
    # Deux couleurs au meme endroit : ne doivent PAS fusionner.
    for _ in range(3):
        f.update([{"x": 0.5, "y": 0.5, "theta": 0.0, "color": "blue"},
                  {"x": 0.51, "y": 0.5, "theta": 0.0, "color": "yellow"}],
                 now=0.0)
    snap = f.snapshot(now=0.0)
    assert len(snap) == 2


def test_dispersion_static_accumulates():
    f = TagFuser(min_hits=1, match_m=0.10, gate_m=0.10, alpha=0.5)
    import random
    random.seed(0)
    for _ in range(200):
        f.update([{"x": 0.5 + random.gauss(0, 0.002),
                   "y": 0.5 + random.gauss(0, 0.002),
                   "theta": 0.0, "color": "blue"}], now=0.0)
    snap = f.snapshot(now=0.0)
    assert len(snap) == 1
    assert snap[0]["n_static"] > 100
    assert snap[0]["std_static_m"] < 0.02  # dispersion faible (bruit ~2mm)


def test_aruco_color_name():
    assert aruco_color_name(47) == "yellow"
    assert aruco_color_name(36) == "blue"
    assert aruco_color_name(41) == "rot"
    assert aruco_color_name(99) == "tag"


# --------------------------------------------------------------------------- #
#  Confiance vs mouvement / persistance
# --------------------------------------------------------------------------- #
def test_confidence_high_when_static():
    f = TagFuser(min_hits=1)
    d = {"x": 0.5, "y": 0.5, "theta": 0.0, "color": "blue"}
    for _ in range(20):
        f.update([dict(d)], now=0.0, vlin=0.0, vt=0.0)   # robot à l'arrêt
    assert f.snapshot(now=0.0)[0]["confidence"] > 0.9


def test_confidence_drops_under_rotation():
    f = TagFuser(min_hits=1)
    d = {"x": 0.5, "y": 0.5, "theta": 0.0, "color": "blue"}
    # D'abord confiance haute à l'arrêt, puis rotation soutenue.
    for _ in range(20):
        f.update([dict(d)], now=0.0, vlin=0.0, vt=0.0)
    conf_static = f.snapshot(now=0.0)[0]["confidence"]
    for _ in range(20):
        f.update([dict(d)], now=0.0, vlin=0.0, vt=0.2)   # rotation
    conf_rot = f.snapshot(now=0.0)[0]["confidence"]
    assert conf_rot < conf_static
    # À vitesse numérique égale, la rotation pénalise plus que la translation.
    g = TagFuser(min_hits=1)
    for _ in range(20):
        g.update([dict(d)], now=0.0, vlin=0.0, vt=0.0)
    for _ in range(20):
        g.update([dict(d)], now=0.0, vlin=0.2, vt=0.0)   # translation seule
    conf_trans = g.snapshot(now=0.0)[0]["confidence"]
    assert conf_rot < conf_trans


def test_persistence_present_then_forgotten():
    f = TagFuser(min_hits=1, absent_s=2.0, forget_s=5.0)
    d = {"x": 0.5, "y": 0.5, "theta": 0.0, "color": "blue"}
    f.update([dict(d)], now=0.0, vlin=0.0, vt=0.0)
    # Vu à t=0 : présent.
    snap = f.snapshot(now=0.0)
    assert snap and snap[0]["present"] is True
    # t=3s sans le revoir : encore mémorisé mais non présent (persistance).
    snap = f.snapshot(now=3.0)
    assert snap and snap[0]["present"] is False
    assert snap[0]["confidence"] < 0.5     # confiance décrue avec l'âge
    # t=6s : oublié après forget_s.
    f.forget(now=6.0)
    assert f.snapshot(now=6.0) == []
