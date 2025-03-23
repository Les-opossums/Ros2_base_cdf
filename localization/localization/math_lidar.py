"""Compute all the necessary functions for mathematics."""

import numpy as np
import copy


def unit_vector(vector):
    """Convert to unit vector."""
    return vector / np.linalg.norm(vector)


def removearray(L, arr):
    """Remove array from the list."""
    ind = 0
    size = len(L)
    while ind != size and not np.array_equal(L[ind], arr):
        ind += 1
    if ind != size:
        L.pop(ind)
    else:
        raise ValueError("array not found in list.")


def sorttuple(*args):
    """Sort tuples by order."""
    liste = list(args)
    liste.sort()
    return tuple(liste)


def toint(char):
    """Convert char to int."""
    return ord(char) - 65


def tochar(int):
    """Convert int to char."""
    return chr(int + 65)


def angle_between(vector1, vector2):
    """Compute the angle between two vectors."""
    v1_u = unit_vector(vector1)
    v2_u = unit_vector(vector2)
    angle = np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
    minor = np.linalg.det(np.stack((v1_u, v2_u)))
    return (
        angle % (2 * np.pi) if minor == 0 else np.sign(minor) * angle % (2 * np.pi),
        angle % (2 * np.pi),
    )


def dt2(ptA, ptB=np.zeros(2)):
    """Compute the square distance (less expensive)."""
    return np.dot((ptA[:2] - ptB[:2]), (ptA[:2] - ptB[:2]))


def dt(ptA, ptB=np.zeros(2)):
    """Compute the distance."""
    return np.linalg.norm(ptA[:2] - ptB[:2])


def approx(distanceA: float, distanceB: float = 0, precision: float = 0.1) -> bool:
    """Check if two distances are close."""
    return abs(distanceA - distanceB) < precision


def cartesian_to_polar(x, y):
    """Convert to polar coordinates cartesian coordinate."""
    return np.sqrt(x**2 + y**2), abs(np.arctan2(y, x))


def angle_in_range(alpha, low, up):
    """Check if the angle is in the range."""
    if low > up:
        raise ValueError("Low bound must be less than or equal to up bound")
    alpha = np.mod(alpha, 2 * np.pi)
    low = np.mod(low, 2 * np.pi)
    up = np.mod(up, 2 * np.pi)
    return low <= alpha <= up if low < up else low <= alpha or alpha <= up


def convert_world_to_robot(pt, robot_frame):
    """Convert point from the world frame to robot one."""
    cos_angle, sin_angle = np.cos(robot_frame[2]), np.sin(robot_frame[2])
    OtoR = np.array(
        [
            [cos_angle, -sin_angle, robot_frame[0]],
            [sin_angle, cos_angle, robot_frame[1]],
            [0, 0, 1],
        ]
    )
    return np.linalg.inv(OtoR) @ np.array([pt[0], pt[1], 1])


def convert_robot_to_world(pt, robot_frame):
    """Convert point from the robot frame to world one."""
    cos_angle, sin_angle = np.cos(robot_frame[2]), np.sin(robot_frame[2])
    OtoR = np.array(
        [
            [cos_angle, -sin_angle, robot_frame[0]],
            [sin_angle, cos_angle, robot_frame[1]],
            [0, 0, 1],
        ]
    )
    return OtoR @ np.array([pt[0], pt[1], 1])


def _compute_error(beacons, position, fixed_beacons):
    """Compute the error between estimation and real position."""
    sum = 0
    for b, fb in zip(beacons, fixed_beacons):
        if b is not None:
            sum += dt2(convert_robot_to_world(b, position), fb)
    return sum


def _find_position_opti(beacons, fixed_beacons) -> None:
    """Compute the best robot position from beacon combination using optimization matrices."""
    BX = [fixed_beacons[i][0] for i in range(4) if beacons[i] is not None]
    BY = [fixed_beacons[i][1] for i in range(4) if beacons[i] is not None]
    AX = [np.array([1, 0, b[0], -b[1]]) for b in beacons if b is not None]
    AY = [np.array([0, 1, b[1], b[0]]) for b in beacons if b is not None]
    B = np.transpose(np.array([BX + BY]))
    A = np.array(AX + AY)
    try:
        X = np.linalg.solve(np.dot(np.transpose(A), A), np.dot(np.transpose(A), B))
        if abs(X[2]) < 1:
            if X[3] > 0:
                theta1 = np.arccos(X[2])
            else:
                theta1 = -np.arccos(X[2])
        elif abs(X[3] < 1):
            if X[2] < 0:
                theta1 = np.pi - np.arcsin(X[3])
            else:
                theta1 = np.arcsin(X[3])
        else:
            if X[2] < -1:
                theta1 = np.pi - np.arctan(X[3] / X[2])
            else:
                theta1 = np.arctan(X[3] / X[2])
        theta1 = np.mod(theta1, 2 * np.pi)
        return np.array([float(X[0]), float(X[1]), float(theta1)])
    except Exception:  # noqa: E722
        return -1 * np.ones(3)


def _find_position_eq(beacons, fixed_beacons) -> None:
    """Compute the best robot position from beacon combination using equations."""
    vfb = [fixed_beacons[i][:2] for i in range(4) if beacons[i] is not None][:3]
    dvb = [dt2(b) for b in beacons if b is not None][:3]
    dvfb = [dt2(vfb) for vfb in vfb]
    delta = [dvb[i] - dvfb[i] for i in range(3)]
    if abs(vfb[0][0] - vfb[1][0]) > 0.1:
        coeff = (vfb[1][1] - vfb[0][1]) / (vfb[0][0] - vfb[1][0])
        const = (delta[1] - delta[0]) / (2 * (vfb[0][0] - vfb[1][0]))
        y = (delta[2] - delta[0] - 2 * const * (vfb[0][0] - vfb[2][0])) / (
            2 * (coeff * (vfb[0][0] - vfb[2][0]) + (vfb[0][1] - vfb[2][1]))
        )
    elif abs(vfb[0][0] - vfb[2][0]) > 0.1:
        coeff = (vfb[2][1] - vfb[0][1]) / (vfb[0][0] - vfb[2][0])
        const = (delta[2] - delta[0]) / (2 * (vfb[0][0] - vfb[2][0]))
        y = (delta[1] - delta[0] - 2 * const * (vfb[0][0] - vfb[1][0])) / (
            2 * (coeff * (vfb[0][0] - vfb[1][0]) + (vfb[0][1] - vfb[1][1]))
        )
    x = const + y * coeff
    theta = find_angle(np.array([x, y]), beacons, fixed_beacons)
    return np.array([x, y, theta])


def find_angle(pos, beacons, fixed_beacons):
    """Find the angle of the robot using beacons position."""
    angle_tot = []
    min_lists = []
    for i in range(4):
        if beacons[i] is not None:
            v1 = beacons[i][:2]
            v2 = fixed_beacons[i][:2] - pos[:2]
            angle_temp_1, min_test = angle_between(v1, v2)
            angle_tot.append(angle_temp_1)
            min_lists.append(min_test)
    index = 0
    min_test = min_lists[0]
    for i in range(1, len(min_lists)):
        if min_lists[i] < min_test:
            index = i
            min_test = min_lists[i]
    return angle_tot[index]


def find_position(beacons_list, fixed_beacons, nombre, boundaries):
    """Find the position of the robot using beacons position."""
    rdatas = []
    for beacons in beacons_list:
        err = []
        position_found = _find_position_opti(beacons, fixed_beacons)
        dt_min = _compute_error(beacons, position_found, fixed_beacons)
        err.append(dt_min)
        position = copy.deepcopy(position_found)
        position[2] = find_angle(position, beacons, fixed_beacons)
        dt = _compute_error(beacons, position, fixed_beacons)
        if dt < dt_min:
            position_found[2] = position[2]
            dt_min = dt
        err.append(dt)
        if nombre > 2:
            position = _find_position_eq(beacons, fixed_beacons)
            dt = _compute_error(beacons, position, fixed_beacons)
            err.append(dt)
            if dt < dt_min:
                position_found = copy.deepcopy(position)
                dt_min = dt
        if (
            position_found[0] > boundaries[0] - 0.1  # Left
            and position_found[0] < boundaries[1] + 0.1  # Right
            and position_found[1] > boundaries[2] - 0.1  # Bottom
            and position_found[1] < boundaries[3] + 0.1  # Top
        ):
            rdatas.append(
                {
                    "position": position_found,
                    "beacons": beacons,
                    "err": err,
                }
            )
    return rdatas


def get_angle_sign(beacons):
    """Get the sign of the angle, using vectorial product."""
    return np.sign(np.cross(beacons[1] - beacons[0], beacons[2] - beacons[0]))


def compute_circle_intersection(ray_dir, center, radius):
    """Compute circle intersections."""
    # Calculate quadratic coefficients
    center
    a = np.dot(ray_dir, ray_dir)
    b = 2 * np.dot(ray_dir, center)
    c = np.dot(center, center) - radius**2
    discriminant = b**2 - 4 * a * c
    if discriminant < 0:
        return None  # No intersection
    # Compute both solutions
    sqrt_disc = np.sqrt(discriminant)
    t1 = (-b - sqrt_disc) / (2 * a)
    t2 = (-b + sqrt_disc) / (2 * a)
    # Choose the smallest positive t
    ts = [t for t in [t1, t2] if t > 0]
    if not ts:
        return None
    return min(ts)


def lidar_scan(angles, objects, max_range=10.0):
    """Simulate lidar point cloud."""
    scan_results = []
    for angle in angles:
        ray_dir = np.array([np.cos(angle), np.sin(angle)])
        min_distance = max_range
        # Check intersections with each object
        for obj in objects:
            if obj["type"] == "circle":
                t = compute_circle_intersection(
                    ray_dir, np.array(obj["center"]), obj["radius"]
                )
                if t is not None and t < min_distance:
                    min_distance = t
            # elif: similar block for rectangle intersections
        scan_results.append(min_distance)
        scan_results = [p if p < max_range else 0.0 for p in scan_results]
    return scan_results

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = np.cos(ai)
    si = np.sin(ai)
    cj = np.cos(aj)
    sj = np.sin(aj)
    ck = np.cos(ak)
    sk = np.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q