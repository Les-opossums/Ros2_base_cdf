"""Compute all the needed math for the simulations."""

import numpy as np


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
