#include "localization/math_lidar.hpp"

Eigen::Vector2d unit_vector(const Eigen::Vector2d& v) {
    if (v.norm() == 0.0) {
        throw std::invalid_argument("Zero vector has no unit vector.");
    }
    return v / v.norm();
}

void removearray(std::vector<Eigen::Vector2d>& L, const Eigen::Vector2d& arr) {
    auto it = std::find_if(L.begin(), L.end(), [&](const Eigen::Vector2d& v) {
        return v.isApprox(arr);
    });
    if (it != L.end()) {
        L.erase(it);
    } else {
        throw std::runtime_error("Array not found in list.");
    }
}

std::pair<double, double> angle_between(const Eigen::Vector2d& vector1, const Eigen::Vector2d& vector2) {
    Eigen::Vector2d v1_u = unit_vector(vector1);
    Eigen::Vector2d v2_u = unit_vector(vector2);
    double angle = std::acos(std::clamp(v1_u.dot(v2_u), -1.0, 1.0));
    double minor = v1_u.x() * v2_u.y() - v1_u.y() * v2_u.x();
    return { (minor == 0 ? angle : std::copysign(angle, minor)), angle };
}

double dt2(const Eigen::Vector2d& ptA, const Eigen::Vector2d& ptB) {
    return (ptA - ptB).squaredNorm();
}

double dt(const Eigen::Vector2d& ptA, const Eigen::Vector2d& ptB) {
    return (ptA - ptB).norm();
}

bool approx(double distanceA, double distanceB, double precision) {
    return std::abs(distanceA - distanceB) < precision;
}

std::pair<double, double> cartesian_to_polar(double x, double y) {
    return { std::sqrt(x * x + y * y), std::abs(std::atan2(y, x)) };
}

bool angle_in_range(double alpha, double low, double up) {
    if (low > up) {
        throw std::invalid_argument("Low bound must be less than or equal to upper bound.");
    }
    alpha = std::fmod(alpha, 2 * M_PI);
    low = std::fmod(low, 2 * M_PI);
    up = std::fmod(up, 2 * M_PI);
    return (low <= alpha && alpha <= up) || (low > up && (low <= alpha || alpha <= up));
}

Eigen::Vector3d chgt_base_plateau_to_robot(const Eigen::Vector2d& pt, const Eigen::Vector3d& robot_frame) {
    double cos_angle = std::cos(robot_frame.z());
    double sin_angle = std::sin(robot_frame.z());

    Eigen::Matrix3d OtoR;
    OtoR << cos_angle, -sin_angle, robot_frame.x(),
            sin_angle, cos_angle, robot_frame.y(),
            0, 0, 1;

    return OtoR.inverse() * Eigen::Vector3d(pt.x(), pt.y(), 1);
}

Eigen::Vector3d chgt_base_robot_to_plateau(const Eigen::Vector2d& pt, const Eigen::Vector3d& robot_frame) {
    double cos_angle = std::cos(robot_frame.z());
    double sin_angle = std::sin(robot_frame.z());

    Eigen::Matrix3d OtoR;
    OtoR << cos_angle, -sin_angle, robot_frame.x(),
            sin_angle, cos_angle, robot_frame.y(),
            0, 0, 1;

    return OtoR * Eigen::Vector3d(pt.x(), pt.y(), 1);
}

bool getAngleSign(const std::array<Eigen::Vector2d, 3>& points) {
    double cross = (points[1].x() - points[0].x()) * (points[2].y() - points[1].y()) 
        - (points[1].y() - points[0].y()) * (points[2].x() - points[1].x());
    return (cross > 0);
}

bool getAngleSign(const std::array<std::optional<Eigen::Vector2d>, 3>& points) {
    double cross = (points[1]->x() - points[0]->x()) * (points[2]->y() - points[1]->y()) 
        - (points[1]->y() - points[0]->y()) * (points[2]->x() - points[1]->x());
    return (cross > 0);
}