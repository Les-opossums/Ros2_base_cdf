#ifndef MATH_LIDAR_HPP_
#define MATH_LIDAR_HPP_

#include <vector>
#include <cmath>
#include <array>
#include <map>
#include <stdexcept>
#include <string>
#include <optional>
#include <Eigen/Dense>

Eigen::Vector2d unit_vector(const Eigen::Vector2d& v);
void removearray(std::vector<Eigen::Vector2d>& L, const Eigen::Vector2d& arr);
std::pair<double, double> angle_between(const Eigen::Vector2d& vector1, const Eigen::Vector2d& vector2);
double dt2(const Eigen::Vector2d& ptA, const Eigen::Vector2d& ptB = Eigen::Vector2d::Zero());
double dt(const Eigen::Vector2d& ptA, const Eigen::Vector2d& ptB = Eigen::Vector2d::Zero());
bool approx(double distanceA, double distanceB = 0, double precision = 0.1);
std::pair<double, double> cartesian_to_polar(double x, double y);
bool angle_in_range(double alpha, double low, double up);
Eigen::Vector3d chgt_base_plateau_to_robot(const Eigen::Vector2d& pt, const Eigen::Vector3d& robot_frame);
Eigen::Vector3d chgt_base_robot_to_plateau(const Eigen::Vector2d& pt, const Eigen::Vector3d& robot_frame);
double _calcul_erreur(const std::vector<Eigen::Vector2d>& beacons, const Eigen::Vector3d& position, const std::vector<Eigen::Vector2d>& fixed_beacons);
Eigen::Vector3d _find_position_opti(const std::vector<Eigen::Vector2d>& beacons, const std::vector<Eigen::Vector2d>& fixed_beacons);
Eigen::Vector3d _find_position_eq(const std::vector<Eigen::Vector2d>& beacons, const std::vector<Eigen::Vector2d>& fixed_beacons);
double find_angle(const Eigen::Vector3d& pos, const std::vector<Eigen::Vector2d>& beacons, const std::vector<Eigen::Vector2d>& fixed_beacons);
std::vector<std::map<std::string, Eigen::Vector3d>> find_position(const std::vector<std::vector<Eigen::Vector2d>>& beacons_list, const std::vector<Eigen::Vector2d>& fixed_beacons, int nombre, const std::vector<double>& boundaries);
bool getAngleSign(const std::array<Eigen::Vector2d, 3>& beacons);
bool getAngleSign(const std::array<std::optional<Eigen::Vector2d>, 3>& beacons);

#endif // MATH_LIDAR_HPP_
