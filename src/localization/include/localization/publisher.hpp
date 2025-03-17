#ifndef PUBLISHER_HPP_
#define PUBLISHER_HPP_

#include <vector>
#include <string>
#include <map>
#include <Eigen/Dense>
#include <optional>
#include <geometry_msgs/msg/point.hpp>
#include <cdf_msgs/msg/poses2_d.hpp>
#include <cdf_msgs/msg/lidar_loc.hpp>
#include <vision_msgs/msg/pose2_d.hpp>
#include <vision_msgs/msg/point2_d.hpp>

struct ChooseColor {
    float red, green, blue;
    ChooseColor(float r, float g, float b) : red(r), green(g), blue(b) {}
};

cdf_msgs::msg::LidarLoc publicate_donnees_lidar(const Eigen::Vector3d& robot_position, const std::vector<Eigen::Vector2d>& others);
cdf_msgs::msg::Poses2D publicate_donnees_zc(const std::vector<Eigen::Vector2d>& objects);
std::array<double, 4> get_quaternion_from_euler(double roll, double pitch, double yaw);

#endif // PUBLISHER_HPP_
