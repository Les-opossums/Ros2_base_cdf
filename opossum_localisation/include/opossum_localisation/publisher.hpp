#ifndef PUBLISHER_HPP_
#define PUBLISHER_HPP_

#include <vector>
#include <string>
#include <map>
#include <Eigen/Dense>
#include <optional>
#include <geometry_msgs/msg/point.hpp>
#include <opossum_msgs/msg/poses2_d.hpp>
#include <opossum_msgs/msg/lidar_loc.hpp>
#include <vision_msgs/msg/pose2_d.hpp>
#include <vision_msgs/msg/point2_d.hpp>

opossum_msgs::msg::LidarLoc publish_pose_from_lidar(const Eigen::Vector3d& robot_position, const std::vector<Eigen::Vector2d>& others);

#endif // PUBLISHER_HPP_
