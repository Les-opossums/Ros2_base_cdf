#ifndef BEACON_DETECTOR_NODE_HPP_
#define BEACON_DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
// #include <visualization_msgs/msg/marker_array.hpp>
#include <obstacle_detector/msg/obstacles.hpp>
#include <cdf_msgs/msg/lidar_loc.hpp>
#include <cdf_msgs/msg/merged_data.hpp>
#include <vector>
#include <string>
#include <array>
#include <Eigen/Dense>

#include "opossum_localisation/math_lidar.hpp"
#include "opossum_localisation/BeaconSorter.hpp"
#include "opossum_localisation/PositionFinder.hpp"
#include "opossum_localisation/publisher.hpp"

class BeaconDetectorNode : public rclcpp::Node
{
public:
    BeaconDetectorNode();

private:
    // Initialization functions
    void init_main_parameters();
    void init_color_callback(const std_msgs::msg::String::SharedPtr msg);
    void init_parameters();
    void init_publishers();
    void init_subscribers();

    // Callbacks
    void object_callback(const obstacle_detector::msg::Obstacles::SharedPtr msg);
    void robot_position_callback(const cdf_msgs::msg::MergedData::SharedPtr msg);

    // Parameters
    bool enable_robot_position_reception_;
    bool enable_initial_position_;
    double distance_tolerance_;
    double angle_tolerance_;
    std::array<double, 4> boundaries_;
    std::array<double, 3> init_position_;
    std::array<Eigen::Vector2d, 4> fixed_beacons_;
    std::array<bool, 4> angle_sign;

    // Team color handling
    bool enable_wait_color_;
    std::string color_topic_;
    std::string default_color_;
    std::string team_color_;
    std::vector<std::string> available_colors_;

    // Topics
    std::string object_topic_;
    std::string robot_position_topic_;
    std::string position_topic_;
    std::string debug_topic_;
    std::string display_topic_;

    // ROS 2 communication
    rclcpp::Publisher<cdf_msgs::msg::LidarLoc>::SharedPtr pub_location_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_debug_;

    rclcpp::Subscription<obstacle_detector::msg::Obstacles>::SharedPtr sub_object_;
    rclcpp::Subscription<cdf_msgs::msg::MergedData>::SharedPtr sub_robot_position_;

    // opossum_localisation Objects
    std::shared_ptr<PositionFinder> position_finder_;
    std::shared_ptr<BeaconSorter> beacon_sorter_;
};

#endif // BEACON_DETECTOR_NODE_HPP_
