#include "localization/beacon_detector.hpp"

BeaconDetectorNode::BeaconDetectorNode() : Node("beacon_detector_node")
{
    init_main_parameters();
}

void BeaconDetectorNode::init_main_parameters()
{
    // Declare parameters
    this->declare_parameter("enable_wait_color", false);
    this->declare_parameter("color_topic", "");
    this->declare_parameter("available_colors", std::vector<std::string>());
    this->declare_parameter("default_color", "");
    this->declare_parameter("enable_robot_position_reception", false);
    this->declare_parameter("enable_initial_position", false);
    this->declare_parameter("distance_tolerance", 0.0);
    this->declare_parameter("angle_tolerance", 0.0);
    this->declare_parameter("init_position", std::vector<double>());
    this->declare_parameter("boundaries", std::vector<double>());
    this->declare_parameter("beacons", std::vector<double>());
    this->declare_parameter("object_topic", "");
    this->declare_parameter("robot_position_topic", "");
    this->declare_parameter("position_topic", "");
    this->declare_parameter("debug_topic", "");
    this->declare_parameter("display_topic", "");

    // Get parameters
    enable_wait_color_ = this->get_parameter("enable_wait_color").as_bool();
    color_topic_ = this->get_parameter("color_topic").as_string();
    available_colors_ = this->get_parameter("available_colors").as_string_array();
    default_color_ = this->get_parameter("default_color").as_string();
    team_color_ = default_color_;
    
    // Subscribe to color topic if enabled
    if (enable_wait_color_) {
        this->create_subscription<std_msgs::msg::String>(
            color_topic_, 10, std::bind(&BeaconDetectorNode::init_color_callback, this, std::placeholders::_1));
    } else {
        init_parameters();
        init_publishers();
        init_subscribers();
    }
}

void BeaconDetectorNode::init_color_callback(const std_msgs::msg::String::SharedPtr msg)
{
    if (std::find(available_colors_.begin(), available_colors_.end(), msg->data) == available_colors_.end()) {
        RCLCPP_ERROR(this->get_logger(), "Invalid team color");
        return;
    }
    team_color_ = msg->data;
    init_parameters();
    init_publishers();
    init_subscribers();
}

void BeaconDetectorNode::init_parameters()
{
    enable_robot_position_reception_ = this->get_parameter("enable_robot_position_reception").as_bool();
    enable_initial_position_ = this->get_parameter("enable_initial_position").as_bool();
    distance_tolerance_ = this->get_parameter("distance_tolerance").as_double();
    angle_tolerance_ = this->get_parameter("angle_tolerance").as_double();
    boundaries_ = this->get_parameter("boundaries").as_double_array();
    std::vector<double> beacons = this->get_parameter("beacons").as_double_array();

    if (team_color_ == "blue") {
        for (size_t i = 1; i < beacons.size(); i += 2) {
            beacons[i] = boundaries_[3] - beacons[i];
        }
    }

    // Convert beacons into a vector of points
    for (size_t i = 0; i < beacons.size(); i += 2) {
        fixed_beacons_.emplace_back(std::vector<double>{beacons[i], beacons[i + 1]});
    }
}

void BeaconDetectorNode::init_publishers()
{
    position_topic_ = this->get_parameter("position_topic").as_string();
    pub_location_ = this->create_publisher<cdf_msgs::msg::LidarLoc>(position_topic_, 10);
}

void BeaconDetectorNode::init_subscribers()
{
    object_topic_ = this->get_parameter("object_topic").as_string();
    sub_object_ = this->create_subscription<cdf_msgs::msg::Obstacles>(
        object_topic_, 10, std::bind(&BeaconDetectorNode::object_callback, this, std::placeholders::_1));
}

void BeaconDetectorNode::object_callback(const cdf_msgs::msg::Obstacles::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Processing obstacle data...");
    // Add beacon detection logic here
}

void BeaconDetectorNode::robot_position_callback(const cdf_msgs::msg::MergedData::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Updating robot position...");
    // Add logic for robot position update
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BeaconDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
