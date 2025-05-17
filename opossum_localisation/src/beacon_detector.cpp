#include "opossum_localisation/beacon_detector.hpp"

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
    auto init_pos = this->get_parameter("init_position").as_double_array();
    if (init_pos.size() != 3) {
        throw std::runtime_error("init_position must have exactly 3 elements");
    }
    std::copy(init_pos.begin(), init_pos.end(), init_position_.begin());
    distance_tolerance_ = this->get_parameter("distance_tolerance").as_double();
    angle_tolerance_ = this->get_parameter("angle_tolerance").as_double();
    auto bound = this->get_parameter("boundaries").as_double_array();
    if (bound.size() != 4) {
        throw std::runtime_error("boundaries must have exactly 4 elements");
    }
    std::copy(bound.begin(), bound.end(), boundaries_.begin());
    std::vector<double> beacons = this->get_parameter("beacons").as_double_array();

    if (std::find(available_colors_.begin(), available_colors_.end(), team_color_) == available_colors_.end()) {
        RCLCPP_ERROR(this->get_logger(), "Invalid team color");
        return;
    }

    if (team_color_ == "blue") {
        for (size_t i = 1; i < beacons.size(); i += 2) {
            beacons[i] = boundaries_[3] - beacons[i];
        }
    }

    // Convert beacons into a vector of points
    for (size_t i = 0; i < 4; i++) {
        fixed_beacons_[i] = {beacons[2 * i], beacons[2 * i + 1]};
    }

    std::array<Eigen::Vector2d, 3> signABC = {fixed_beacons_[0], fixed_beacons_[1], fixed_beacons_[2]};
    std::array<Eigen::Vector2d, 3> signABD = {fixed_beacons_[0], fixed_beacons_[1], fixed_beacons_[3]};
    std::array<Eigen::Vector2d, 3> signACD = {fixed_beacons_[0], fixed_beacons_[2], fixed_beacons_[3]};
    std::array<Eigen::Vector2d, 3> signBCD = {fixed_beacons_[1], fixed_beacons_[2], fixed_beacons_[3]};
    angle_sign[0] = getAngleSign(signABC);
    angle_sign[1] = getAngleSign(signABD);
    angle_sign[2] = getAngleSign(signACD);
    angle_sign[3] = getAngleSign(signBCD);

    // SKIP POSITION FINDER FOR NOW
    std::array<double, 6> dst_beacons = {dt(fixed_beacons_[0], fixed_beacons_[1]),
        dt(fixed_beacons_[0], fixed_beacons_[2]),
        dt(fixed_beacons_[1], fixed_beacons_[2]),
        dt(fixed_beacons_[0], fixed_beacons_[3]),
        dt(fixed_beacons_[1], fixed_beacons_[3]),
        dt(fixed_beacons_[2], fixed_beacons_[3])};

    beacon_sorter_ = std::make_shared<BeaconSorter>(dst_beacons, angle_sign, angle_tolerance_, distance_tolerance_);
    position_finder_ = std::make_shared<PositionFinder>(fixed_beacons_, boundaries_, distance_tolerance_, init_position_);

    RCLCPP_INFO(this->get_logger(), "Initialized beacon_detector_node.");
}

void BeaconDetectorNode::init_publishers()
{
    position_topic_ = this->get_parameter("position_topic").as_string();
    pub_location_ = this->create_publisher<cdf_msgs::msg::LidarLoc>(position_topic_, 10);
    pub_location_cmd_ = this->create_publisher<std_msgs::msg::String>("command", 10);
}

void BeaconDetectorNode::init_subscribers()
{
    object_topic_ = this->get_parameter("object_topic").as_string();
    sub_object_ = this->create_subscription<obstacle_detector::msg::Obstacles>(
        object_topic_, 10, std::bind(&BeaconDetectorNode::object_callback, this, std::placeholders::_1));
    if (enable_robot_position_reception_)
    {
        robot_position_topic_ = this->get_parameter("robot_position_topic").as_string();
        sub_robot_position_ = this->create_subscription<cdf_msgs::msg::MergedData>(
            robot_position_topic_, 10, std::bind(&BeaconDetectorNode::robot_position_callback, this, std::placeholders::_1));
    }
}

void BeaconDetectorNode::object_callback(const obstacle_detector::msg::Obstacles::SharedPtr msg)
{
    // rclcpp::Time begin = this->now();
    // int64_t ns_begin = begin.nanoseconds();
    // rclcpp::Time last_time = msg->header.stamp;
    // int64_t ns_last_time = last_time.nanoseconds();

    if ((!enable_robot_position_reception_ || !position_finder_->previous_robot.has_value()) && position_finder_->current_robot.has_value())
    {
        position_finder_->previous_robot = position_finder_->current_robot;
    }
    std::optional<std::array<Eigen::Vector2d, 4>> previous_beacons;

    if (position_finder_->previous_robot.has_value())
    {
        previous_beacons = position_finder_->recreate_beacons(position_finder_->previous_robot.value());
    }
    else
    {
        previous_beacons = {std::nullopt};
    }

    std::vector<Eigen::Vector2d> new_objects_detected;
    for (std::size_t i = 0; i < msg->circles.size(); i++)
    {
        if (pow(msg->circles[i].center.x, 2) + pow(msg->circles[i].center.y, 2) < 13.5)
        {
            new_objects_detected.push_back({msg->circles[i].center.x, msg->circles[i].center.y});
        }
    }
    std::pair<int, std::vector<std::array<std::optional<Eigen::Vector2d>, 4>>> beacons_result;
    beacons_result = beacon_sorter_->find_possible_beacons(previous_beacons, new_objects_detected);
    // RCLCPP_INFO(this->get_logger(), "NUMBER BEACONS %d", beacons_result.first);
    if (beacons_result.first > 1)
    {
        std::pair<std::array<std::optional<Eigen::Vector2d>, 4>, std::optional<Eigen::Vector3d>> output = position_finder_->search_pos(beacons_result.first, beacons_result.second, new_objects_detected);
        std::array<std::optional<Eigen::Vector2d>, 4> beacons = output.first;
        std::optional<Eigen::Vector3d> position_found = output.second;
        if (position_found.has_value())
        {
            std::vector<Eigen::Vector2d> others;
            others = position_finder_->find_robots_on_plateau(new_objects_detected);
            cdf_msgs::msg::LidarLoc msg = publish_pose_from_lidar(position_found.value(), others);
            pub_location_->publish(msg);
            std_msgs::msg::String msg_cmd;
            std::ostringstream ss;
            ss << "SETLIDAR "
            << position_found->x() << " "
            << position_found->y() << " "
            << position_found->z();
            msg_cmd.data = ss.str();
            // Log the message to console
            // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg_cmd.data.c_str());

            // Publish the message.
            pub_location_cmd_->publish(msg_cmd);
        }
    }
    // rclcpp::Time now = this->now();
    // int64_t ns_now = now.nanoseconds();
    // RCLCPP_INFO(this->get_logger(), "Current time: %d seconds", ns_now - ns_begin);
    // RCLCPP_INFO(this->get_logger(), "Last Time: %d seconds", ns_now - ns_last_time);
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
