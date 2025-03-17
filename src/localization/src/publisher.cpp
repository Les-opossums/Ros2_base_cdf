#include "localization/publisher.hpp"

cdf_msgs::msg::LidarLoc publicate_donnees_lidar(const Eigen::Vector3d& robot_position, const std::vector<Eigen::Vector2d>& others) {
    cdf_msgs::msg::LidarLoc pos;
    pos.robot_position.x = robot_position.x();
    pos.robot_position.y = robot_position.y();
    pos.robot_position.z = robot_position.z();


    for (std::size_t i=0; i < others.size(); i++) {
        geometry_msgs::msg::Point opos;
        opos.x = others[i].x();
        opos.y = others[i].y();
        pos.other_robot_position.push_back(opos);
    }
    return pos;
}

cdf_msgs::msg::Poses2D publicate_donnees_zc(const std::vector<Eigen::Vector2d>& objects) {
    cdf_msgs::msg::Poses2D rdatas;
    for (const auto& obj : objects) {
        vision_msgs::msg::Point2D rob;
        rob.x = obj.x();
        rob.y = obj.y();
        rdatas.opos.push_back(rob);
    }
    return rdatas;
}

std::array<double, 4> get_quaternion_from_euler(double roll, double pitch, double yaw) {
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);

    return { sr * cp * cy - cr * sp * sy,
             cr * sp * cy + sr * cp * sy,
             cr * cp * sy - sr * sp * cy,
             cr * cp * cy + sr * sp * sy };
}
