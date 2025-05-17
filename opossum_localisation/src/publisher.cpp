#include "opossum_localisation/publisher.hpp"

cdf_msgs::msg::LidarLoc publish_pose_from_lidar(const Eigen::Vector3d& robot_position, const std::vector<Eigen::Vector2d>& others) {
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
