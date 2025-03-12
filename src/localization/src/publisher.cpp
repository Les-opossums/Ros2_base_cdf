#include "localization/publisher.hpp"

cdf_msgs::msg::Poses2D publicate_donnees_lidar(const std::vector<Eigen::Vector3d>& robot_datas) {
    cdf_msgs::msg::Poses2D rdatas;
    rdatas.rpos.position.x = robot_datas[0].x();
    rdatas.rpos.position.y = robot_datas[0].y();
    rdatas.rpos.theta = robot_datas[0].z();

    if (robot_datas.size() > 5){
        for (long unsigned int i=5; i<robot_datas.size(); i++) {
            vision_msgs::msg::Point2D opos;
            opos.x = robot_datas[i].x();
            opos.y = robot_datas[i].y();
            rdatas.opos.push_back(opos);
        }    
    }
    return rdatas;
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
