#ifndef POSITION_FINDER_HPP_
#define POSITION_FINDER_HPP_

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include "localization/math_lidar.hpp"

class PositionFinder
{
public:
    PositionFinder(
        const std::array<Eigen::Vector2d, 4>& fixed_beacons,
        const std::array<double, 4>& boundaries,
        double precision = 0.1,
        std::optional<std::array<double, 3>> init_position = std::nullopt);

    std::optional<Eigen::Vector3d> search_pos(
        int nb_potential_beacons,
        const std::vector<std::array<std::optional<Eigen::Vector2d>, 4>>& potential_beacons,
        const std::vector<Eigen::Vector2d>& new_objects_detected);

    std::vector<Eigen::Vector2d> find_robots_on_plateau(const std::vector<Eigen::Vector2d>& obstacles);
    std::array<Eigen::Vector2d, 4> recreate_beacons(const Eigen::Vector3d& robot_frame);
    std::optional<Eigen::Vector3d> previous_robot;
    std::optional<Eigen::Vector3d> current_robot;

private:
    void update_data();
    void compare_previous_found_positions(
        const std::vector<std::pair<Eigen::Vector3d, double>>& potential_positions);
    void init_finding(const std::vector<std::pair<Eigen::Vector3d, double>>& potential_positions);
    std::array<Eigen::Vector2d, 4> fixed_beacons_;
    std::array<double, 4> boundaries_;
    double precision_;
    bool true_valor_;
    bool initialisation_;
    int tour_repr_;
    int tour_reinit_;
    int tour_test_;
    std::vector<std::pair<Eigen::Vector3d, int>> registre_init_;
};

#endif // POSITION_FINDER_HPP_
