#ifndef POSITION_FINDER_HPP_
#define POSITION_FINDER_HPP_

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "localization/math_lidar.hpp"

class PositionFinder
{
public:
    PositionFinder(
        const std::vector<Eigen::Vector2d>& fixed_beacons,
        const std::vector<double>& boundaries,
        double precision = 0.1,
        std::optional<Eigen::Vector3d> init_position = std::nullopt);

    std::optional<Eigen::Vector3d> search_pos(
        int nb_potential_beacons,
        const std::vector<std::vector<Eigen::Vector2d>>& potential_beacons,
        const std::vector<Eigen::Vector2d>& obstacles);

private:
    void update_data();
    void compare_previous_found_positions(
        const std::vector<std::map<std::string, Eigen::Vector3d>>& potential_robot_datas);
    void recreate_beacons();
    void find_robots_on_plateau(const std::vector<Eigen::Vector2d>& obstacles);
    void init_finding(const std::vector<std::map<std::string, Eigen::Vector3d>>& potential_robot_datas);

    bool true_valor_;
    bool initialisation_;
    double precision_;
    int tour_repr_;
    int tour_reinit_;
    int tour_test_;

    std::vector<double> boundaries_;
    std::vector<Eigen::Vector2d> fixed_beacons_;
    std::optional<Eigen::Vector3d> previous_robot_;
    std::optional<Eigen::Vector3d> current_robot_;
    std::vector<std::vector<std::map<std::string, Eigen::Vector3d>>> registre_init_;
};

#endif // POSITION_FINDER_HPP_
