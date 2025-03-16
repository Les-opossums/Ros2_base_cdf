#ifndef BEACON_SORTER_HPP_
#define BEACON_SORTER_HPP_

#include <vector>
#include <map>
#include <array>
#include <string>
#include <memory>
#include <utility>
#include <optional>
#include <algorithm>
#include <cmath>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "localization/math_lidar.hpp"

class BeaconSorter
{
public:
    BeaconSorter(
        const std::array<double, 6>& dst_beacons, // AB, AC, BC, AD, BD, CD
        const std::array<bool, 4>& angle_sign, // ABC, ABD, ACD, BCD
        double ang_tol,
        double dst_tol);

        std::pair<int, std::vector<std::array<std::optional<Eigen::Vector2d>, 4>>> find_possible_beacons(
            const std::array<std::optional<Eigen::Vector2d>, 4>& previous_beacons,
            const std::vector<Eigen::Vector2d>& new_objects_detected);

private:
    std::array<double, 6> dst_beacons_;
    std::array<bool, 4> angle_sign_;
    double ang_tol_;
    double dst_tol_;

    std::map<char, std::vector<Eigen::Vector2d>> sort_comparison(
        const std::array<std::optional<Eigen::Vector2d>, 4>& previous_beacons,
        const std::vector<Eigen::Vector2d>& new_objects_detected);

    std::pair<int, std::vector<std::array<std::optional<Eigen::Vector2d>, 4>>> find_beacons_prev(
        const std::array<std::optional<Eigen::Vector2d>, 4>& previous_beacons,
        const std::vector<Eigen::Vector2d>& new_objects_detected);

    std::pair<int, std::vector<std::array<std::optional<Eigen::Vector2d>, 4>>> find_beacons_naive(
        const std::vector<Eigen::Vector2d>& new_objects_detected);
};

#endif // BEACON_SORTER_HPP_
