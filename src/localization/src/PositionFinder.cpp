#include "localization/PositionFinder.hpp"

PositionFinder::PositionFinder(
    const std::array<Eigen::Vector2d, 4>& fixed_beacons,
    const std::array<double, 4>& boundaries,
    double precision,
    std::optional<std::array<double, 3>> init_position)
    : fixed_beacons_(fixed_beacons), // Real Beacons existing
      boundaries_(boundaries), // Boundaries of the board
      precision_(precision), // Precision required
      true_valor_(false), // Has the valor been found?
      initialisation_(false), // Is the node init?
      tour_repr_(0), // Useless
      tour_reinit_(0), // Init laps
      tour_test_(0) // Test false value incr
{
    if (init_position.has_value())
    {
        Eigen::Vector3d vec_init(init_position.value()[0], init_position.value()[1], init_position.value()[2]);
        previous_robot = vec_init;
        initialisation_ = true;
    }
}

void PositionFinder::update_data()
{
    tour_repr_++;
    // If the robot is initialized but not located well, we incr the test
    if (!true_valor_ && initialisation_)
    {
        tour_test_++;
    }
    // If the position seems good, we reset the test laps
    else if (true_valor_)
    {
        tour_test_ = 0;
    }
    // We check if the robot is lost for too long
    if (tour_test_ >= 15)
    {
        tour_test_ = 0;
        initialisation_ = false;
    }

    // If the robot is not initialized, we incr the laps of init
    if (!initialisation_)
    {
        tour_reinit_++;
    }
}

void PositionFinder::compare_previous_found_positions(
    const std::vector<std::pair<Eigen::Vector3d, double>>& potential_positions)
{
    Eigen::Vector3d best_match = potential_positions[0].first;
    double min_err = potential_positions[0].second;

    for (std::size_t i = 1; i < potential_positions.size(); i++)
    {
        double err = potential_positions[i].second;
        if (err < min_err)
        {
            min_err = err;
            best_match = potential_positions[i].first;
        }
    }
    Eigen::Vector2d p1(best_match.x(), best_match.y());
    Eigen::Vector2d p2(previous_robot->x(), previous_robot->y());
    if (dt(p1, p2) < precision_)
    {
        current_robot = best_match;
        true_valor_ = true;
    }
    else
    {
        true_valor_ = false;
        current_robot = previous_robot;
    }
}

std::array<Eigen::Vector2d, 4> PositionFinder::recreate_beacons(const Eigen::Vector3d& robot_frame)
{
    std::array<Eigen::Vector2d, 4> robot_beacons;
    for (int i = 0; i < 4; ++i)
    {
        robot_beacons[i] = chgt_base_plateau_to_robot(fixed_beacons_[i], robot_frame).head<2>();
    }
    return robot_beacons;
}

std::vector<Eigen::Vector2d> PositionFinder::find_robots_on_plateau(const std::vector<Eigen::Vector2d>& new_objects_detected)
{
    std::vector<Eigen::Vector2d> other_robots;
    double cos_theta = std::cos(current_robot->z());
    double sin_theta = std::sin(current_robot->z());
    Eigen::Matrix3d OtoR;
    OtoR << cos_theta, -sin_theta, current_robot->x(),
        sin_theta, cos_theta, current_robot->y(),
        0, 0, 1;

    for (const auto& obstacle : new_objects_detected)
    {
        Eigen::Vector3d oP = OtoR * Eigen::Vector3d(obstacle.x(), obstacle.y(), 1);
        if (oP.x() > boundaries_[0] && oP.x() < boundaries_[1] &&
            oP.y() > boundaries_[2] && oP.y() < boundaries_[3] &&
            !(abs(oP.x() - current_robot->x()) < 0.2 &&
            abs(oP.y() - current_robot->y()) < 0.2 ))
        {
            Eigen::Vector2d other(oP.x(), oP.y());
            other_robots.push_back(other);
        }
    }
    return other_robots;
}

void PositionFinder::init_finding(
    const std::vector<std::pair<Eigen::Vector3d, double>>& potential_positions)
{
    for (auto pos : potential_positions)
    {
        Eigen::Vector2d p1(pos.first.x(), pos.first.y());
        bool found = false;
        std::size_t i = 0;
        while (!found && i < registre_init_.size())
        {
            Eigen::Vector2d p2(pos.first.x(), pos.first.y());
            if (dt(p1, p2) < precision_)
            {
                found = true;
                registre_init_[i].second += 1;
                registre_init_[i].first = pos.first;
            }
        }
        if (!found)
        {
            std::pair<Eigen::Vector3d, int> reg;
            reg.first = pos.first;
            reg.second = 1;
            registre_init_.push_back(reg);
        }
    }
    if (tour_reinit_ >= 4 && !registre_init_.empty())
    {
        Eigen::Vector3d best_match = registre_init_[0].first;
        int max_count = registre_init_[0].second;
        for (std::size_t i = 1; i < registre_init_.size(); i++)
        {
            if (registre_init_[i].second > max_count)
            {
                max_count = registre_init_[i].second;
                best_match = registre_init_[i].first;
            }
        }
        current_robot = best_match;
        true_valor_ = true;
        initialisation_ = true;
        tour_reinit_ = 0;
        tour_test_ = 0;
        registre_init_.clear();
    }
    else if (tour_reinit_ >= 4)
    {
        current_robot = std::nullopt;
        true_valor_ = false;
        initialisation_ = false;
        tour_reinit_ = 0;
        tour_test_ = 0;
        registre_init_.clear();
    }
}

std::optional<Eigen::Vector3d> PositionFinder::search_pos(
    int nb_potential_beacons,
    const std::vector<std::array<std::optional<Eigen::Vector2d>, 4>>& potential_beacons,
    const std::vector<Eigen::Vector2d>& new_objects_detected)
{
    update_data();
    std::vector<std::pair<Eigen::Vector3d, double>> potential_positions = find_position(potential_beacons, fixed_beacons_, nb_potential_beacons, boundaries_);
    if (!potential_positions.empty())
    {
        if (!initialisation_)
        {
            init_finding(potential_positions);
        }
        else
        {
            compare_previous_found_positions(potential_positions);
        }

        if (initialisation_ && true_valor_)
        {
            find_robots_on_plateau(new_objects_detected);
            return current_robot;
        }
        else
        {
            return std::nullopt;
        }
    }
    true_valor_ = false;
    return std::nullopt;
}
