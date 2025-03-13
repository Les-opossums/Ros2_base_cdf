#include "localization/PositionFinder.hpp"

// PositionFinder::PositionFinder(
//     const std::vector<Eigen::Vector2d>& fixed_beacons,
//     const std::vector<double>& boundaries,
//     double precision,
//     std::optional<Eigen::Vector3d> init_position)
//     : fixed_beacons_(fixed_beacons),
//       boundaries_(boundaries),
//       precision_(precision),
//       true_valor_(false),
//       initialisation_(false),
//       tour_repr_(0),
//       tour_reinit_(0),
//       tour_test_(0)
// {
//     if (init_position)
//     {
//         previous_robot_ = *init_position;
//         previous_robot_->z() = M_PI * init_position->z() / 180.0; // Convert angle to radians

//         // Convert beacons to robot base frame
//         for (const auto& beacon : fixed_beacons_)
//         {
//             previous_robot_->beacons.push_back(
//                 chgt_base_plateau_to_robot(beacon, *previous_robot_));
//         }

//         initialisation_ = true;
//     }
// }

// void PositionFinder::update_data()
// {
//     tour_repr_++;
//     if (!true_valor_ && initialisation_)
//     {
//         tour_test_++;
//     }
//     else if (true_valor_)
//     {
//         tour_test_ = 0;
//     }

//     if (tour_test_ >= 30)
//     {
//         tour_test_ = 0;
//         initialisation_ = false;
//     }

//     if (!initialisation_)
//     {
//         tour_reinit_++;
//     }
// }

// void PositionFinder::compare_previous_found_positions(
//     const std::vector<std::map<std::string, Eigen::Vector3d>>& potential_robot_datas)
// {
//     auto best_match = potential_robot_datas[0];
//     double min_err = best_match.at("err").x();

//     for (const auto& robot_data : potential_robot_datas)
//     {
//         double err = robot_data.at("err").x();
//         if (err < min_err)
//         {
//             min_err = err;
//             best_match = robot_data;
//         }
//     }

//     if (dt(previous_robot_->position, best_match.at("position")) < precision_)
//     {
//         current_robot_ = best_match;
//         true_valor_ = true;
//     }
//     else
//     {
//         true_valor_ = false;
//         current_robot_ = previous_robot_;
//     }
// }

// void PositionFinder::recreate_beacons()
// {
//     for (size_t i = 0; i < 4; ++i)
//     {
//         if (current_robot_->beacons[i] == Eigen::Vector2d::Zero())
//         {
//             current_robot_->beacons[i] = chgt_base_plateau_to_robot(
//                 fixed_beacons_[i], current_robot_->position);
//         }
//     }
// }

// void PositionFinder::find_robots_on_plateau(const std::vector<Eigen::Vector2d>& obstacles)
// {
//     double cos_theta = std::cos(current_robot_->position.z());
//     double sin_theta = std::sin(current_robot_->position.z());

//     Eigen::Matrix3d OtoR;
//     OtoR << cos_theta, -sin_theta, current_robot_->position.x(),
//         sin_theta, cos_theta, current_robot_->position.y(),
//         0, 0, 1;

//     current_robot_->other_robots.clear();

//     for (const auto& obstacle : obstacles)
//     {
//         Eigen::Vector3d oP = OtoR * Eigen::Vector3d(obstacle.x(), obstacle.y(), 1);
//         if (oP.x() > boundaries_[0] && oP.x() < boundaries_[1] &&
//             oP.y() > boundaries_[2] && oP.y() < boundaries_[3])
//         {
//             current_robot_->other_robots.push_back(oP);
//         }
//     }
// }

// void PositionFinder::init_finding(
//     const std::vector<std::map<std::string, Eigen::Vector3d>>& potential_robot_datas)
// {
//     if (registre_init_.empty())
//     {
//         std::vector<std::pair<std::map<std::string, Eigen::Vector3d>, int>> liste_tool_globale;

//         for (auto bal : potential_robot_datas)
//         {
//             bool found = false;
//             for (auto& element : liste_tool_globale)
//             {
//                 if (!found && std::abs(element.first.at("position").x() - bal.at("position").x()) < precision_ &&
//                     std::abs(element.first.at("position").y() - bal.at("position").y()) < precision_)
//                 {
//                     found = true;
//                     element.second += 1;
//                     element.first.at("position").x() =
//                         (element.first.at("position").x() * (element.second - 1) + bal.at("position").x()) / element.second;
//                 }
//             }
//             if (!found)
//             {
//                 liste_tool_globale.push_back({bal, 1});
//             }
//         }

//         registre_init_.push_back(liste_tool_globale);
//     }
//     else
//     {
//         if (tour_reinit_ >= 4 && !registre_init_.empty() && !registre_init_.back().empty())
//         {
//             auto best_data = registre_init_.back().front();
//             int max_count = best_data.second;

//             for (const auto& balise : registre_init_.back())
//             {
//                 if (balise.second > max_count)
//                 {
//                     max_count = balise.second;
//                     best_data = balise;
//                 }
//             }

//             best_data.first.at("position").z() = find_angle(
//                 best_data.first.at("position"),
//                 best_data.first.at("beacons"),
//                 fixed_beacons_);

//             current_robot_ = best_data.first;
//             recreate_beacons();
//             true_valor_ = true;
//             initialisation_ = true;
//         }

//         tour_reinit_ = 0;
//         tour_test_ = 0;
//         registre_init_.clear();
//     }
// }

// std::optional<Eigen::Vector3d> PositionFinder::search_pos(
//     int nb_potential_beacons,
//     const std::vector<std::vector<Eigen::Vector2d>>& potential_beacons,
//     const std::vector<Eigen::Vector2d>& obstacles)
// {
//     update_data();

//     auto potential_robot_datas = find_position(potential_beacons, fixed_beacons_, nb_potential_beacons, boundaries_);

//     if (!potential_robot_datas.empty())
//     {
//         if (!initialisation_)
//         {
//             init_finding(potential_robot_datas);
//         }
//         else
//         {
//             compare_previous_found_positions(potential_robot_datas);
//         }

//         if (initialisation_ && true_valor_)
//         {
//             recreate_beacons();
//             find_robots_on_plateau(obstacles);
//             return current_robot_->position;
//         }
//     }

//     true_valor_ = false;
//     return std::nullopt;
// }
