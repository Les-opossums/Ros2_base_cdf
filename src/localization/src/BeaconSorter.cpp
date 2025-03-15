#include "localization/BeaconSorter.hpp"

BeaconSorter::BeaconSorter(
    const std::array<double, 6>& dst_beacons,
    const std::array<bool, 4>& angle_sign,
    double ang_tol,
    double dst_tol)
    : dst_beacons_(dst_beacons),
      angle_sign_(angle_sign),
      ang_tol_(ang_tol),
      dst_tol_(dst_tol)
{
}

std::map<char, std::vector<Eigen::Vector2d>> BeaconSorter::sort_comparison(
    const std::array<std::optional<Eigen::Vector2d>, 4>& beacons,
    const std::vector<Eigen::Vector2d>& new_objects_detected)
{
    std::map<char, std::vector<Eigen::Vector2d>> sort_list = {
        {'A', {}}, {'B', {}}, {'C', {}}, {'D', {}}};

    std::array<std::pair<double, double>, 4> beacons_ang;
    std::array<double, 4> beacons_dst;

    for (int i = 0; i < 4; i++)
    {
        double angle, distance;
        std::tie(angle, distance) = cartesian_to_polar(beacons[i]->x(), beacons[i]->y());
        beacons_ang[i] = {angle - ang_tol_, angle + ang_tol_};
        beacons_dst[i] = distance;
    }

    for (const auto& obstacle : new_objects_detected)
    {
        double angle, distance;
        std::tie(angle, distance) = cartesian_to_polar(obstacle.x(), obstacle.y());

        for (int i = 0; i < 4; i++)
        {
            if (std::abs(distance - beacons_dst[i]) < dst_tol_ &&
                angle_in_range(angle, beacons_ang[i].first, beacons_ang[i].second))
            {
                char let = 'A' + i;
                sort_list[let].push_back(obstacle);
            }
        }
    }

    for (auto& [key, val] : sort_list)
    {
        if (val.empty())
        {
            val.push_back(Eigen::Vector2d(-100., -100.));
        }
    }

    return sort_list;
}

std::pair<int, std::vector<std::array<std::optional<Eigen::Vector2d>, 4>>> BeaconSorter::find_beacons_prev(
    const std::array<std::optional<Eigen::Vector2d>, 4>& beacons,
    const std::vector<Eigen::Vector2d>& new_objects_detected)
{
    auto sort_list = sort_comparison(beacons, new_objects_detected);
    std::vector<std::array<int, 4>> indexes2, indexes3, indexes4;
    bool flags[6] = {false};

    for (std::size_t i0 = 0; i0 < sort_list['A'].size(); i0++)
    {
        int i = static_cast<int>(i0);
        for (std::size_t j0 = 0; j0 < sort_list['B'].size(); j0++)
        {
            int j = static_cast<int>(j0);
            flags[0] = false;
            if (approx(dt(sort_list['A'][i], sort_list['B'][j]), dst_beacons_[0], dst_tol_))
            {
                flags[0] = true;
                indexes2.push_back({i, j, -1, -1});
            }
            for (std::size_t k0 = 0; k0 < sort_list['C'].size(); k0++)
            {
                int k = static_cast<int>(k0);
                flags[1] = false;
                flags[2] = false;
                if (approx(dt(sort_list['A'][i], sort_list['C'][k]), dst_beacons_[1], dst_tol_))
                {
                    flags[1] = true;
                    indexes2.push_back({i, -1, k, -1});
                }
                if (approx(dt(sort_list['B'][j], sort_list['C'][k]), dst_beacons_[2], dst_tol_))
                {
                    flags[2] = true;
                    indexes2.push_back({-1, j, k, -1});
                }
                if (flags[0] && flags[1] && flags[2])
                {
                    indexes3.push_back({i, j, k, -1});
                }
                for (std::size_t l0 = 0; l0 < sort_list['D'].size(); l0++)
                {
                    int l = static_cast<int>(l0);
                    flags[3] = false;
                    flags[4] = false;
                    flags[5] = false;
                    if (approx(dt(sort_list['A'][i], sort_list['D'][l]), dst_beacons_[3], dst_tol_))
                    {
                        flags[3] = true;
                        indexes2.push_back({i, -1, -1, l});
                    }
                    if (approx(dt(sort_list['B'][j], sort_list['D'][l]), dst_beacons_[4], dst_tol_))
                    {
                        flags[4] = true;
                        indexes2.push_back({-1, j, -1, l});
                    }
                    if (approx(dt(sort_list['C'][k], sort_list['D'][l]), dst_beacons_[5], dst_tol_))
                    {
                        flags[5] = true;
                        indexes2.push_back({-1, -1, k, l});
                    }
                    if (flags[0] && flags[3] && flags[4])
                    {
                        indexes3.push_back({i, j, -1, l});
                    }
                    if (flags[1] && flags[3] && flags[5])
                    {
                        indexes3.push_back({i, -1, k, l});
                    }
                    if (flags[2] && flags[4] && flags[5])
                    {
                        indexes3.push_back({-1, j, k, l});
                    }
                    if (flags[2] && flags[4] && flags[5])
                    {
                        indexes3.push_back({-1, j, k, l});
                    }
                    if (flags[0] && flags[1] && flags[2] && flags[3] && flags[4] && flags[5])
                    {
                        indexes4.push_back({i, j, k, l});
                    }
                }
            }
        }
    }

    if (!indexes4.empty())
    {
        std::sort(indexes4.begin(), indexes4.end());
        auto last = std::unique(indexes4.begin(), indexes4.end());
        indexes4.erase(last, indexes4.end());
        std::vector<std::array<std::optional<Eigen::Vector2d>, 4>> clean_beacons4;
        for (auto const& index: indexes4) 
        {
            std::array<std::optional<Eigen::Vector2d>, 4> beacon;
            for (int i = 0; i < 4; i++)
            {
                char let = 'A' + i;
                beacon[i] = sort_list[let][index[i]];
            }
            clean_beacons4.push_back(beacon);
        }
        std::vector<std::array<std::optional<Eigen::Vector2d>, 4>> temp_beacons;
        for (int i=temp_beacons.size(); i>=0; i--)
        {
            std::array<std::optional<Eigen::Vector2d>, 3> signABC = {temp_beacons[i][0], temp_beacons[i][1], temp_beacons[i][2]};
            std::array<std::optional<Eigen::Vector2d>, 3> signABD = {temp_beacons[i][0], temp_beacons[i][1], temp_beacons[i][3]};
            std::array<std::optional<Eigen::Vector2d>, 3> signACD = {temp_beacons[i][0], temp_beacons[i][2], temp_beacons[i][3]};
            std::array<std::optional<Eigen::Vector2d>, 3> signBCD = {temp_beacons[i][1], temp_beacons[i][2], temp_beacons[i][3]};
            if (getAngleSign(signABC) != angle_sign_[0] || getAngleSign(signABD) != angle_sign_[1] 
            || getAngleSign(signACD) != angle_sign_[2] || getAngleSign(signBCD) != angle_sign_[3])
            {
                clean_beacons4.erase(clean_beacons4.begin() + i);
            }
        }
        if (!clean_beacons4.empty()) return {4, clean_beacons4};
    }
    if (!indexes3.empty())
    {
        std::sort(indexes3.begin(), indexes3.end());
        auto last = std::unique(indexes3.begin(), indexes3.end());
        indexes3.erase(last, indexes3.end());
        std::vector<std::array<std::optional<Eigen::Vector2d>, 4>> clean_beacons3;
        for (auto const& index: indexes3) 
        {
            std::array<std::optional<Eigen::Vector2d>, 4> beacon;
            for (int i = 0; i < 4; i++)
            {
                if (index[i] >= 0) 
                {
                    char let = 'A' + i;
                    beacon[i] = sort_list[let][index[i]];
                }
                else beacon[i] = std::nullopt;
            }
            clean_beacons3.push_back(beacon);
        }
        std::vector<std::array<std::optional<Eigen::Vector2d>, 4>> temp_beacons;
        for (int i=temp_beacons.size(); i>=0; i--)
        {
            if (!temp_beacons[i][0].has_value())
            {
                std::array<std::optional<Eigen::Vector2d>, 3> signABC = {temp_beacons[i][0], temp_beacons[i][1], temp_beacons[i][2]};
                if (getAngleSign(signABC) != angle_sign_[0]) clean_beacons3.erase(clean_beacons3.begin() + i);
            }
            else if (!temp_beacons[i][1].has_value())
            {
                std::array<std::optional<Eigen::Vector2d>, 3> signABD = {temp_beacons[i][0], temp_beacons[i][1], temp_beacons[i][3]};
                if (getAngleSign(signABD) != angle_sign_[1]) clean_beacons3.erase(clean_beacons3.begin() + i);
            }
            else if (!temp_beacons[i][2].has_value())
            {
                std::array<std::optional<Eigen::Vector2d>, 3> signACD = {temp_beacons[i][0], temp_beacons[i][2], temp_beacons[i][3]};
                if (getAngleSign(signACD) != angle_sign_[2]) clean_beacons3.erase(clean_beacons3.begin() + i);
            }
            else if (!temp_beacons[i][3].has_value())
            {
                std::array<std::optional<Eigen::Vector2d>, 3> signBCD = {temp_beacons[i][1], temp_beacons[i][2], temp_beacons[i][3]};
                if (getAngleSign(signBCD) != angle_sign_[3]) clean_beacons3.erase(clean_beacons3.begin() + i);
            }
        }
        if (!clean_beacons3.empty()) return {3, clean_beacons3};
    }
    if (!indexes2.empty())
    {
        std::sort(indexes2.begin(), indexes2.end());
        auto last = std::unique(indexes2.begin(), indexes2.end());
        indexes2.erase(last, indexes2.end());
        std::vector<std::array<std::optional<Eigen::Vector2d>, 4>> clean_beacons2;
        for (auto const& index: indexes2) 
        {
            std::array<std::optional<Eigen::Vector2d>, 4> beacon;
            for (int i = 0; i < 4; i++)
            {
                if (index[i] >= 0) 
                {
                    char let = 'A' + i;
                    beacon[i] = sort_list[let][index[i]];
                }
                else beacon[i] = std::nullopt;
            }
            clean_beacons2.push_back(beacon);
        }
        return {2, clean_beacons2};
    }

    return {0, {}};
}


std::pair<int, std::vector<std::array<std::optional<Eigen::Vector2d>, 4>>> BeaconSorter::find_possible_beacons(
    const std::array<std::optional<Eigen::Vector2d>, 4>& previous_beacons,
    const std::vector<Eigen::Vector2d>& new_objects_detected) {
        std::pair<int, std::vector<std::array<std::optional<Eigen::Vector2d>, 4>>> beacons_data; 
    if (previous_beacons[0].has_value()) {
        beacons_data = find_beacons_prev(previous_beacons, new_objects_detected);
        if (beacons_data.first > 1) {
            return beacons_data;
        }
    }
    return find_beacons_naive(new_objects_detected);

};


std::pair<int, std::vector<std::array<std::optional<Eigen::Vector2d>, 4>>> BeaconSorter::find_beacons_naive(
    const std::vector<Eigen::Vector2d>& new_objects_detected)
{
    // std::vector<Eigen::Vector2d> olist = new_objects_detected;
    // std::vector<std::vector<std::optional<Eigen::Vector2d>>> beacon_list4, beacon_list3, beacon_list2;

    // std::vector<char> letters = {'A', 'B', 'C', 'D'};

    // while (olist.size() >= 2)
    // {
    //     Eigen::Vector2d beacon1 = olist.back();
    //     olist.pop_back();

    //     std::map<std::pair<char, char>, std::vector<int>> dict_indexes;
    //     for (char i = 0; i < 4; ++i)
    //     {
    //         for (char j = i + 1; j < 4; ++j)
    //         {
    //             dict_indexes[{letters[i], letters[j]}] = {};
    //         }
    //     }

    //     for (size_t index = 0; index < olist.size(); ++index)
    //     {
    //         double distance = dt(beacon1, olist[index]);
    //         for (const auto& key : dict_indexes)
    //         {
    //             if (approx(distance, dst_beacons_[key.first], dst_tol_))
    //             {
    //                 dict_indexes[key.first].push_back(index);
    //             }
    //         }
    //     }
    // }

    // if (!beacon_list4.empty())
    // {
    //     return {4, beacon_list4};
    // }
    // if (!beacon_list3.empty())
    // {
    //     return {3, beacon_list3};
    // }
    // if (!beacon_list2.empty())
    // {
    //     return {2, beacon_list2};
    // }
    // 
    // return {0, {}};
    std::pair<int, std::vector<std::array<std::optional<Eigen::Vector2d>, 4>>> result;
    return result;
}
