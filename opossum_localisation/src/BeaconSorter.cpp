#include "opossum_localisation/BeaconSorter.hpp"

BeaconSorter::BeaconSorter(
    const std::array<double, 6>& dst_beacons,
    const std::array<bool, 4>& angle_sign,
    double ang_tol,
    double dst_tol,
    double dst_tol_beacons)
    : dst_beacons_(dst_beacons),
      angle_sign_(angle_sign),
      ang_tol_(ang_tol),
      dst_tol_(dst_tol),
      dst_tol_beacons_(dst_tol_beacons)
{
}

std::map<char, std::vector<Eigen::Vector2d>> BeaconSorter::sort_comparison(
    const std::array<Eigen::Vector2d, 4>& beacons,
    const std::vector<Eigen::Vector2d>& new_objects_detected)
{
    std::map<char, std::vector<Eigen::Vector2d>> sort_list = {
        {'A', {}}, {'B', {}}, {'C', {}}, {'D', {}}};

    std::array<std::pair<double, double>, 4> beacons_ang;
    std::array<double, 4> beacons_dst;

    for (int i = 0; i < 4; i++)
    {
        double angle, distance;
        std::tie(angle, distance) = cartesian_to_polar(beacons[i].x(), beacons[i].y());
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
    const std::array<Eigen::Vector2d, 4>& beacons,
    const std::vector<Eigen::Vector2d>& new_objects_detected)
{

    auto sort_list = sort_comparison(beacons, new_objects_detected);
    std::vector<std::array<int, 4>> indexes2, indexes3, indexes4;
    bool flags[6] = {false}; // AB, AC, BC, AD, BD, CD

    for (std::size_t i0 = 0; i0 < sort_list['A'].size(); i0++)
    {
        int i = static_cast<int>(i0);
        for (std::size_t j0 = 0; j0 < sort_list['B'].size(); j0++)
        {
            int j = static_cast<int>(j0);
            flags[0] = false;
            if (approx(dt(sort_list['A'][i], sort_list['B'][j]), dst_beacons_[0], dst_tol_beacons_))
            {
                flags[0] = true;
                indexes2.push_back({i, j, -1, -1});
            }
            for (std::size_t k0 = 0; k0 < sort_list['C'].size(); k0++)
            {
                int k = static_cast<int>(k0);
                flags[1] = false;
                flags[2] = false;
                if (approx(dt(sort_list['A'][i], sort_list['C'][k]), dst_beacons_[1], dst_tol_beacons_))
                {
                    flags[1] = true;
                    indexes2.push_back({i, -1, k, -1});
                }
                if (approx(dt(sort_list['B'][j], sort_list['C'][k]), dst_beacons_[2], dst_tol_beacons_))
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
                    if (approx(dt(sort_list['A'][i], sort_list['D'][l]), dst_beacons_[3], dst_tol_beacons_))
                    {
                        flags[3] = true;
                        indexes2.push_back({i, -1, -1, l});
                    }
                    if (approx(dt(sort_list['B'][j], sort_list['D'][l]), dst_beacons_[4], dst_tol_beacons_))
                    {
                        flags[4] = true;
                        indexes2.push_back({-1, j, -1, l});
                    }
                    if (approx(dt(sort_list['C'][k], sort_list['D'][l]), dst_beacons_[5], dst_tol_beacons_))
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
        for (int i = clean_beacons4.size() - 1; i >= 0; i--)
        {
            std::array<Eigen::Vector2d, 3> signABC = {clean_beacons4[i][0].value(), clean_beacons4[i][1].value(), clean_beacons4[i][2].value()};
            std::array<Eigen::Vector2d, 3> signABD = {clean_beacons4[i][0].value(), clean_beacons4[i][1].value(), clean_beacons4[i][3].value()};
            std::array<Eigen::Vector2d, 3> signACD = {clean_beacons4[i][0].value(), clean_beacons4[i][2].value(), clean_beacons4[i][3].value()};
            std::array<Eigen::Vector2d, 3> signBCD = {clean_beacons4[i][1].value(), clean_beacons4[i][2].value(), clean_beacons4[i][3].value()};
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

        for (int i = clean_beacons3.size() - 1; i >= 0; i--)
        {
            if (!clean_beacons3[i][3].has_value())
            {

                std::array<Eigen::Vector2d, 3> signABC = {clean_beacons3[i][0].value(), clean_beacons3[i][1].value(), clean_beacons3[i][2].value()};
                if (getAngleSign(signABC) != angle_sign_[0]) clean_beacons3.erase(clean_beacons3.begin() + i);
            }
            else if (!clean_beacons3[i][2].has_value())
            {
                std::array<Eigen::Vector2d, 3> signABD = {clean_beacons3[i][0].value(), clean_beacons3[i][1].value(), clean_beacons3[i][3].value()};
                if (getAngleSign(signABD) != angle_sign_[1]) clean_beacons3.erase(clean_beacons3.begin() + i);
            }
            else if (!clean_beacons3[i][1].has_value())
            {

                std::array<Eigen::Vector2d, 3> signACD = {clean_beacons3[i][0].value(), clean_beacons3[i][2].value(), clean_beacons3[i][3].value()};
                if (getAngleSign(signACD) != angle_sign_[2]) clean_beacons3.erase(clean_beacons3.begin() + i);
            }
            else if (!clean_beacons3[i][0].has_value())
            {

                std::array<Eigen::Vector2d, 3> signBCD = {clean_beacons3[i][1].value(), clean_beacons3[i][2].value(), clean_beacons3[i][3].value()};
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
    const std::optional<std::array<Eigen::Vector2d, 4>>& previous_beacons,
    const std::vector<Eigen::Vector2d>& new_objects_detected) {
    std::pair<int, std::vector<std::array<std::optional<Eigen::Vector2d>, 4>>> beacons_data;

    if (previous_beacons.has_value()) {
        beacons_data = find_beacons_prev(previous_beacons.value(), new_objects_detected);

        if (beacons_data.first > 2) {
            return beacons_data;
        }
    }
    return find_beacons_naive(new_objects_detected);

};


std::pair<int, std::vector<std::array<std::optional<Eigen::Vector2d>, 4>>> BeaconSorter::find_beacons_naive(
    const std::vector<Eigen::Vector2d>& new_objects_detected)
{
    std::vector<Eigen::Vector2d> olist = new_objects_detected;
    std::vector<std::array<std::optional<Eigen::Vector2d>, 4>> beacon_list2, beacon_list3, beacon_list4;
    // indexes[0]: AB
    // indexes[1]: AC
    // indexes[2]: BC
    // indexes[3]: AD
    // indexes[0]: BD
    // indexes[0]: CD
    std::array<std::vector<int>, 6> indexes;
    while (olist.size() > 1)
    {
        for (int i =0 ; i < 6; i++){
            indexes[i].clear();
        }
        Eigen::Vector2d beacon1 = olist.back();
        olist.pop_back();
        for (std::size_t i0 = 0; i0 < olist.size(); i0++)
        {
            int i = static_cast<int>(i0);
            double distance = dt(beacon1, olist[i]);
            if (approx(distance, dst_beacons_[0], dst_tol_beacons_))
            {
                indexes[0].push_back(i);
                beacon_list2.push_back({olist[i], beacon1, std::nullopt, std::nullopt});
                beacon_list2.push_back({beacon1, olist[i], std::nullopt, std::nullopt});
            }
            if (approx(distance, dst_beacons_[1], dst_tol_beacons_))
            {
                indexes[1].push_back(i);
                beacon_list2.push_back({olist[i], std::nullopt, beacon1, std::nullopt});
                beacon_list2.push_back({beacon1, std::nullopt, olist[i], std::nullopt});
            }
            if (approx(distance, dst_beacons_[2], dst_tol_beacons_))
            {
                indexes[2].push_back(i);
                beacon_list2.push_back({std::nullopt, olist[i], beacon1, std::nullopt});
                beacon_list2.push_back({std::nullopt, beacon1, olist[i], std::nullopt});
            }
            if (approx(distance, dst_beacons_[3], dst_tol_beacons_))
            {
                indexes[3].push_back(i);
                beacon_list2.push_back({olist[i], std::nullopt, std::nullopt, beacon1});
                beacon_list2.push_back({beacon1, std::nullopt, std::nullopt, olist[i]});
            }
            if (approx(distance, dst_beacons_[4], dst_tol_beacons_))
            {
                indexes[4].push_back(i);
                beacon_list2.push_back({std::nullopt, olist[i], std::nullopt, beacon1});
                beacon_list2.push_back({std::nullopt, beacon1, std::nullopt, olist[i]});
            }
            if (approx(distance, dst_beacons_[5], dst_tol_beacons_))
            {
                indexes[5].push_back(i);
                beacon_list2.push_back({std::nullopt, std::nullopt, olist[i], beacon1});
                beacon_list2.push_back({std::nullopt, std::nullopt, beacon1, olist[i]});
            }
        }
        ///
        /// A TEST
        ///
        // The list of AB links or BA, but test as if the current was A
        for (std::size_t i = 0; i < indexes[0].size(); i++)
        {
            std::vector<int> test_indexesA;
            // Comparing AB with AC to get the check for ABC combinations
            // In the same time, saving the positions of the interesting points to compare then
            // The list of AC, Checking the distance BC
            for (std::size_t j = 0; j < indexes[1].size(); j++)
            {
                double distance = dt(olist[indexes[0][i]], olist[indexes[1][j]]);
                if (approx(distance, dst_beacons_[2]))
                {
                    beacon_list3.push_back({beacon1, olist[indexes[0][i]], olist[indexes[1][j]], std::nullopt});
                    test_indexesA.push_back(indexes[1][j]);
                }
            }
            // Compare now the AD list combination, to see if there is a link between ABD
            for (std::size_t j = 0; j < indexes[3].size(); j++)
            {
                double distance = dt(olist[indexes[0][i]], olist[indexes[3][j]]);
                if (approx(distance, dst_beacons_[4]))
                {
                    beacon_list3.push_back({beacon1, olist[indexes[0][i]], std::nullopt, olist[indexes[3][j]]});
                    // We check AB, AC, BC, AD, BD, need CD to have the 4 beacons now that we have in values to test
                    for (std::size_t k = 0; k < test_indexesA.size(); k++)
                    {
                        double distance = dt(olist[test_indexesA[k]], olist[indexes[3][j]]);
                        if (approx(distance, dst_beacons_[5]))
                        {
                            beacon_list4.push_back({beacon1, olist[indexes[0][i]], olist[test_indexesA[k]], olist[indexes[3][j]]});
                        }
                    }
                }
            }
        }
        // The list of AC
        for (std::size_t i = 0; i < indexes[1].size(); i++)
        {
            // The list of AD, Checking the distance CD
            for (std::size_t j = 0; j < indexes[3].size(); j++)
            {
                double distance = dt(olist[indexes[1][i]], olist[indexes[3][j]]);
                if (approx(distance, dst_beacons_[5]))
                {
                    beacon_list3.push_back({beacon1, std::nullopt, olist[indexes[1][i]], olist[indexes[3][j]]});
                }
            }
        }

        ///
        /// B TEST
        ///
        // The list of AB, but test as if the current was B
        for (std::size_t i = 0; i < indexes[0].size(); i++)
        {
            std::vector<int> test_indexesB;
            // Comparing AB with BC to get the check for ABC combinations
            // In the same time, saving the positions of the interesting points to compare then
            // The list of BC, Checking the distance AC
            for (std::size_t j = 0; j < indexes[2].size(); j++)
            {
                double distance = dt(olist[indexes[0][i]], olist[indexes[2][j]]);
                if (approx(distance, dst_beacons_[1]))
                {
                    beacon_list3.push_back({olist[indexes[0][i]], beacon1, olist[indexes[2][j]], std::nullopt});
                    test_indexesB.push_back(indexes[2][j]);
                }
            }
            // Compare now the BD list combination, to see if there is a link between ABD
            for (std::size_t j = 0; j < indexes[4].size(); j++)
            {
                double distance = dt(olist[indexes[0][i]], olist[indexes[4][j]]);
                if (approx(distance, dst_beacons_[3]))
                {
                    beacon_list3.push_back({olist[indexes[0][i]], beacon1, std::nullopt, olist[indexes[4][j]]});
                    // We checked AB, BC, AC, BD, AD, need CD to have the 4 beacons now that we have in values to test
                    for (std::size_t k = 0; k < test_indexesB.size(); k++)
                    {
                        double distance = dt(olist[test_indexesB[k]], olist[indexes[4][j]]);
                        if (approx(distance, dst_beacons_[5]))
                        {
                            beacon_list4.push_back({olist[indexes[0][i]], beacon1, olist[test_indexesB[k]], olist[indexes[4][j]]});
                        }
                    }
                }
            }
        }
        // The list of BC
        for (std::size_t i = 0; i < indexes[2].size(); i++)
        {
            // The list of BD, Checking the distance CD
            for (std::size_t j = 0; j < indexes[4].size(); j++)
            {
                double distance = dt(olist[indexes[2][i]], olist[indexes[4][j]]);
                if (approx(distance, dst_beacons_[5]))
                {
                    beacon_list3.push_back({std::nullopt, beacon1, olist[indexes[2][i]], olist[indexes[4][j]]});
                }
            }
        }

        ///
        /// C TEST
        ///
        // The list of AC, but test as if the current was C
        for (std::size_t i = 0; i < indexes[1].size(); i++)
        {
            std::vector<int> test_indexesC;
            // Comparing AC with BC to get the check for ABC combinations
            // In the same time, saving the positions of the interesting points to compare then
            // The list of BC, Checking the distance AB
            for (std::size_t j = 0; j < indexes[2].size(); j++)
            {
                double distance = dt(olist[indexes[1][i]], olist[indexes[2][j]]);
                if (approx(distance, dst_beacons_[0]))
                {
                    beacon_list3.push_back({olist[indexes[1][i]], olist[indexes[2][j]], beacon1, std::nullopt});
                    test_indexesC.push_back(indexes[2][j]);
                }
            }
            // Compare now the CD list combination, to see if there is a link between ACD
            for (std::size_t j = 0; j < indexes[5].size(); j++)
            {
                double distance = dt(olist[indexes[1][i]], olist[indexes[5][j]]);
                if (approx(distance, dst_beacons_[3]))
                {
                    beacon_list3.push_back({olist[indexes[1][i]], std::nullopt, beacon1, olist[indexes[5][j]]});
                    // We checked AC, BC, AB, CD, AC, need BD to have the 4 beacons now that we have in values to test
                    for (std::size_t k = 0; k < test_indexesC.size(); k++)
                    {
                        double distance = dt(olist[test_indexesC[k]], olist[indexes[5][j]]);
                        if (approx(distance, dst_beacons_[4]))
                        {
                            beacon_list4.push_back({olist[indexes[1][i]], olist[test_indexesC[k]], beacon1, olist[indexes[5][j]]});
                        }
                    }
                }
            }
        }
        // The list of BC
        for (std::size_t i = 0; i < indexes[2].size(); i++)
        {
            // The list of CD, Checking the distance BD
            for (std::size_t j = 0; j < indexes[5].size(); j++)
            {
                double distance = dt(olist[indexes[2][i]], olist[indexes[5][j]]);
                if (approx(distance, dst_beacons_[4]))
                {
                    beacon_list3.push_back({std::nullopt, olist[indexes[2][i]], beacon1, olist[indexes[5][j]]});
                }
            }
        }

        ///
        /// D TEST
        ///
        // The list of AD, but test as if the current was D
        for (std::size_t i = 0; i < indexes[3].size(); i++)
        {
            std::vector<int> test_indexesD;
            // Comparing AD with BD to get the check for ABD combinations
            // In the same time, saving the positions of the interesting points to compare then
            // The list of BD, Checking the distance AB
            for (std::size_t j = 0; j < indexes[4].size(); j++)
            {
                double distance = dt(olist[indexes[3][i]], olist[indexes[4][j]]);
                if (approx(distance, dst_beacons_[0]))
                {
                    beacon_list3.push_back({olist[indexes[3][i]], olist[indexes[4][j]], std::nullopt, beacon1});
                    test_indexesD.push_back(indexes[4][j]);
                }
            }

            // The list of CD, Checking the distance AC
            for (std::size_t j = 0; j < indexes[5].size(); j++)
            {
                double distance = dt(olist[indexes[3][i]], olist[indexes[5][j]]);
                if (approx(distance, dst_beacons_[1]))
                {
                    beacon_list3.push_back({olist[indexes[3][i]], std::nullopt, olist[indexes[5][j]], beacon1});
                    // We checked AD, BD, AB, CD, AC, need BC to have the 4 beacons now that we have in values to test
                    for (std::size_t k = 0; k < test_indexesD.size(); k++)
                    {
                        double distance = dt(olist[test_indexesD[k]], olist[indexes[5][j]]);
                        if (approx(distance, dst_beacons_[2]))
                        {
                            beacon_list4.push_back({olist[indexes[3][i]], olist[test_indexesD[k]], olist[indexes[5][j]], beacon1});
                        }
                    }
                }
            }
        }
        // The list of BD
        for (std::size_t i = 0; i < indexes[4].size(); i++)
        {
            // The list of CD, Checking the distance BC
            for (std::size_t j = 0; j < indexes[5].size(); j++)
            {
                double distance = dt(olist[indexes[4][i]], olist[indexes[5][j]]);
                if (approx(distance, dst_beacons_[2]))
                {
                    beacon_list3.push_back({std::nullopt, olist[indexes[4][i]], olist[indexes[5][j]], beacon1});
                }
            }
        }
    }
    if (!beacon_list4.empty())
    {
        for (int i = beacon_list4.size() - 1; i>=0; i--)
        {
            std::array<Eigen::Vector2d, 3> signABC = {beacon_list4[i][0].value(), beacon_list4[i][1].value(), beacon_list4[i][2].value()};
            std::array<Eigen::Vector2d, 3> signABD = {beacon_list4[i][0].value(), beacon_list4[i][1].value(), beacon_list4[i][3].value()};
            std::array<Eigen::Vector2d, 3> signACD = {beacon_list4[i][0].value(), beacon_list4[i][2].value(), beacon_list4[i][3].value()};
            std::array<Eigen::Vector2d, 3> signBCD = {beacon_list4[i][1].value(), beacon_list4[i][2].value(), beacon_list4[i][3].value()};
            if (getAngleSign(signABC) != angle_sign_[0] || getAngleSign(signABD) != angle_sign_[1]
            || getAngleSign(signACD) != angle_sign_[2] || getAngleSign(signBCD) != angle_sign_[3])
            {
                beacon_list4.erase(beacon_list4.begin() + i);
            }
        }
        if (!beacon_list4.empty()) return {4, beacon_list4};
    }
    if (!beacon_list3.empty())
    {
        for (int i = beacon_list3.size() - 1; i >= 0; i--)
        {
            if (!beacon_list3[i][3].has_value())
            {
                std::array<Eigen::Vector2d, 3> signABC = {beacon_list3[i][0].value(), beacon_list3[i][1].value(), beacon_list3[i][2].value()};
                if (getAngleSign(signABC) != angle_sign_[0]) beacon_list3.erase(beacon_list3.begin() + i);
            }
            else if (!beacon_list3[i][2].has_value())
            {
                std::array<Eigen::Vector2d, 3> signABD = {beacon_list3[i][0].value(), beacon_list3[i][1].value(), beacon_list3[i][3].value()};
                if (getAngleSign(signABD) != angle_sign_[1]) beacon_list3.erase(beacon_list3.begin() + i);
            }
            else if (!beacon_list3[i][1].has_value())
            {
                std::array<Eigen::Vector2d, 3> signACD = {beacon_list3[i][0].value(), beacon_list3[i][2].value(), beacon_list3[i][3].value()};
                if (getAngleSign(signACD) != angle_sign_[2]) beacon_list3.erase(beacon_list3.begin() + i);
            }
            else if (!beacon_list3[i][0].has_value())
            {
                std::array<Eigen::Vector2d, 3> signBCD = {beacon_list3[i][1].value(), beacon_list3[i][2].value(), beacon_list3[i][3].value()};
                if (getAngleSign(signBCD) != angle_sign_[3]) beacon_list3.erase(beacon_list3.begin() + i);
            }
        }
        if (!beacon_list3.empty()) return {3, beacon_list3};
    }
    if (!beacon_list2.empty()) return {2, beacon_list2};

    return {0, {}};
}
