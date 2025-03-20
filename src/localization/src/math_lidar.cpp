#include "localization/math_lidar.hpp"

Eigen::Vector2d unit_vector(const Eigen::Vector2d& v) {
    if (v.norm() == 0.0) {
        throw std::invalid_argument("Zero vector has no unit vector.");
    }
    return v / v.norm();
}

void removearray(std::vector<Eigen::Vector2d>& L, const Eigen::Vector2d& arr) {
    auto it = std::find_if(L.begin(), L.end(), [&](const Eigen::Vector2d& v) {
        return v.isApprox(arr);
    });
    if (it != L.end()) {
        L.erase(it);
    } else {
        throw std::runtime_error("Array not found in list.");
    }
}

std::pair<double, double> angle_between(const Eigen::Vector2d& vector1, const Eigen::Vector2d& vector2) {
    Eigen::Vector2d v1_u = unit_vector(vector1);
    Eigen::Vector2d v2_u = unit_vector(vector2);
    double angle = std::acos(std::clamp(v1_u.dot(v2_u), -1.0, 1.0));
    double minor = v1_u.x() * v2_u.y() - v1_u.y() * v2_u.x();
    return { (minor == 0 ? angle : std::copysign(angle, minor)), angle };
}

double dt2(const Eigen::Vector2d& ptA, const Eigen::Vector2d& ptB) {
    return (ptA - ptB).squaredNorm();
}

double dt(const Eigen::Vector2d& ptA, const Eigen::Vector2d& ptB) {
    return (ptA - ptB).norm();
}

bool approx(double distanceA, double distanceB, double precision) {
    return std::abs(distanceA - distanceB) < precision;
}

std::pair<double, double> cartesian_to_polar(double x, double y) {
    return { std::sqrt(x * x + y * y), std::abs(std::atan2(y, x)) };
}

bool angle_in_range(double alpha, double low, double up) {
    if (low > up) {
        throw std::invalid_argument("Low bound must be less than or equal to upper bound.");
    }
    alpha = std::fmod(alpha, 2 * M_PI);
    low = std::fmod(low, 2 * M_PI);
    up = std::fmod(up, 2 * M_PI);
    return (low <= alpha && alpha <= up) || (low > up && (low <= alpha || alpha <= up));
}

Eigen::Vector3d convert_world_to_robot(const Eigen::Vector2d& pt, const Eigen::Vector3d& robot_frame) {
    double cos_angle = std::cos(robot_frame.z());
    double sin_angle = std::sin(robot_frame.z());

    Eigen::Matrix3d OtoR;
    OtoR << cos_angle, -sin_angle, robot_frame.x(),
            sin_angle, cos_angle, robot_frame.y(),
            0, 0, 1;

    return OtoR.inverse() * Eigen::Vector3d(pt.x(), pt.y(), 1);
}

Eigen::Vector3d convert_robot_to_world(const Eigen::Vector2d& pt, const Eigen::Vector3d& robot_frame) {
    double cos_angle = std::cos(robot_frame.z());
    double sin_angle = std::sin(robot_frame.z());

    Eigen::Matrix3d OtoR;
    OtoR << cos_angle, -sin_angle, robot_frame.x(),
            sin_angle, cos_angle, robot_frame.y(),
            0, 0, 1;

    return OtoR * Eigen::Vector3d(pt.x(), pt.y(), 1);
}

double _compute_error(const std::array<std::optional<Eigen::Vector2d>, 4>& beacons,
    const Eigen::Vector3d& position,
    const std::array<Eigen::Vector2d, 4>& fixed_beacons)
{
double sum = 0.0;
// Pour chaque paire (beacon, fixed_beacon)
for (size_t i = 0; i < beacons.size(); ++i) {
if (beacons[i].has_value()) {
// Transformation du beacon du repère robot vers le repère plateau
Eigen::Vector2d transformed = convert_robot_to_world(beacons[i].value(), position).head<2>();
// Accumulation de l'erreur (distance au carré entre la position transformée et le beacon fixe)
sum += dt2(transformed, fixed_beacons[i]);
}
}
return sum;
}

Eigen::Vector3d _find_position_opti(
    const std::array<std::optional<Eigen::Vector2d>, 4>& beacons,
    const std::array<Eigen::Vector2d, 4>& fixed_beacons)
{
    std::vector<double> BX;
    std::vector<double> BY;
    std::vector<Eigen::Vector4d> rows;  // Chaque paire de lignes correspond aux équations associées à un beacon

    // Pour chaque beacon non nul, accumuler les équations
    for (size_t i = 0; i < 4; ++i) {
        if (beacons[i].has_value()) {
            // Récupération des coordonnées fixes
            double fx = fixed_beacons[i](0);
            double fy = fixed_beacons[i](1);
            BX.push_back(fx);
            BY.push_back(fy);

            // Coordonnées mesurées du beacon
            const Eigen::Vector2d& b = beacons[i].value();

            // Construction de la première ligne: [1, 0, b[0], -b[1]]
            Eigen::Vector4d row_ax;
            row_ax << 1, 0, b(0), -b(1);
            // Construction de la deuxième ligne: [0, 1, b[1], b[0]]
            Eigen::Vector4d row_ay;
            row_ay << 0, 1, b(1), b(0);

            rows.push_back(row_ax);
            rows.push_back(row_ay);
        }
    }

    int n = BX.size();       // Nombre de beacons valides
    int m = 2 * n;           // Nombre total d'équations
    if(n == 0) {
        return -Eigen::Vector3d::Ones();
    }

    // Construction de la matrice A (m x 4) et du vecteur B (m x 1)
    Eigen::MatrixXd A(m, 4);
    Eigen::VectorXd B(m);
    for (int i = 0; i < m; ++i) {
        A.row(i) = rows[i].transpose();
    }
    // Les n premières lignes de B correspondent aux BX et les n suivantes aux BY
    for (int i = 0; i < n; ++i) {
        B(i) = BX[i];
    }
    for (int i = 0; i < n; ++i) {
        B(i + n) = BY[i];
    }

    // Résolution des équations normales : (AᵀA)X = AᵀB
    Eigen::MatrixXd AtA = A.transpose() * A;
    if (std::abs(AtA.determinant()) < 1e-6) {
        return -Eigen::Vector3d::Ones();
    }
    Eigen::Vector4d X = AtA.ldlt().solve(A.transpose() * B);

    // Calcul de theta1 selon la logique du code Python
    double theta1 = 0.0;
    if (std::abs(X(2)) < 1) {
        if (X(3) > 0)
            theta1 = std::acos(X(2));
        else
            theta1 = -std::acos(X(2));
    }
    else if (std::abs(X(3)) < 1) {
        if (X(2) < 0)
            theta1 = M_PI - std::asin(X(3));
        else
            theta1 = std::asin(X(3));
    }
    else {
        if (X(2) < -1)
            theta1 = M_PI - std::atan(X(3) / X(2));
        else
            theta1 = std::atan(X(3) / X(2));
    }
    // Mise à l'échelle de theta1 dans l'intervalle [0, 2π)
    theta1 = std::fmod(theta1, 2 * M_PI);
    if (theta1 < 0)
        theta1 += 2 * M_PI;

    return Eigen::Vector3d(X(0), X(1), theta1);
}

Eigen::Vector3d _find_position_eq(
    const std::array<std::optional<Eigen::Vector2d>, 4>& beacons,
    const std::array<Eigen::Vector2d, 4>& fixed_beacons)
{
    // Sélectionner les trois premiers beacons valides
    std::vector<Eigen::Vector2d> vfb;  // Coordonnées fixes des beacons valides
    std::vector<double> dvb;           // dt2 appliqué aux mesures (beacons)
    for (size_t i = 0; i < 4 && vfb.size() < 3; ++i) {
        if (beacons[i].has_value()) {
            vfb.push_back(fixed_beacons[i]);             // On garde les coordonnées fixes
            dvb.push_back(dt2(beacons[i].value()));        // dt2 sur le beacon mesuré
        }
    }
    if (vfb.size() < 3) {
        return -Eigen::Vector3d::Ones();
    }

    // Calculer dt2 pour chacun des points fixes sélectionnés
    std::vector<double> dvfb;
    for (const auto &v : vfb) {
        dvfb.push_back(dt2(v));
    }

    // Calculer delta = dvb - dvfb pour les trois premiers éléments
    std::array<double, 3> delta;
    for (size_t i = 0; i < 3; ++i) {
        delta[i] = dvb[i] - dvfb[i];
    }

    double coeff = 0.0, const_val = 0.0, y = 0.0;
    // On teste la différence des coordonnées x entre les deux premiers beacons
    if (std::abs(vfb[0](0) - vfb[1](0)) > 0.1) {
        coeff     = (vfb[1](1) - vfb[0](1)) / (vfb[0](0) - vfb[1](0));
        const_val = (delta[1] - delta[0]) / (2 * (vfb[0](0) - vfb[1](0)));
        y         = (delta[2] - delta[0] - 2 * const_val * (vfb[0](0) - vfb[2](0))) /
                    (2 * (coeff * (vfb[0](0) - vfb[2](0)) + (vfb[0](1) - vfb[2](1))));
    }
    // Sinon, on teste avec le premier et le troisième beacon
    else if (std::abs(vfb[0](0) - vfb[2](0)) > 0.1) {
        coeff     = (vfb[2](1) - vfb[0](1)) / (vfb[0](0) - vfb[2](0));
        const_val = (delta[2] - delta[0]) / (2 * (vfb[0](0) - vfb[2](0)));
        y         = (delta[1] - delta[0] - 2 * const_val * (vfb[0](0) - vfb[1](0))) /
                    (2 * (coeff * (vfb[0](0) - vfb[1](0)) + (vfb[0](1) - vfb[1](1))));
    }
    else {
        // Si aucune des conditions n'est satisfaite, on retourne une erreur.
        return -Eigen::Vector3d::Ones();
    }
    double x = const_val + y * coeff;

    // Calcul de l'angle à partir du point (x, y)
    Eigen::Vector2d pos(x, y);
    std::array<std::optional<Eigen::Vector2d>, 4> b = beacons;
    std::array<Eigen::Vector2d, 4> fb = fixed_beacons;
    double theta = find_angle(pos, b, fb);

    return Eigen::Vector3d(x, y, theta);
}

double find_angle(
    const Eigen::Vector2d& pos,
    const std::array<std::optional<Eigen::Vector2d>, 4>& beacons,
    const std::array<Eigen::Vector2d, 4>& fixed_beacons)
{
    std::vector<double> angle_tot;
    std::vector<double> min_lists;

    // Parcours des 4 beacons
    for (size_t i = 0; i < 4; ++i) {
        if (beacons[i].has_value()) {
            // v1 correspond aux coordonnées du beacon mesuré (déjà en 2D)
            Eigen::Vector2d v1 = beacons[i].value();
            // v2 est le vecteur allant du point pos aux coordonnées fixes du beacon
            Eigen::Vector2d v2 = fixed_beacons[i] - pos;

            // Appel à angle_between qui renvoie {angle, min_test}
            auto [angle_temp_1, min_test] = angle_between(v1, v2);
            angle_tot.push_back(angle_temp_1);
            min_lists.push_back(min_test);
        }
    }

    // On sélectionne l'indice du beacon dont la valeur min_test est la plus petite
    size_t best_index = 0;
    double best_min = min_lists[0];
    for (size_t i = 1; i < min_lists.size(); ++i) {
        if (min_lists[i] < best_min) {
            best_index = i;
            best_min = min_lists[i];
        }
    }

    return angle_tot[best_index];
}

std::vector<std::pair<Eigen::Vector3d, double>> find_position(
    const std::vector<std::array<std::optional<Eigen::Vector2d>, 4>>& beacons_list,
    const std::array<Eigen::Vector2d, 4>& fixed_beacons,
    int nombre,
    const std::array<double, 4>& boundaries)
{
    std::vector<std::pair<Eigen::Vector3d, double>> results;

    // Parcours de la liste de beacons
    for (const auto& beacons : beacons_list) {
        std::vector<double> err;  // Stockage intermédiaire des erreurs (non retourné)

        // 1. Calcul initial de la position optimisée et de l'erreur associée
        Eigen::Vector3d position_found = _find_position_opti(beacons, fixed_beacons);
        double dt_min = _compute_error(beacons, position_found, fixed_beacons);
        err.push_back(dt_min);

        // 2. Calcul de l'angle et mise à jour de la position
        Eigen::Vector3d position = position_found;  // copie de la position trouvée
        Eigen::Vector2d xyPos = position.head<2>();
        position(2) = find_angle(xyPos, beacons, fixed_beacons);
        double dt = _compute_error(beacons, position, fixed_beacons);
        if (dt < dt_min) {
            position_found(2) = position(2);
            dt_min = dt;
        }
        err.push_back(dt);

        // 3. Si le nombre de beacons est supérieur à 2, on essaie une autre méthode
        if (nombre > 2) {
            position = _find_position_eq(beacons, fixed_beacons);
            dt = _compute_error(beacons, position, fixed_beacons);
            err.push_back(dt);
            if (dt < dt_min) {
                position_found = position;
                dt_min = dt;
            }
        }

        // 4. Vérification des limites (boundaries)
        if (position_found(0) > boundaries[0] - 0.1 &&
            position_found(0) < boundaries[1] + 0.1 &&
            position_found(1) > boundaries[2] - 0.1 &&
            position_found(1) < boundaries[3] + 0.1)
        {
            results.emplace_back(position_found, dt_min);
        }
    }

    return results;
}

bool getAngleSign(const std::array<Eigen::Vector2d, 3>& points) {
    double cross = (points[1].x() - points[0].x()) * (points[2].y() - points[1].y())
        - (points[1].y() - points[0].y()) * (points[2].x() - points[1].x());
    return (cross > 0);
}
