#pragma once

#include <Eigen/Core>

namespace RandomUtils {
    float random_float();
    Eigen::Vector2f unit_in_circle();
    Eigen::Vector3f unit_on_sphere();
};