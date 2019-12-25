#pragma once

#include <Eigen/Core>

namespace TransUtils {

Eigen::Vector2f transform_2D(const Eigen::Matrix4f& trans_matrix,
    const Eigen::Vector3f& vec, int is_pos = 1);

Eigen::Vector3f transform(const Eigen::Matrix4f& trans_matrix,
    const Eigen::Vector3f& vec, int is_pos = 1);

Eigen::Matrix4f get_normal_matrix(const Eigen::Matrix4f& trans_matrix);

Eigen::Matrix4f rotation_x(float a);

Eigen::Matrix4f rotation_y(float a);

Eigen::Matrix4f rotation_z(float a);

Eigen::Matrix4f translation(float tx, float ty, float tz);
Eigen::Matrix4f translation(Eigen::Vector3f trans);

Eigen::Matrix4f scale(float s);
Eigen::Matrix4f scale2D(float s);
Eigen::Matrix4f scale(const Eigen::Vector3f& s);

Eigen::Vector3f reflect(const Eigen::Vector3f& in,
        const Eigen::Vector3f& nrom);
};