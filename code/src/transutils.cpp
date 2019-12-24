#include "transutils.h"
#include <cmath>
#include <Eigen/Dense>
using namespace Eigen;

namespace TransUtils{

Vector2f transform_2D(const Matrix4f& trans_matrix,
    const Vector3f& vec, int is_pos) {
    Vector4f hom_res = trans_matrix
            * Vector4f(vec(0), vec(1), 0, float(is_pos));
    if (hom_res(3) != 0) {
        hom_res /= hom_res(3);
    }
    return Vector2f(hom_res(0), hom_res(1));
}

Vector3f transform(const Matrix4f& trans_matrix,
    const Vector3f& vec, int is_pos) {
    Vector4f hom_res = trans_matrix
            * Vector4f(vec(0), vec(1), vec(2), float(is_pos));
    if (hom_res(3) != 0) {
        hom_res /= hom_res(3);
    }
    return Vector3f(hom_res(0), hom_res(1), hom_res(2));
}

Eigen::Matrix4f get_normal_matrix(const Matrix4f& trans_matrix) {
    return trans_matrix.inverse().transpose();
}

// get transform matrix for rotation along x-axis
Matrix4f rotation_x(float a) {
    float s = sin(a);
    float_t c = cos(a);
    Matrix4f m;
    m << 1, 0, 0, 0,
         0, c, -s, 0,
         0, s, c, 0,
         0, 0, 0, 1;
    return m;
}

// get transform matrix for rotation along y-axis
Matrix4f rotation_y(float a) {
    float s = sin(a);
    float c = cos(a);
    Matrix4f m;
    m << c, 0, s, 0,
         0, 1, 0, 0,
         -s, 0, c, 0,
         0, 0, 0, 1;
    return m;
}

// get transform matrix for rotation along z-axis
Matrix4f rotation_z(float a) {
    float s = sin(a);
    float c = cos(a);
    Matrix4f m;
    m << c, -s, 0, 0,
         s, c, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
    return m;
}

// translation with Vec3(tx, ty, tz)
Matrix4f translation(float tx, float ty, float tz) {
    Matrix4f m;
    m << 1, 0, 0, tx,
         0, 1, 0, ty,
         0, 0, 1, tz,
         0, 0, 0, 1;
    return m;
}

Matrix4f translation(Eigen::Vector3f trans) {
    Matrix4f m;
    m << 1, 0, 0, trans(0),
         0, 1, 0, trans(1),
         0, 0, 1, trans(2),
         0, 0, 0, 1;
    return m;
}

// scale with s
Matrix4f scale(float s) {
    Matrix4f m;
    m << s, 0, 0, 0,
         0, s, 0, 0,
         0, 0, s, 0,
         0, 0, 0, 1;
    return m;
}

// scale with s, xy plane
Matrix4f scale2D(float s) {
    Matrix4f m;
    m << s, 0, 0, 0,
         0, s, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
    return m;
}

// reflect a ray at normal
Vector3f reflect(const Vector3f& in, const Vector3f& norm) {
    return (in - 2 * in.dot(norm) * norm).normalized();
}

};