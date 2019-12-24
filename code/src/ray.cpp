#include "ray.h"

using namespace Eigen;

Ray::Ray() : origin_(Vector3f(0, 0, 0)), direction_(Vector3f(0, 0, -1)),
        t_min_(0), t_max_(100) {}

Ray::Ray(const Vector3f& origin, const Vector3f& direction,
        float tmin, float tmax) : origin_(origin),
        direction_(direction.normalized()), t_min_(tmin), t_max_(tmax) {}

Vector3f Ray::get_ori() const { return origin_; }
Vector3f Ray::get_dir() const { return direction_; }
Vector3f Ray::get_point_at(float t) const { return origin_ + t * direction_; }
Vector3f Ray::get_min_point() const { return origin_ + t_min_ * direction_; }
float Ray::get_tmin() const { return t_min_; }
float Ray::get_tmax() const { return t_max_; }
bool Ray::covers(float t) const { return t >= t_min_ && t <= t_max_; }