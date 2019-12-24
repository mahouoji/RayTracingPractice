#pragma once

#include <Eigen/Core>

class Ray {
 public:
    Ray();
    Ray(const Eigen::Vector3f& origin_, const Eigen::Vector3f& direction_, 
      float t_min, float t_max);
    Eigen::Vector3f get_ori() const;
    Eigen::Vector3f get_dir() const;
    Eigen::Vector3f get_point_at(float t) const;
    float get_tmin() const;
    float get_tmax() const;
    Eigen::Vector3f get_min_point() const; // get point at tmin
    // check if given t in [t_min, t_max]
    bool covers(float t) const;
 private:
    Eigen::Vector3f origin_;
    Eigen::Vector3f direction_;
    float t_min_, t_max_;
};