#pragma once

#include <Eigen/Dense>
#include "ray.h"

class Camera {
 public:
    Camera(int image_width, int image_height,
        float scale, float z_near, float z_far,
        float focus, float aperture);
    Ray get_ray_perspective(float x, float y);
    Ray get_ray_perspective_focus(float x, float y);
    void look_at(Eigen::Vector3f from, Eigen::Vector3f to,
            Eigen::Vector3f rand = Eigen::Vector3f(0, 1, 0));
    void config_focus(float focus, float aperture);

        // trasformation matrix for camera
    Eigen::Matrix4f camera_to_world_;
 private:
    // image size in pixel
    int image_width_, image_height_;
    // attributes for camera
    float fov_scale_; // tan(FOV/2)
    float z_near_, z_far_;
    float z_focus_, aperture_;
};