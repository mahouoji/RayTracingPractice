
#include "camera.h"

#include "transutils.h"
#include "randomutils.h"

using namespace Eigen;

// Given the coordinate (x, y) on image pane, generate the ray
Camera::Camera(int image_width, int image_height, 
        float scale, float z_near, float z_far,
        float focus, float aperture) :
        image_width_(image_width), image_height_(image_height),
        fov_scale_(scale), z_near_(z_near), z_far_(z_far),
        z_focus_(focus), aperture_(aperture) {
    camera_to_world_ = Matrix4f::Identity();
}

void Camera::config(int image_width, int image_height, 
        float scale, float z_near, float z_far,
        float focus, float aperture) {
    image_width_ = image_width;
    image_height_ = image_height;
    fov_scale_ = scale;
    z_near_ = z_near;
    z_far_ = z_far;
    z_focus_ = focus;
    aperture_ = aperture;
}

void Camera::config_focus(float focus, float aperture) {
    z_focus_ = focus;
    aperture_ = aperture;
}

Ray Camera::get_ray_perspective(float x, float y) {
    float px = (2 * x - image_width_) / image_height_ * fov_scale_;
    float py = (1 - 2 * y / image_height_) * fov_scale_ ;
    Vector3f dir = TransUtils::transform(camera_to_world_, Vector3f(px, py, -1), 0);
    Vector3f pos = TransUtils::transform(camera_to_world_, Vector3f(0, 0, 0), 1);
    float len = dir.norm();
    return Ray(pos, dir, len * z_near_, len * z_far_);
}

Ray Camera::get_ray_perspective_focus(float x, float y) {
    // Get the size of the window
    int width = image_width_;
    int height = image_height_;
    float aspect_ratio_ = float(height)/float(width);
    // Convert screen position to world coordinates
    float px = (2 * x / width - 1) * z_focus_ * fov_scale_;
    float py = (1 - 2 * y / height) * z_focus_ * fov_scale_ * aspect_ratio_;

    Vector3f foc = TransUtils::transform(camera_to_world_, Vector3f(px, py, -z_focus_), 1);
    Vector2f rnd2 = RandomUtils::unit_in_circle();
    Vector3f rnd = Vector3f(rnd2(0), rnd2(1), 0);
    Vector3f ori = TransUtils::transform(camera_to_world_, Vector3f(0, 0, 0) + rnd * aperture_, 1);
    Vector3f dir= foc - ori;
    float len = dir.norm();
    return Ray(ori, dir, len * z_near_ / z_focus_, len * z_far_ / z_focus_);
    //return Ray(ori, dir, len, len * z_far_ / z_near_);
}

void Camera::look_at(Vector3f from, Vector3f to, Vector3f rand) {
    Vector3f z = (from - to).normalized();
    Vector3f x = rand.cross(z).normalized();
    Vector3f y = z.cross(x);
    
    camera_to_world_ << x(0), y(0), z(0), from(0),
           x(1), y(1), z(1), from(1),
           x(2), y(2), z(2), from(2),
           0, 0, 0, 1;
}