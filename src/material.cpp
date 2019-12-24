#include "material.h"
#include "randomutils.h"

using namespace Eigen;

Vector3f Material::gen_unit_sphere() {
    Vector3f p;
    p = Vector3f::Random();
    if (p.norm() >= 1.0) {
        p = p.normalized();
        float s = float(rand() % 1000) / 1000.0;
        p *= s;
    }
    return p;
}

bool Material::scatter(const Ray& ray, const HitRecord& rec, ScatterRecord* srec) const {
    return false;
}

Vector3f Material::emit() const {
    return Vector3f::Zero();
}

Lambertian::Lambertian(const Eigen::Vector3f& albedo) : albedo_(albedo) {}
bool Lambertian::scatter(const Ray& ray, const HitRecord& rec, ScatterRecord* srec) const {
    Vector3f target = rec.position_ + rec.normal_ + Material::gen_unit_sphere();
    srec->specular_ray_ = Ray(rec.position_, target - rec.position_, 1.0e-6, ray.get_tmax());
    srec->attenuation = albedo_;
    return true;
}

Metal::Metal(const Eigen::Vector3f& albedo, float fuzz) : albedo_(albedo), fuzz_(fuzz) {}
bool Metal::scatter(const Ray& ray, const HitRecord& rec, ScatterRecord* srec) const {
    Vector3f reflected = TransUtils::reflect(ray.get_dir(), rec.normal_);
    srec->specular_ray_ = Ray(rec.position_, reflected + fuzz_ * Material::gen_unit_sphere(), 1.0e-6, ray.get_tmax());
    srec->attenuation = albedo_;
    return rec.normal_.dot(srec->specular_ray_.get_dir()) > 0;
}

DiffuseLight::DiffuseLight(const Vector3f& e) : emit_color_(e) {}
bool DiffuseLight::scatter(const Ray& ray, const HitRecord& rec, ScatterRecord* srec) const {
    return false;
}
Vector3f DiffuseLight::emit() const {
    return emit_color_;
}