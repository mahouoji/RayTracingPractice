#pragma once

#include <Eigen/Core>
#include "ray.h"
#include "hitable.h"
#include "transutils.h"

struct ScatterRecord {
    Ray specular_ray_;
    bool is_specular_;
    Eigen::Vector3f attenuation;
};

class Material {
 public:
    virtual bool scatter(const Ray& ray, const HitRecord& rec, ScatterRecord* srec) const;
    virtual Eigen::Vector3f emit() const;
    //static Eigen::Vector3f gen_unit_sphere();
};

class Lambertian : public Material {
 public:
    Lambertian(const Eigen::Vector3f& albedo);
    virtual bool scatter(const Ray& ray, const HitRecord& rec, ScatterRecord* srec) const;

    Eigen::Vector3f albedo_;
};

class Metal : public Material {
 public:
    Metal(const Eigen::Vector3f& albedo, float fuzz);
    virtual bool scatter(const Ray& ray, const HitRecord& rec, ScatterRecord* srec) const;

    Eigen::Vector3f albedo_;
    float fuzz_;
};

class DiffuseLight : public Material {
 public:
    DiffuseLight(const Eigen::Vector3f& e);
    virtual bool scatter(const Ray& ray, const HitRecord& rec, ScatterRecord* srec) const;
    virtual Eigen::Vector3f emit() const;

    Eigen::Vector3f emit_color_;
};