#pragma once

#include <string>
#include <vector>
#include <Eigen/Core>
#include "ray.h"

class Material;

// hitting test by ray casting
struct HitRecord {
    float t_;
    Eigen::Vector3f position_;
    Eigen::Vector3f normal_;
    Material* materialp_;
};

struct RandRecord {
   Eigen::Vector3f position_;
   Ray ray_to_rand_;
};

class Hitable {
 public:
    virtual bool hit(const Ray& ray, HitRecord* record) const = 0;
    virtual bool rand_pos(const Eigen::Vector3f& p_from, RandRecord* record) const;
};

class HitableList : public Hitable{
 public:
    virtual bool hit(const Ray& ray, HitRecord* record) const ;
    void add(Hitable* hit);
    std::vector<Hitable*> hitables_;
};

class Sphere : public Hitable {
 public:
    Sphere();
    Sphere(const Eigen::Vector3f& center, float radius, Material* mp);
    virtual bool hit(const Ray& ray, HitRecord* rec) const;
    bool easy_hit(const Ray&ray) const;
    virtual bool rand_pos(const Eigen::Vector3f& p_from, RandRecord* record) const;
 private:
    Eigen::Vector3f center_;
    float radius_;
    Material* materialp_;
};

// triangle
class Triangle : public Hitable {
 public:
    Triangle();
    Triangle(const Eigen::Vector3f& v0, const Eigen::Vector3f& v1,
         const Eigen::Vector3f& v2, Material* mp);
    virtual bool hit(const Ray& ray, HitRecord* rec) const;
 private:
    Eigen::Vector3f vertex0_;
    Eigen::Vector3f vertex1_;
    Eigen::Vector3f vertex2_;
    Material* materialp_;
};

// plane
class Plane : public Hitable {
 public:
    Plane();
    Plane(const Eigen::Vector3f& point, const Eigen::Vector3f& normal, Material* mp);
    virtual bool hit(const Ray& ray, HitRecord* rec) const;
 private:
    Eigen::Vector3f point_;
    Eigen::Vector3f normal_;
    Material* materialp_;
};

// mesh object
class MeshPrimitive {
 public:
    MeshPrimitive();
    bool load_off(const std::string& filename);
    inline float max4(float a, float b, float c, float d);
    Eigen::MatrixXf vertices_;
    std::vector<Eigen::Vector3i> faces_;
};

struct TransRec {
    TransRec();
    Eigen::Matrix4f compute_matrix();
    Eigen::Matrix4f compute_with_trans(Eigen::Vector3f trans);
    float scale_;
    float rotation_[3]; // along x, y, z
    Eigen::Vector3f translation_;
};

class MeshObject : public Hitable {
 public:
    MeshObject(MeshPrimitive* p, Material* mp);
    virtual bool hit(const Ray& ray, HitRecord* record) const;
    // transformations
    void translate(const Eigen::Vector3f& t);
    void rotate(float a, int axis);
    void scale(float s);
    void update_trans_matrix();
    void update_trans_vertices();
 private:
    TransRec trans_rec_;
    bool trans_updated_;
    std::vector<Eigen::Vector3f> trans_vertices_;
    Eigen::Matrix4f trans_matrix_;
    MeshPrimitive* primitivep_;
    Material* materialp_;
};

class Rect : public Hitable {
 public:
   Rect();
   Rect(Eigen::Vector2f lowerleft, Eigen::Vector2f upperright, float k, Material* mp, int axis, bool flip=false);
   virtual bool hit(const Ray& ray, HitRecord* record) const;
 private:
   int norm_axis_;
   int plane_axis_[2];
   Eigen::Vector2f lowerleft_, upperright_;
   float k_;
   Material* materialp_;
   Eigen::Vector3f normal_;
};