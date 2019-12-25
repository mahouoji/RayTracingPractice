#include "hitable.h"
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include "transutils.h"
#include "randomutils.h"

using namespace std;
using namespace Eigen;

// hitable
bool Hitable::rand_pos(const Eigen::Vector3f& p_from, RandRecord* rec) const {
    return false;
}

// hitable list
bool HitableList::hit(const Ray& ray, HitRecord* record) const {
    HitRecord tmp_rec;
    bool found = false;
    float t_front = ray.get_tmax();
    Vector3f normal_front;
    Material* mp_front;
    for (auto phitable : hitables_) {
        if(phitable->hit(ray, &tmp_rec)
                && tmp_rec.t_ < t_front) {
            found = true;
            t_front = tmp_rec.t_;
            normal_front = tmp_rec.normal_;
            mp_front = tmp_rec.materialp_;
        }
    }
    if (found) {
        record->t_ = t_front;
        record->position_ = ray.get_point_at(t_front);
        record->normal_ = normal_front;
        record->materialp_ = mp_front;
        return true;
    }
    return false;
}

void HitableList::add(Hitable* hitable) {
    hitables_.push_back(hitable);
}

// Sphere
Sphere::Sphere() {}

Sphere::Sphere(const Vector3f& center, float radius, Material* mp)
        : center_(center), radius_(radius), materialp_(mp) {}

bool Sphere::easy_hit(const Ray&ray) const {
    Vector3f p0 = ray.get_min_point();
    Vector3f p1 = ray.get_point_at(ray.get_tmax());

    Vector3f v = ray.get_dir();
    Vector3f w = center_ - ray.get_ori();
    float dist = v.cross(w).squaredNorm();
    return radius_ * radius_ >= dist;
}

bool Sphere::rand_pos(const Eigen::Vector3f& p_from, RandRecord* rec) const {
    Vector3f dir = center_ - p_from;
    if (dir.norm() <= radius_) { return false; }
    Vector3f rnd = RandomUtils::unit_on_sphere();
    rec->position_ = center_ + rnd * radius_;
    Vector3f ldir = rec->position_ - p_from;
    rec->ray_to_rand_ = Ray(p_from, ldir, 0.01, 100);//ldir.norm());
    return true;
}

bool Sphere::hit(const Ray& ray, HitRecord* record) const {
    Vector3f dir = ray.get_dir();
    Vector3f oc = ray.get_ori() - center_;
    float a = dir.squaredNorm();
    float b = dir.dot(oc); // this is accually B / 2
    float c = oc.squaredNorm() - radius_ * radius_;
    float discriminant = b * b - a * c;
    bool hit = false;
    if (discriminant > 0) {
        float t = (-b - sqrt(discriminant)) / a;
        if (ray.covers(t)) {
            record->t_ = t;
            record->position_ = ray.get_point_at(t);
            record->normal_ =  (record->position_ - center_).normalized();
            record->materialp_ = materialp_;
            return true;
        }
        // if the nearer point not in fruntrum, try next one
        t = (-b + sqrt(discriminant)) / a;
        if (ray.covers(t)) {
            record->t_ = t;
            record->position_ = ray.get_point_at(t);
            record->normal_ =  (record->position_ - center_).normalized();
            record->materialp_ = materialp_;
            return true;
        }
    }
    return false;
}

// Triangle
Triangle::Triangle() {}
Triangle::Triangle(const Vector3f& v0, const Vector3f& v1,
        const Vector3f& v2, Material* mp) :
        vertex0_(v0), vertex1_(v1), vertex2_(v2), materialp_(mp) {}

bool Triangle::hit(const Ray& ray, HitRecord* record) const {
    float u, v, t;
    Vector3f ab = vertex1_ - vertex0_;
    Vector3f ac = vertex2_ - vertex0_;
    Vector3f ae = ray.get_ori() - vertex0_;

    Vector3f tmp = ray.get_dir().cross(ac);
    float det = ab.dot(tmp); // det[ab, ac, dir]
    //ray parallel to triangle
    if (abs(det) < 1.0e-6) { return false; }

    u = ae.dot(tmp) / det; // det[ae, ac, dir]
    if (u < 0 || u > 1) { return false; }

    v = ray.get_dir().dot(ae.cross(ab)) / det; // det[ab ae dir]
    if (v < 0 || u + v > 1) { return false; }

    tmp = ab.cross(ac);
    t = ae.dot(tmp) / det; // det[ab ac ae]
    if (ray.covers(t)) {
        record->t_ = t;
        record->position_ = ray.get_point_at(t);
        record->normal_ = tmp.normalized();
        record->materialp_ = materialp_;
        return true;
    }
    return false;
}


// Plane
Plane::Plane() {}
Plane::Plane(const Vector3f& p, const Vector3f& n, Material* mp)
        : point_(p), normal_(n.normalized()), materialp_(mp) {}

bool Plane::hit(const Ray& ray, HitRecord* rec) const {
    float v = ray.get_dir().dot(normal_);
    if (abs(v) < 1.0e-6) { return false; }

    float t = (point_ - ray.get_ori()).dot(normal_) / v;
    if (ray.covers(t)) {
        rec->t_ = t;
        rec->position_ = ray.get_point_at(t);
        rec->normal_ = normal_;
        rec->materialp_ = materialp_;
        return true;
    }
    return false;
}

// Mesh Object
MeshPrimitive::MeshPrimitive() {}

inline float MeshPrimitive::max4(float a, float b, float c, float d) {
    a = a > b ? a : b;
    a = a > c ? a : c;
    a = a > d ? a : d;
    return a;
}

bool MeshPrimitive::load_off(const std::string& filename) {
    string line;
    ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open off file " << filename << std::endl;
        return false;
    }
    file >> line;
    if (line.compare("OFF") != 0) {
        std::cerr << "Not a off file" << std::endl;
        file.close();
        return false;
    }
    //number of vertex, faces, edges
    int nv, nf, ne;
    file >> nv >> nf >> ne;
    if (nv <= 0 || nf <= 0) {
        std::cerr << "Nor vertex or no mesh" << std::endl;
        file.close();
        return false;
    }

    float x, y, z, max_t = 0;
    Vector3f center = Vector3f::Zero();
    vertices_.resize(3, nv);
    for (int i = 0; i < nv; i++) {
        file >> x >> y >> z;
        vertices_.col(i) << x, y, z;
        center += vertices_.col(i); // sum up
    }
    int n, d0, d1, d2;
    faces_.clear();
    faces_.reserve(nf);
    for (int i = 0; i < nf; i++) {
        file >> n >> d0 >> d1 >> d2;
        if (n != 3) {
            std::cerr << "Sorry, only triangle meshes are supported now" << std::endl;
            file.close();
            return false;
        }
        faces_.push_back(Vector3i(d0, d1, d2));
    }
    file.close();

    // process loaded data
    // 1. put into unit cube
    center /= nv; // calculate barycenter
    for (int i = 0; i < nv; i++) {
        vertices_.col(i) -= center; // 1.1 move center to origin
        // compute maximum coordinates
        float x = abs(vertices_.col(i)(0));
        float y = abs(vertices_.col(i)(1));
        float z = abs(vertices_.col(i)(1));
        max_t = max4(x, y, z, max_t);
    }
    if (max_t > 0) { // 1.2 scale to 1*1*1 unit cube
        vertices_ /= max_t * 2;
    }
    return true;
}

//TransRec
TransRec::TransRec() {
    scale_ = 1;
    rotation_[0] = 0;
    rotation_[1] = 0;
    rotation_[2] = 0;
    translation_ = Vector3f::Zero();
}

Matrix4f TransRec::compute_matrix() {
    return TransUtils::translation(translation_)
        * TransUtils::rotation_x(rotation_[0])
        * TransUtils::rotation_y(rotation_[1])
        * TransUtils::rotation_z(rotation_[2])
        * TransUtils::scale(scale_);
}

Matrix4f TransRec::compute_with_trans(Vector3f trans) {
    return TransUtils::translation(trans)
        * TransUtils::rotation_x(rotation_[0])
        * TransUtils::rotation_y(rotation_[1])
        * TransUtils::rotation_z(rotation_[2])
        * TransUtils::scale(scale_);
}

//MeshObject
MeshObject::MeshObject(MeshPrimitive* p, Material* mp) {
    trans_matrix_ = Matrix4f::Identity();
    primitivep_ = p;
    materialp_ = mp;
    trans_updated_ = false;
}

void MeshObject::translate(const Vector3f& t) {
    trans_rec_.translation_ += t;
    //update_trans_matrix();
}

void MeshObject::rotate(float a, int axis) {
    trans_rec_.rotation_[axis] += a;
    //update_trans_matrix();
}

void MeshObject::scale(float s) {
    float res = trans_rec_.scale_ * s;
    if (res >= 0.01) {
        trans_rec_.scale_ = res;
        //update_trans_matrix();
    }
}

void MeshObject::update_trans_matrix() {
    trans_matrix_ = trans_rec_.compute_matrix();
    trans_updated_ = false;
}

void MeshObject::update_trans_vertices() {
    if (trans_updated_) { return; }
    int nv = primitivep_->vertices_.cols();
    if (trans_vertices_.size() != nv) {
        trans_vertices_.clear();
        trans_vertices_.reserve(nv);
        for (int i = 0; i < nv; i++) {
            trans_vertices_.push_back(Vector3f::Zero());
        }
    }
    for (int i = 0; i < nv; i++) {
        trans_vertices_[i] = TransUtils::transform(
                trans_matrix_, primitivep_->vertices_.col(i));
    }
    trans_updated_ = true;
}

bool MeshObject::hit(const Ray& ray, HitRecord* record) const {
    // test with sphere boundary
    HitRecord sphere_rec;
    Sphere s(trans_rec_.translation_, sqrt(3) * trans_rec_.scale_, NULL);
    if (!s.easy_hit(ray)) { return false; }

    HitRecord tmp_rec;
    bool found = false;
    float t_front = ray.get_tmax();
    Vector3f normal_front;
    Material* mp_front;
    //update_trans_vertices(); // lazy-updating translation
    for (const auto & face : primitivep_->faces_) {
        if(Triangle(trans_vertices_[face(0)], trans_vertices_[face(1)],
                trans_vertices_[face(2)], materialp_).hit(ray, &tmp_rec)
                && tmp_rec.t_ < t_front) {
            found = true;
            t_front = tmp_rec.t_;
            normal_front = tmp_rec.normal_;
            mp_front = tmp_rec.materialp_;
        }
    }
    if (found) {
        record->t_ = t_front;
        record->position_ = ray.get_point_at(t_front);
        record->normal_ = normal_front;
        record->materialp_ = mp_front;
        return true;
    }
    return false;
}

//Rectangular
Rect::Rect() {}
Rect::Rect(Vector2f lowerleft, Vector2f upperright, float k, Material* mp, int naxis, bool flip) :
      lowerleft_(lowerleft), upperright_(upperright), k_(k), materialp_(mp) {
    norm_axis_ = naxis;
    if (naxis == 1) {
        plane_axis_[0] = 0;
        plane_axis_[1] = 2;
    } else {
        plane_axis_[0] = (naxis + 1) % 3;
        plane_axis_[1] = (naxis + 2) % 3;
    }
    normal_ = Vector3f::Zero();
    normal_(naxis) = 1;
    if (flip) {
        normal_ *= -1;
    }
}

bool Rect::hit(const Ray& ray, HitRecord* record) const {
   Vector3f ori = ray.get_ori();
   Vector3f dir = ray.get_dir();

   float t = (k_ - ori(norm_axis_)) / dir(norm_axis_);
   if (!ray.covers(t)) { return false; }
   float x = ori(plane_axis_[0]) + t * dir(plane_axis_[0]);
   float y = ori(plane_axis_[1]) + t * dir(plane_axis_[1]);
   if (x < lowerleft_(0) || x > upperright_(0) || y < lowerleft_(1) || y > upperright_(1) ) {
      return false;
   }
   record->materialp_ = materialp_;
   record->normal_ = normal_;
   record->position_ = ray.get_point_at(t);
   record->t_ = t;
   return true;
}