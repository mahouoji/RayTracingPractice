// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <sstream>
#include <ctime>
#include <cstdlib>
#include <iomanip>
// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"
#include "utils.h"
// Header files added by mhoj
#include "hitable.h"
#include "camera.h"
#include "material.h"
#include "transutils.h"
#include <tbb/parallel_for.h>
#include <tbb/atomic.h>
#include <tbb/blocked_range2d.h>
#include <tbb/blocked_range3d.h>
// Shortcut to avoid Eigen:: and std:: everywhere, DO NOT USE IN .h
using namespace std;
using namespace Eigen;
using namespace tbb;

const int MAX_DEPTH = 4;
const size_t SAMPLES = 100;
const bool IS_TEST = true;
const int TRACE_MODE = 1;
const int TRACE_PATH= 0;
const int TRACE_RAY= 1;

Vector3f gen_positive_rand() {
    return 0.5 * (Vector3f::Random() + Vector3f::Ones());
}

void build_world_a1(HitableList* world, Camera* cam, HitableList* lights = NULL) {
    HitableList* lp = world;
    if (lights) { lp = lights; }   
    Sphere* l1 = new Sphere(Vector3f(-10, 3, 10), 5, new DiffuseLight(Vector3f(1, 0, 1)) );
    Sphere* l2 = new Sphere(Vector3f(10, 3, 10), 5, new DiffuseLight(Vector3f(0, 1, 1)) );
    Sphere* l3 = new Sphere(Vector3f(0, 3, -10), 5, new DiffuseLight(Vector3f::Ones()) );
    world->add(l1);
    world->add(l2);
    world->add(l3);
    if (lights) {
        lights->add(l1);
        lights->add(l2);
        lights->add(l3);
    }

    world->add(new Plane(Vector3f(0, 0, 0), Vector3f(0, 1, 0), new Metal(Vector3f(0.5, 0.5, 0.5), 0.0) ));

    world->add(new Sphere(Vector3f(-2, 0.6, -3), 0.6, new Lambertian(Vector3f(0.8, 0.2, 0.2)) ));
    world->add(new Sphere(Vector3f(-1, 2, -2), 0.9, new Metal(Vector3f(0.2, 0.2, 0.8), 0.1) ));
    world->add(new Sphere(Vector3f(-0.5, 0.2, 1), 0.2, new Metal(Vector3f(0.8, 0.2, 0.8), 0.1) ));

    MeshPrimitive* box = new MeshPrimitive();
    box->load_off("../data/test.off");
    MeshPrimitive* cube = new MeshPrimitive();
    cube->load_off("../data/bumpy_cube.off");
    MeshPrimitive* bunny = new MeshPrimitive();
    bunny->load_off("../data/bunny.off");

    MeshObject* cube_obj = new MeshObject(cube, new Metal(Vector3f(0.2, 0.8, 0.2), 0.3) );
    cube_obj->rotate(M_PI / 6, 1);
    cube_obj->rotate(M_PI / 4, 2);
    cube_obj->translate(Vector3f(1.5, 1.5, -0.2));
    cube_obj->scale(1.5);
    cube_obj->update_trans_matrix();
    cube_obj->update_trans_vertices();

    MeshObject* bunny_obj = new MeshObject(bunny, new Metal(Vector3f(1, 1, 1), 0.1) );
    bunny_obj->translate(Vector3f(0.8, 0.6, -1));
    bunny_obj->scale(2);
    bunny_obj->update_trans_matrix();
    bunny_obj->update_trans_vertices();
    if (!IS_TEST) {
        world->add(cube_obj);
        world->add(bunny_obj);
    }

    cam->look_at(Vector3f(0, 1, 6), Vector3f(0, 0, 0));
}

void build_world(HitableList* world, Camera* cam, HitableList* lights = NULL) {
    HitableList* lp = world;
    if (lights) { lp = lights; }
    Sphere* l1 = new Sphere(Vector3f(-3, 0.5, -1), 0.5, new DiffuseLight(Vector3f::Ones()) );
    Sphere* l2 = new Sphere(Vector3f(2.5, 0.8, 0), 0.5, new DiffuseLight(Vector3f::Ones()) );
    world->add(l1);
    world->add(l2);
    if (lights) {
        lights->add(l1);
        lights->add(l2);
    }
    //world->add(new Plane(Vector3f(0, 0, 0), Vector3f(0, 1, 0), new Lambertian(Vector3f(0.8, 0.8, 0.8)) ));
    world->add(new Plane(Vector3f(0, 0, 0), Vector3f(0, 1, 0), new Metal(Vector3f(0.5, 0.5, 0.5), 0.0) ));
    
    world->add(new Sphere(Vector3f(-3, 1.5, -3), 1.5, new Metal(Vector3f(0.5, 0.5, 0.5), 0.5) ));
    for (int i = 0; i < 11; i++) {
        world->add(new Sphere(Vector3f(0, 0.3, -6 + i), 0.3, new Lambertian(gen_positive_rand()) ));
    }
    MeshPrimitive* box = new MeshPrimitive();
    box->load_off("../data/test.off");
    MeshPrimitive* cube = new MeshPrimitive();
    cube->load_off("../data/bumpy_cube.off");
    MeshPrimitive* bunny = new MeshPrimitive();
    bunny->load_off("../data/bunny.off");
    MeshObject* obj = new MeshObject(cube, new Lambertian(gen_positive_rand()) );
    obj->scale(2);
    obj->translate(Vector3f(-2, 1.5, 2));
    obj->rotate(M_PI / 6, 2);
    obj->update_trans_matrix();
    obj->update_trans_vertices();
    if (!IS_TEST) { world->add(obj); }

    MeshObject* obj2 = new MeshObject(bunny, new Metal(gen_positive_rand(), 0.005) );
    obj2->scale(2);
    obj2->translate(Vector3f(1.5, 1, 1));
    obj2->rotate(M_PI / 6, 1);
    obj2->update_trans_matrix();
    obj2->update_trans_vertices();
    if (!IS_TEST) { world->add(obj2); }

    cam->look_at(Vector3f(5, 5, 5), Vector3f(0, 0, 0));
}

struct ImageInfo {
    // Store the color
    MatrixXd c_R;
    MatrixXd c_G;
    MatrixXd c_B;
    // Store the alpha mask
    MatrixXd A;
    int width_, height_;

    ImageInfo(size_t width, size_t height) {
        width_ = width;
        height_ = height;
        c_R.resize(width, height);
        c_G.resize(width, height);
        c_B.resize(width, height);
        A.resize(width, height);
    }
    void set(size_t i, size_t j, double r, double g, double b, double a) {
        c_R(i, j) = r;
        c_G(i, j) = g;
        c_B(i, j) = b;
        A(i, j) = a;
    }

    void map_tone() {
        for (size_t i = 0; i < width_; ++i) {
            for (size_t j = 0; j < height_; ++j) {
                c_R(i, j) = c_R(i, j) / (1 + c_R(i, j));
                c_G(i, j) = c_G(i, j) / (1 + c_G(i, j));
                c_B(i, j) = c_B(i, j) / (1 + c_B(i, j));
                c_R(i, j) = pow(c_R(i, j), 0.45);
                c_G(i, j) = pow(c_G(i, j), 0.45);
                c_B(i, j) = pow(c_B(i, j), 0.45);
                if (c_R(i, j) > 1) { cout << c_R(i, j) << endl; }
                if (c_G(i, j) > 1) { cout << c_G(i, j) << endl; }
                if (c_B(i, j) > 1) { cout << c_B(i, j) << endl; }
            }
        }
    }

    void gamma_correction() {
        for (size_t i = 0; i < width_; ++i) {
            for (size_t j = 0; j < height_; ++j) {
                c_R(i, j) = pow(c_R(i, j), 0.45);
                c_G(i, j) = pow(c_G(i, j), 0.45);
                c_B(i, j) = pow(c_B(i, j), 0.45);
                if (c_R(i, j) > 1) { cout << c_R(i, j) << endl; }
                if (c_G(i, j) > 1) { cout << c_G(i, j) << endl; }
                if (c_B(i, j) > 1) { cout << c_B(i, j) << endl; }
            }
        }
    }

    void output_image(string output_filename) {
        //gamma_correction();
        map_tone();
        stringstream ss;
        ss << output_filename << ".png";
        write_matrix_to_png(c_R, c_G, c_B, A, ss.str());
    }
};

class RayRender {
 public:
    void init(ImageInfo* image, HitableList* world, Camera* camera, HitableList* light) {
        imagep_ = image;
        worldp_ = world;
        camerap_ = camera;
        lightp_ = light;
    }

    Vector3f color_ray(const Ray& ray, const HitableList* world, int depth) const {
        Vector3f global_light = Vector3f(0.1, 0.1, 0.1);
        int light_samples = 5;
        if (depth == MAX_DEPTH) { return global_light; }
        HitRecord rec;
        if (world->hit(ray, &rec)) {
            Material* mp = rec.materialp_;
            ScatterRecord srec;
            Vector3f emitted = mp->emit();
            if (mp->scatter(ray, rec, &srec)) {
                if (TRACE_MODE == TRACE_PATH || depth < MAX_DEPTH - 2 || lightp_->hitables_.size() == 0) {
                    return emitted + srec.attenuation.asDiagonal() * color_ray(srec.specular_ray_, world, depth + 1);
                }
                //return emitted + srec.attenuation.asDiagonal() * color_ray(srec.specular_ray_, world, depth + 1);
                Vector3f color = Vector3f::Zero();
                for (const auto& lp : lightp_->hitables_) {
                    RandRecord rrec;
                    for (int s = 0; s < light_samples; s++) {
                        if (lp->rand_pos(rec.position_, &rrec)) {
                            Vector3f dir = rrec.ray_to_rand_.get_dir();
                            Vector3f pos = rec.position_ + 1.0e-6 * rec.normal_;
                            Vector3f lcolor = color_ray(rrec.ray_to_rand_, world, depth + 1);
                            color += srec.attenuation.asDiagonal() * max(rec.normal_.dot(dir), 0.0f) * lcolor;
                        }
                    }
                }
                return emitted + color / (light_samples);
                //return emitted + color / (lightp_->hitables_.size() * light_samples);
                //return emitted + srec.attenuation.asDiagonal() * color_ray(srec.specular_ray_, world, depth + 1);
            } else {
                return emitted;
            }
        } else {
            return global_light;
        }
    }

    void operator()(const blocked_range2d<size_t>& r) const {
        for (size_t i = r.rows().begin(); i != r.rows().end(); ++i){
            for (size_t j = r.cols().begin(); j != r.cols().end(); ++j) {
                size_t sample = SAMPLES;
                Vector3f color = Vector3f::Zero();
                for (size_t k = 0; k < sample; ++k) {
                    Vector2f rnd = 0.5 * (Vector2f::Random() + Vector2f::Ones());
                    Ray ray = camerap_->get_ray_perspective_focus(i + rnd(0), j + rnd(0));
                    color += color_ray(ray, worldp_, 0);
                }
                color /= (float)sample;
                imagep_->set(i, j, color(0), color(1), color(2), 1);
            }
        }
    }


    ImageInfo* imagep_;
    HitableList* worldp_;
    HitableList* lightp_;
    Camera* camerap_;
};

class PathRender {
 public:
    void init(ImageInfo* image, HitableList* world, Camera* camera) {
        imagep_ = image;
        worldp_ = world;
        camerap_ = camera;
    }

    Vector3f color_ray(const Ray& ray, const HitableList* world, int depth) const {
        if (depth > MAX_DEPTH) { return Vector3f(0.3, 0.3, 0.3); }
        HitRecord rec;
        if (world->hit(ray, &rec)) {
            Material* mp = rec.materialp_;
            ScatterRecord srec;
            Vector3f emitted = mp->emit();
            if (mp->scatter(ray, rec, &srec)) {
                Vector3f emitted = mp->emit();
                return emitted + srec.attenuation.asDiagonal() * color_ray(srec.specular_ray_, world, depth + 1);
            } else {
                return emitted;
            }
        } else {
            return Vector3f(0.3, 0.3, 0.3);
        }
    }

    void operator()(const blocked_range2d<size_t>& r) const {
        for (size_t i = r.rows().begin(); i != r.rows().end(); ++i){
            for (size_t j = r.cols().begin(); j != r.cols().end(); ++j) {
                size_t sample = SAMPLES;
                Vector3f color = Vector3f::Zero();
                for (size_t k = 0; k < sample; ++k) {
                    Vector2f rnd = 0.5 * (Vector2f::Random() + Vector2f::Ones());
                    Ray ray = camerap_->get_ray_perspective_focus(i + rnd(0), j + rnd(0));
                    color += color_ray(ray, worldp_, 0);
                }
                color /= (float)sample;
                imagep_->set(i, j, color(0), color(1), color(2), 1);
            }
        }
    }

    void operator()(const blocked_range3d<size_t>& r) const {
        for (size_t i = r.pages().begin(); i != r.pages().end(); ++i){
            for (size_t j = r.rows().begin(); j != r.rows().end(); ++j) {
                tbb::atomic<float> pr;
                tbb::atomic<float> pg;
                tbb::atomic<float> pb;
                pr = 0; pb = 0; pb = 0;
                for (size_t k = r.cols().begin(); k != r.cols().end(); ++k) {
                    Vector2f rnd = 0.5 * (Vector2f::Random() + Vector2f::Ones());
                    Ray ray = camerap_->get_ray_perspective_focus(i + rnd(0), j + rnd(0));
                    Vector3f color = color_ray(ray, worldp_, 0);
                    pr = pr + color(0);
                    pg = pg + color(1);
                    pb = pb + color(2);
                }
                pr = pr / 100;
                pg = pg / 100;
                pb = pb / 100;
                imagep_->set(i, j, pr, pg, pb, 1);
            }
        }
    }

    ImageInfo* imagep_;
    HitableList* worldp_;
    Camera* camerap_;
};

void task_focus() {
    ImageInfo img(800, 800);

    HitableList world;
    HitableList lights;
    Camera camera(800, 800, 0.5, 3, 100, 6, 0.1);
    build_world(&world, &camera, &lights);

    RayRender rayrender;
    rayrender.init(&img, &world, &camera, &lights);

    for (int f = 1; f <= 20; f++) {
        auto t_from = chrono::high_resolution_clock::now();

        camera.config_focus(f, 0.1);
        parallel_for(blocked_range2d<size_t>(0, 800, 0, 800), rayrender);
        stringstream ss;
        ss << "test_focus" << f;
        img.output_image(ss.str());

        auto t_to = chrono::high_resolution_clock::now();
        float sec = chrono::duration_cast<std::chrono::duration<float>>(
                    t_to - t_from).count();
        cout << " (" << sec << " sec)"<< endl;

    }
}

void task_aperture() {
    ImageInfo img(800, 800);

    HitableList world;
    HitableList lights;
    Camera camera(800, 800, 0.5, 3, 100, 6, 0.1);
    build_world(&world, &camera, &lights);

    RayRender rayrender;
    rayrender.init(&img, &world, &camera, &lights);
    float as[14] = {0, 0.02, 0.04, 0.06, 0.08, 0.10, 0.12, 0.14, 0.16, 0.18, 0.20, 0.22, 0.24, 0.26};
    for (int i = 0; i < 14; i++) {
        auto t_from = chrono::high_resolution_clock::now();

        float r = sqrt(as[i]);
        camera.config_focus(8, r);
        parallel_for(blocked_range2d<size_t>(0, 800, 0, 800), rayrender);
        stringstream ss;
        ss << "test_aperture" << i << "_" << setprecision(3) << r;
        img.output_image(ss.str());

        auto t_to = chrono::high_resolution_clock::now();
        float sec = chrono::duration_cast<std::chrono::duration<float>>(
                    t_to - t_from).count();
        cout << " (" << sec << " sec)"<< endl;

    }
}

void task_camera() {
    ImageInfo img(800, 800);

    HitableList world;
    HitableList lights;
    Camera camera(800, 800, 0.5, 3, 100, 8, 0.2);
    build_world(&world, &camera, &lights);

    RayRender rayrender;
    rayrender.init(&img, &world, &camera, &lights);

    int pic_num = 24;
    float delta = 2 * M_PI / pic_num;
    float arc = 0;
    for (int k = 0; k < pic_num; k++, arc += delta) {
        auto t_from = chrono::high_resolution_clock::now();

        float d_sin = sin(arc);
        float d_cos = cos(arc);
        camera.look_at(Vector3f(7 * d_sin, 4, 7 * d_cos), Vector3f(0, 0, 0));

        parallel_for(blocked_range2d<size_t>(0, 800, 0, 800), rayrender);

        stringstream ss;
        ss << "test_camera" << k;
        img.output_image(ss.str());

        auto t_to = chrono::high_resolution_clock::now();
        float sec = chrono::duration_cast<std::chrono::duration<float>>(
                    t_to - t_from).count();
        cout << " (" << sec << " sec)"<< endl;

    }
}

void task_mesh() {
    ImageInfo img(800, 800);

    HitableList world;
    HitableList lights;
    Camera camera(800, 800, 0.5, 3, 100, 6, 0.02);
    build_world(&world, &camera, &lights);

    RayRender rayrender;
    rayrender.init(&img, &world, &camera, &lights);

    int pic_num = 1;
    float delta =  M_PI_2 / pic_num;
    float arc = 0;
    camera.look_at(Vector3f(5, 7, 5), Vector3f(0, 0, 0));
    for (int k = 0; k < pic_num; k++, arc += delta) {
        auto t_from = chrono::high_resolution_clock::now();

        float d_sin = sin(arc);
        float d_cos = cos(arc);
        camera.look_at(Vector3f(5 - 0.5*k, 7 - 0.7*k, 5-0.5*k), Vector3f(0, 0, 0));

        parallel_for(blocked_range2d<size_t>(0, 800, 0, 800), rayrender);

        stringstream ss;
        ss << "test_mesh" << k;
        img.output_image(ss.str());

        auto t_to = chrono::high_resolution_clock::now();
        float sec = chrono::duration_cast<std::chrono::duration<float>>(
                    t_to - t_from).count();
        cout << " (" << sec << " sec)"<< endl;

    }
}

int main() {
    initParallel();

    auto t_from = chrono::high_resolution_clock::now();
    /*
    PathRender render;
    ImageInfo img(800, 800);

    HitableList world;
    HitableList lights;

    Camera camera(800, 800, 0.5, 3, 100, 6, 0.05);

    build_world(&world, &camera);

    render.init(&img, &world, &camera);
    parallel_for(blocked_range2d<size_t>(0, 800, 0, 800), render);
    //parallel_for(blocked_range3d<size_t>(0, 800, 0, 800, 0, 100), render);
    img.output_image("test");

    build_world_a1(&world, &camera);
    render.init(&img, &world, &camera);
    parallel_for(blocked_range2d<size_t>(0, 800, 0, 800), render);
    img.output_image("testa1");

    RayRender rayrender;
    HitableList lights;
    build_world(&world, &camera, &lights);
    rayrender.init(&img, &world, &camera, &lights);
    parallel_for(blocked_range2d<size_t>(0, 800, 0, 800), rayrender);
    //parallel_for(blocked_range3d<size_t>(0, 800, 0, 800, 0, 100), render);
    img.output_image("lighttest");
    */
    //task_focus();
    //task_aperture();
    //task_camera();
    task_mesh();

    auto t_to = chrono::high_resolution_clock::now();
    float sec = chrono::duration_cast<std::chrono::duration<float>>(
                    t_to - t_from).count();
    cout << " (" << sec << " sec)"<< endl;

    return 0;
}