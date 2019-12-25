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

const int MAX_DEPTH = 3;
const size_t SAMPLES = 100;
const bool IS_TEST = true;
const int TRACE_MODE = 0;
const int TRACE_PATH= 0;
const int TRACE_RAY= 1;

Vector3f gen_positive_rand() {
    return 0.5 * (Vector3f::Random() + Vector3f::Ones());
}

void build_world_connell(HitableList* world, Camera* cam, HitableList* lights = NULL) {

    //world->add(new Plane(Vector3f(0, 0, 0), Vector3f(0, 1, 0), new Metal(Vector3f(0.5, 0.5, 0.5), 0.0) ));
    
    Material* red = new Lambertian(Vector3f(0.65, 0.05, 0.05));
    Material* white = new Lambertian(Vector3f(0.73, 0.73, 0.73));
    Material* green = new Lambertian(Vector3f(0.12, 0.45, 0.15));
    Material* light = new DiffuseLight(Vector3f(15, 15, 15));

    world->add(new Rect(Vector2f(0, 0), Vector2f(555, 555), 555, green, 0, true));
    world->add(new Rect(Vector2f(0, 0), Vector2f(555, 555), 0, red, 0));
    world->add(new Rect(Vector2f(213, 227), Vector2f(343, 332), 554, light, 1, true));
    world->add(new Rect(Vector2f(0, 0), Vector2f(555, 555), 555, white, 1, true));
    world->add(new Rect(Vector2f(0, 0), Vector2f(555, 555), 0, white, 1));
    world->add(new Rect(Vector2f(0, 0), Vector2f(555, 555), 555, white, 2, true));


    cam->config(500, 500, tan(M_PI / 9), 10, 2000, 10, 0);
    cam->look_at(Vector3f(278, 278, -800), Vector3f(278, 278, 0));
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
        Vector3f global_light = Vector3f(0.01, 0.01, 0.01);
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

void task_box() {
    ImageInfo img(500, 500);

    HitableList world;
    HitableList lights;
    Camera camera(500, 500, 0.5, 3, 100, 6, 0.02);
    build_world_connell(&world, &camera, &lights);

    RayRender rayrender;
    rayrender.init(&img, &world, &camera, &lights);

    parallel_for(blocked_range2d<size_t>(0, 500, 0, 500), rayrender);
    img.output_image("test_box.png");
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
    task_box();

    auto t_to = chrono::high_resolution_clock::now();
    float sec = chrono::duration_cast<std::chrono::duration<float>>(
                    t_to - t_from).count();
    cout << " (" << sec << " sec)"<< endl;

    return 0;
}