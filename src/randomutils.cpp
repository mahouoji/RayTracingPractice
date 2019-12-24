#include "randomutils.h"
#include <cmath>

using namespace Eigen;

namespace RandomUtils{
float random_float() {
    return static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
}

Vector2f unit_in_circle() {
    float r = sqrt(random_float());
    float theta = random_float() * 2 * M_PI;
    return Vector2f(r * sin(theta), r * cos(theta));
}

Vector3f unit_on_sphere() {
    Vector3f rnd = Vector3f::Random();
    return rnd.normalized();
    float r1 = random_float();
    float r2 = random_float();
    float x = cos(2*M_PI*r1)*2*sqrt(r2*(1-r2));
    float y = sin(2*M_PI*r1)*2*sqrt(r2*(1-r2));
    float z = 1 - 2 * r2;
    return Vector3f(x, y, z);
}

};