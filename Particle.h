#pragma once
#include <glm/glm.hpp>

struct Particle {
    glm::vec3 pos;
    glm::vec3 vel;
    glm::vec3 acc;
    float mass;
};