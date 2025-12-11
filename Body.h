#pragma once
#include "Vec3.h"

/* Stores all of the information related to the particle bodies involved 
in the simulation.
*/
struct Body {
    Vec3 pos;
    Vec3 vel;
    Vec3 force;
    double mass;

    Body(double x, double y, double z, double vx, double vy, double vz, double m)
        : pos(x, y, z), vel(vx, vy, vz), force(0, 0, 0), mass(m) {
    }

    void resetForce() { force = Vec3(0, 0, 0); }

    /* Updates the position of the body according to the force and the
      delta time.
    */
    void update(double dt) {
        vel += force * (dt / mass);
        pos += vel * dt;
    }
};