#pragma once

#include <string>
struct Vec3 {
    double x, y, z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(double X, double Y, double Z) : x(X), y(Y), z(Z) {}

    Vec3 operator+(Vec3 const& o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
    Vec3 operator-(Vec3 const& o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
    Vec3 operator*(double s) const { return Vec3(x * s, y * s, z * s); }
    Vec3 operator/(double s) const { return Vec3(x / s, y / s, z / s); }


    Vec3& operator+=(Vec3 const& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(Vec3 const& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }

    bool operator==(Vec3 const& o) { return x == o.x && y == o.y && z == o.z; }


    static double calcDistance(Vec3 one, Vec3 two) {

        return std::sqrt ( 
            (one.x - two.x) * (one.x - two.x) +
            (one.y - two.y) * (one.y - two.y) +
            (one.z - two.z) * (one.z - two.z) );
    }

    std::string toString() {
        return std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z);
    }
};