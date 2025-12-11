#pragma once
struct Vec3 {
    double x, y, z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(double X, double Y, double Z) : x(X), y(Y), z(Z) {}

    Vec3 operator+(Vec3 const& o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
    Vec3 operator-(Vec3 const& o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
    Vec3 operator*(double s) const { return Vec3(x * s, y * s, z * s); }

    Vec3& operator+=(Vec3 const& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(Vec3 const& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
};