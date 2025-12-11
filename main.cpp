// NBodyNaive.cpp
// Build: g++ -std=c++17 NBodyNaive.cpp -lGL -lGLU -lglut -o nbody_naive

#include <GL/freeglut.h>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>

constexpr double MY_PI = 3.14159265358979323846;
double dt = 0.001;
double worldRadius = 200;

// -----------------------------------------------------------------------------
// Simple Vec3
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

// -----------------------------------------------------------------------------
// Simple Body
struct Body {
    Vec3 pos;
    Vec3 vel;
    Vec3 force;
    double mass;

    Body(double x, double y, double z, double vx, double vy, double vz, double m)
        : pos(x, y, z), vel(vx, vy, vz), force(0, 0, 0), mass(m) {
    }

    void resetForce() { force = Vec3(0, 0, 0); }

    void update(double dt) {
        vel += force * (dt / mass);
        pos += vel * dt;
    }
};

std::vector<Body> bodies;

// camera spherical coords
double camRadius = 500.0;
double camTheta = 0.0;
double camPhi = MY_PI / 2.0;

// -----------------------------------------------------------------------------
// Naive N² gravity
void computeForces() {
    const double softening = 1e-2;
    const double G = 0.1;

    for (auto& b : bodies)
        b.resetForce();

    int N = bodies.size();

    for (int i = 0; i < N; i++) {
        for (int j = i + 1; j < N; j++) {       
            Vec3 d = bodies[j].pos - bodies[i].pos;
            double r2 = d.x * d.x + d.y * d.y + d.z * d.z;
            double invR = 1.0 / std::sqrt(r2);
            double invR3 = invR * invR * invR + softening * softening * softening;
            double F = G * bodies[i].mass * bodies[j].mass * invR3;

            Vec3 f = d * F;
            bodies[i].force += f;
            bodies[j].force -= f;
        }
    }
}

void stepSimulation() {
    computeForces();
    for (auto& b : bodies)
        b.update(dt);
}

// -----------------------------------------------------------------------------
// camera input
void specialKeys(int key, int, int) {
    const double dA = 0.05;
    if (key == GLUT_KEY_LEFT)  camTheta -= dA;
    if (key == GLUT_KEY_RIGHT) camTheta += dA;
    if (key == GLUT_KEY_UP)    camPhi = std::max(0.1, camPhi - dA);
    if (key == GLUT_KEY_DOWN)  camPhi = std::min(MY_PI - 0.1, camPhi + dA);
    glutPostRedisplay();
}

void keyboard(unsigned char k, int, int) {
    if (k == '+') camRadius = std::max(10.0, camRadius - 20.0);
    if (k == '-') camRadius += 20.0;
    if (k == 27) std::exit(0);
    glutPostRedisplay();
}

// -----------------------------------------------------------------------------
// draw
void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // camera position
    Vec3 camPos{
        camRadius * std::sin(camPhi) * std::cos(camTheta),
        camRadius * std::cos(camPhi),
        camRadius * std::sin(camPhi) * std::sin(camTheta)
    };

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(camPos.x, camPos.y, camPos.z,
        0.0, 0.0, 0.0,
        0.0, 1.0, 0.0);

    glDisable(GL_DEPTH_TEST);
    glColor3f(1, 1, 1);

    for (auto& b : bodies) {
        glPointSize(5.0f);
        glBegin(GL_POINTS);
        glVertex3d(b.pos.x, b.pos.y, b.pos.z);
        glEnd();
    }

    glutSwapBuffers();
}

// -----------------------------------------------------------------------------
// idle
void idleFunc() {
    stepSimulation();
    glutPostRedisplay();
}

// -----------------------------------------------------------------------------
// reshape
void reshape(int w, int h) {
    if (h == 0) h = 1;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, double(w) / double(h), 1.0, 5000.0);
    glMatrixMode(GL_MODELVIEW);
}

// -----------------------------------------------------------------------------
// main
int main(int argc, char** argv) {

    int N = 400;
    dt = 0.1;

    std::srand(unsigned(std::time(nullptr)));
    bodies.reserve(N);

    for (int i = 0; i < N; i++) {
        double x = (std::rand() / double(RAND_MAX)) * 2 * worldRadius - worldRadius;
        double y = (std::rand() / double(RAND_MAX)) * 2 * worldRadius - worldRadius;
        double z = (std::rand() / double(RAND_MAX)) * 2 * worldRadius - worldRadius;
        bodies.emplace_back(x, y, z, 0, 0, 0, 10.0);
    }

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 800);
    glutCreateWindow("Naive N-Body Gravity");

    glClearColor(0, 0, 0, 1);
    glEnable(GL_DEPTH_TEST);

    glutDisplayFunc(display);
    glutIdleFunc(idleFunc);
    glutReshapeFunc(reshape);
    glutSpecialFunc(specialKeys);
    glutKeyboardFunc(keyboard);

    glutMainLoop();
    return 0;
}
