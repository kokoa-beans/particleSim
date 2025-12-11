// NBodyOctreeGL_DepthCamera.cpp
// Build: g++ -std=c++17 NBodyOctreeGL_DepthCamera.cpp -lGL -lGLU -lglut -o nbody_cam

#include <GL/freeglut.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <memory>
#include <array>
#include <cstdlib>
#include <ctime>
#include <unordered_map>
#include "Octree.cpp"

const double M_PI = 3.1415;


// tunable particle radius (in world units)
static double particleRadius = 5.0;
static constexpr double MIN_RADIUS = 0.1;
static constexpr double MAX_RADIUS = 50.0;

// simulation parameters
double Ttotal, dt, currentTime = 0;
double worldRadius;

// camera in spherical coords
double camRadius = 500.0;
double camTheta = 0.0;          // azimuth
double camPhi = M_PI / 2.0;    // inclination



// -----------------------------------------------------------------------------
// Globals, data & stepping
std::vector<Body> bodies;

void stepSimulation() {
    // NEW:
    initNodePool(bodies.size() * 2);
    OctreeNode* root = allocNode(Cube{ 0,0,0, worldRadius });

    // build
    for (auto& b : bodies)
        root->insert(&b);

    // forces
    for (auto& b : bodies) {
        b.resetForce();
        root->updateForce(&b);
    }

    // integrate
    for (auto& b : bodies) {
        b.update(dt);
    }

    // collision cell size
    double cellSize = 2.0 * particleRadius;
    std::unordered_map<long long, std::vector<int>> grid;
    grid.reserve(bodies.size() * 2);

    // 1) bin bodies
    for (int i = 0; i < bodies.size(); ++i) {
        auto& p = bodies[i].pos;
        int gx = int(floor((p.x + worldRadius) / cellSize));
        int gy = int(floor((p.y + worldRadius) / cellSize));
        int gz = int(floor((p.z + worldRadius) / cellSize));
        long long key = ((long long)gx << 42)
            | ((long long)gy << 21)
            | ((long long)gz);
        grid[key].push_back(i);
    }

    // 2) resolve only local collisions
    for (auto& cell : grid) {
        auto& idx = cell.second;
        for (int a = 0; a < idx.size(); ++a) {
            for (int b = a + 1; b < idx.size(); ++b) {
                int i = idx[a], j = idx[b];
                Vec3 d = bodies[j].pos - bodies[i].pos;
                double dist2 = d.x * d.x + d.y * d.y + d.z * d.z;
                double minD = 2 * particleRadius;
                if (dist2 < minD * minD) {
                    double dist = std::sqrt(dist2);
                    if (dist < 1e-6) dist = minD;
                    Vec3 nrm = d * (1.0 / dist);
                    double overlap = 0.5 * (minD - dist);
                    bodies[i].pos = bodies[i].pos - (nrm * overlap);
                    bodies[j].pos = bodies[j].pos + (nrm * overlap);
                }
            }
        }
    }

}

// -----------------------------------------------------------------------------
// Camera & input handlers
void specialKeys(int key, int, int) {
    const double dA = 0.05;
    switch (key) {
    case GLUT_KEY_LEFT:  camTheta -= dA; break;
    case GLUT_KEY_RIGHT: camTheta += dA; break;
    case GLUT_KEY_UP:    camPhi = std::max(0.1, camPhi - dA); break;
    case GLUT_KEY_DOWN:  camPhi = std::min(M_PI - 0.1, camPhi + dA); break;
    }
    glutPostRedisplay();
}

void keyboard(unsigned char k, int, int) {
    switch (k) {
    case '+': camRadius = std::max(10.0, camRadius - 20.0); break;
    case '-': camRadius += 20.0;                          break;

    case '[':  // decrease particle radius
        particleRadius = std::max(MIN_RADIUS, particleRadius - 1.0);
        break;
    case ']':  // increase particle radius
        particleRadius = std::min(MAX_RADIUS, particleRadius + 1.0);
        break;

    case 27:  std::exit(0);
    }
    glutPostRedisplay();
}

// -----------------------------------------------------------------------------
// Display: camera, depth‐scaled points
// -----------------------------------------------------------------------------
// Display: camera, depth‐scaled points + force arrows
void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // 1) compute camera position in Cartesian coords
    Vec3 camPos{
       camRadius * sin(camPhi) * cos(camTheta),
       camRadius * cos(camPhi),
       camRadius * sin(camPhi) * sin(camTheta)
    };

    // 2) setup camera
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(camPos.x, camPos.y, camPos.z,
        0.0, 0.0, 0.0,
        0.0, 1.0, 0.0);

    // 3) find max distance (for point‐size scaling)
    double maxDist2 = 0.0;
    for (auto& b : bodies) {
        Vec3 v = camPos - b.pos;
        double d2 = v.x * v.x + v.y * v.y + v.z * v.z;
        if (d2 > maxDist2) maxDist2 = d2;
    }
    double invMaxDist = 1.0 / sqrt(maxDist2);

    // 4) draw depth‐scaled points
    glDisable(GL_DEPTH_TEST);
    glColor3f(1, 1, 1);
    for (auto& b : bodies) {
        Vec3 v = camPos - b.pos;
        double d = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        // previous “base” size from depth:
        float baseSize = static_cast<float>(8.0 * (1.0 - (d * invMaxDist)) + 1.0);
        // scale by our tunable radius (default = 5)
        float size = baseSize * static_cast<float>(particleRadius / 5.0);
        glPointSize(size);
        glBegin(GL_POINTS);
        glVertex3d(b.pos.x, b.pos.y, b.pos.z);
        glEnd();
    }

    // 5) compute arrow scale based on max force magnitude
    double maxF2 = 0.0;
    for (auto& b : bodies) {
        double f2 = b.force.x * b.force.x
            + b.force.y * b.force.y
            + b.force.z * b.force.z;
        if (f2 > maxF2) maxF2 = f2;
    }
    double arrowScale = (maxF2 > 0.0)
        ? (worldRadius * 0.1 / sqrt(maxF2))
        : 1.0;

    // 6) draw force arrows
    glLineWidth(1.5f);
    glColor3f(1, 0, 0);
    glBegin(GL_LINES);
    for (auto& b : bodies) {
        Vec3 tip = b.pos + b.force * arrowScale;
        glVertex3d(b.pos.x, b.pos.y, b.pos.z);
        glVertex3d(tip.x, tip.y, tip.z);
    }
    glEnd();
    glEnable(GL_DEPTH_TEST);

    glutSwapBuffers();
}

// -----------------------------------------------------------------------------
// GL window & main
void idleFunc() {
    if (currentTime < Ttotal) {
        stepSimulation();
        currentTime += dt;
        glutPostRedisplay();
    }
}

void reshape(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, double(w) / h, 1.0, 5000.0);
    glMatrixMode(GL_MODELVIEW);
}

int main(int argc, char** argv) {



    // simulation setup
    int N = 100;
    worldRadius = 100.0;
    Ttotal = 1000000.0;
    dt = 1;

    std::srand(unsigned(std::time(nullptr)));
    bodies.reserve(N);
    for (int i = 0;i < N;i++) {
        double x = (rand() / double(RAND_MAX)) * 2 * worldRadius - worldRadius;
        double y = (rand() / double(RAND_MAX)) * 2 * worldRadius - worldRadius;
        double z = (rand() / double(RAND_MAX)) * 2 * worldRadius - worldRadius;
        double vx = 0, vy = 0, vz = 0;
        double m = 10;
        bodies.emplace_back(x, y, z, vx, vy, vz, m);
    }

    // GLUT + callbacks
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 800);
    glutCreateWindow("N-Body Octree: Depth & Camera");

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