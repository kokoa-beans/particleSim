// NBodyNaive.cpp
// Build: g++ -std=c++17 NBodyNaive.cpp -lGL -lGLU -lglut -o nbody_naive

#include <GL/freeglut.h>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>

#include "Vec3.h"
#include "Body.h"
#include "Octree.h"

constexpr double MY_PI = 3.14159265358979323846;
double dt = 1e-5;
double worldRadius = 500;

double octreeSize = 0.0;

const double softening = 1e-4;
const double G = 1;

const double errorConstant = 0.01;

std::vector<Body> bodies;
Octree tree;

Body OctreeNode::defaultBodyPlaceholder = Body(INFINITE, INFINITE, INFINITE, INFINITE, INFINITE, INFINITE, INFINITE);


// camera spherical coords
double camRadius = 500.0;
double camTheta = 0.0;
double camPhi = MY_PI / 2.0;


void recurCalculateForce(OctreeNode* curNode, Body& body) {
    if (curNode->isLeaf() && curNode->centerOfMass == body.pos) return;
    
    double ratio = (curNode->boundingCube.width) / (Vec3::calcDistance(curNode->centerOfMass, body.pos));
    if (ratio < errorConstant || (curNode->isLeaf()) ) {
        Vec3 d = curNode->centerOfMass - body.pos;
        double r2 = d.x * d.x + d.y * d.y + d.z * d.z;

        r2 += softening * softening;

        double invR = 1.0 / std::sqrt(r2);
        double invR3 = (invR * invR * invR);
        double F = G * body.mass * curNode->mass * invR3;

        Vec3 f = d * F;

        body.force += f;
    }
    else {
        for (OctreeNode* each : curNode->nodes) {
            if (each != nullptr) recurCalculateForce(each, body);
        }
    }
}

void computeEfficientForces() {
    for (auto& b : bodies) b.resetForce();
    
    int N = bodies.size();

    for (int i = 0; i < N; i++) {
        Body& checking = bodies[i];

        recurCalculateForce(tree.root, checking);
    }
}

// -----------------------------------------------------------------------------
// Naive N^2 gravity
void computeNaiveForces() {
    for (auto& b : bodies) b.resetForce();

    int N = bodies.size();

    for (int i = 0; i < N; i++) {
        for (int j = i + 1; j < N; j++) {
            Vec3 d = bodies[j].pos - bodies[i].pos;
            double r2 = d.x * d.x + d.y * d.y + d.z * d.z;
            double invR = 1.0 / std::sqrt(r2);
            double invR3 = (invR * invR * invR) + (softening * softening * softening);
            double F = G * bodies[i].mass * bodies[j].mass * invR3;

            Vec3 f = d * F;
            bodies[i].force += f;
            bodies[j].force -= f;
        }
    }
}

void stepSimulation() {
    computeEfficientForces();
    //computeNaiveForces();

    int size = bodies.size();

    std::cout << "Size is " << size << std::endl;


    for (auto& b : bodies) {
        std::cout << b.pos.toString() << std::endl;
        b.update(dt);
    }
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


void octreeRecursiveDraw(OctreeNode* node) {
    Cube boundingBox = node->boundingCube;


    // Assuming these are your min/max coordinates from boundingBox
    float minX = boundingBox.origin.x;
    float minY = boundingBox.origin.y;
    float minZ = boundingBox.origin.z;
    float maxX = minX + boundingBox.width; // Assuming width applies to all dims
    float maxY = minY + boundingBox.width;
    float maxZ = minZ + boundingBox.width;

    glBegin(GL_LINES);

    // Calculate a normalized value between 0.0 and 1.0 based on the box width relative to octreeSize
    float t = boundingBox.width / octreeSize;

    // Ensure 't' is clamped within the 0.0 to 1.0 range in case the input values are slightly outside
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    // For a blue (far/small 't') to red (close/large 't') gradient:
    // Red component increases as 't' approaches 1.0
    // Blue component decreases as 't' approaches 1.0 (or increases as 't' approaches 0.0)
    float red = t;
    float blue = 1.0f - t;
    float green = node->isLeaf() ? 1.0f : 0.0f; // No green for a pure blue-to-red gradient

    glColor3f(red, green, blue); // Set the color

    // Bottom Face (Z-min)
    glVertex3f(minX, minY, minZ); glVertex3f(maxX, minY, minZ); // Bottom edge X
    glVertex3f(maxX, minY, minZ); glVertex3f(maxX, maxY, minZ); // Right edge Y
    glVertex3f(maxX, maxY, minZ); glVertex3f(minX, maxY, minZ); // Top edge X
    glVertex3f(minX, maxY, minZ); glVertex3f(minX, minY, minZ); // Left edge Y

    // Top Face (Z-max)
    glVertex3f(minX, minY, maxZ); glVertex3f(maxX, minY, maxZ); // Bottom edge X
    glVertex3f(maxX, minY, maxZ); glVertex3f(maxX, maxY, maxZ); // Right edge Y
    glVertex3f(maxX, maxY, maxZ); glVertex3f(minX, maxY, maxZ); // Top edge X
    glVertex3f(minX, maxY, maxZ); glVertex3f(minX, minY, maxZ); // Left edge Y

    // Connecting Edges (Vertical)
    glVertex3f(minX, minY, minZ); glVertex3f(minX, minY, maxZ);
    glVertex3f(maxX, minY, minZ); glVertex3f(maxX, minY, maxZ);
    glVertex3f(maxX, maxY, minZ); glVertex3f(maxX, maxY, maxZ);
    glVertex3f(minX, maxY, minZ); glVertex3f(minX, maxY, maxZ);

    glEnd();


    for (OctreeNode* each : node->nodes) {
        
        if (each != nullptr) octreeRecursiveDraw(each);
    }
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

    octreeRecursiveDraw(tree.root);

    glutSwapBuffers();
}


// -----------------------------------------------------------------------------
// idle
void idleFunc() {
    octreeSize = worldRadius * 10;
    tree = Octree(bodies, octreeSize);
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

    int N = 10;
    dt = 1e-1;

    std::srand(unsigned(std::time(nullptr)));
    bodies.reserve(N);


    for (int i = 0; i < N; i++) {
        double x = (std::rand() / double(RAND_MAX)) * 2 * worldRadius - worldRadius;
        double y = (std::rand() / double(RAND_MAX)) * 2 * worldRadius - worldRadius;
        double z = (std::rand() / double(RAND_MAX)) * 2 * worldRadius - worldRadius;

        double vx = (std::rand() * .5) - 1;
        double vy = (std::rand() * .5) - 1;
        double vz = (std::rand() * .5) - 1;


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
