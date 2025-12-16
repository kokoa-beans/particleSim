cmake_minimum_required(VERSION 3.16)
project(particleSim)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

add_executable(particleSim main.cpp)

target_link_libraries(particleSim
        OpenGL::GL
        OpenGL::GLU
        GLUT::GLUT
)
