#version 330 core
out vec4 FragColor;
void main() {
    float d = length(gl_PointCoord - vec2(0.5));
    if (d > 0.5) discard;
    FragColor = vec4(1.0, 0.8, 0.2, 1.0);
}