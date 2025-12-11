#include "Vec3.cpp"
class Cube {
public:
	float sideLen;
	Vec3 origin;

	Cube() {
		this->origin = Vec3(0, 0, 0);
		this->sideLen = 1.0f;
	}

	Cube(Vec3 orig, float sizeL) {
		this->origin = orig;
		this->sideLen = sizeL;
	}

	bool containsPoint(Vec3 point) {
		return (point.x >= origin.x && point.x <= origin.x + sideLen) &&
			(point.y >= origin.y && point.y <= origin.y + sideLen) &&
			(point.z >= origin.z && point.z <= origin.z + sideLen);
	}

};