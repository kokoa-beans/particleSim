#pragma once

#include "Vec3.h"
struct Cube{
	Vec3 origin;
	double width;

	Cube() : origin(0,0,0), width(0) {};

	Cube(double x, double y, double z, double w)
		: origin(x, y, z), width(w) {
	}

	bool containsPoint(Vec3 checkPoint) {
		// Check if the point is within the bounds for all three axes (X, Y, and Z)
		bool withinX = (checkPoint.x >= origin.x) && (checkPoint.x <= origin.x + width);
		bool withinY = (checkPoint.y >= origin.y) && (checkPoint.y <= origin.y + width);
		bool withinZ = (checkPoint.z >= origin.z) && (checkPoint.z <= origin.z + width);

		return withinX && withinY && withinZ;
	}
};