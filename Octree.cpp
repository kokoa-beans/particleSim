#include "Cube.cpp"
#include "memory"

class Octree {

public:
	Cube bounds;

	Vec3 centerOfMass;
	int numMass;

	std::unique_ptr<Octree> children[8];

	Octree(Cube bounds) {
		this->bounds = bounds;
	}

	void insertPointMass(Vec3 point, float mass, bool breakOut = false) {
		if (numMass == 1) {
			centerOfMass = point;
			return;
		}

		float halfSide = bounds.sideLen / 2.0f;
		for (int i = 0; i < 8; i++) {
			Vec3 newOrigin = bounds.origin;
			if (i & 4) newOrigin.z += halfSide; // back
			if (i & 2) newOrigin.y += halfSide; // top
			if (i & 1) newOrigin.x += halfSide; // right
			Cube childBounds(newOrigin, halfSide);

			if (childBounds.containsPoint(point)) {
				if (children[i] == nullptr) {
					children[i] = std::make_unique<Octree>(childBounds);
					numMass++;
				}
				children[i]->insertPointMass(point, mass, true);
			}
		}
	}
};