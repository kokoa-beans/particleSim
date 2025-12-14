#pragma once

#include <vector>

#include "Body.h"
#include "Cube.h"


struct OctreeNode {
	Cube boundingCube;
	int numAdded;

	Vec3 centerOfMass;
	double mass;

	OctreeNode* nodes[8];

	static Body defaultBodyPlaceholder;
	Body& thisBody;

	OctreeNode() : boundingCube(), numAdded(0), centerOfMass(), mass(0), thisBody(defaultBodyPlaceholder){
		for (int i = 0; i < 8; i++) nodes[i] = nullptr;
	};

	OctreeNode(double x, double y, double z, double w) : OctreeNode() {
		boundingCube = Cube(x, y, z, w);
	}

	/* Adds the body recursively into the 8-subsections.
	*/
	void addBody(Body& body) {
		numAdded++;

		/*
			If there is exactly 2 bodies, its a special case which requires the original mass to added to another node.
			At this point, it should be guaranteed that the class' COM + mass stores the singular COM + mass from the first mass.
		*/

		if (numAdded == 2) {

			//The "pseu-body" from the original singular mass. The velocity information (all 0s) can be ommited.
			Body tempBody = Body(centerOfMass.x, centerOfMass.y, centerOfMass.z, 0, 0, 0, mass);
			
			partitionAndAddBody(tempBody);
		}

		//If there is more than or exactly 2 body, then you have to recursively add it until there is a node with only 1.
		if (numAdded >= 2) {
			partitionAndAddBody(body);
		}
		

		centerOfMass = body.pos;
		mass += body.mass;
		thisBody = body;
	}

	/*
		Handles the partitioning of the Octree, and inserts it into the correct partition.
		If the partition does not exist yet, creates it and then adds it.
	*/
	void partitionAndAddBody(Body body) {
		int ind = 0;

		bool xLowerHalf = body.pos.x <= (boundingCube.origin.x + boundingCube.width / 2.0);
		bool yLowerHalf = body.pos.y <= (boundingCube.origin.y + boundingCube.width / 2.0);
		bool zLowerHalf = body.pos.z <= (boundingCube.origin.z + boundingCube.width / 2.0);


		//Check if the x-component is in the lower half, set the 3rd bit from the left if so.
		ind |= ((xLowerHalf) ? 1 : 0) << 2;

		//Check if the y-component is in the lower half, set the 2nd bit from the left if so.
		ind |= ((yLowerHalf) ? 1 : 0) << 1;

		//Check if the z-component is in the lower half, set the 1st bit from the left if so.
		ind |= ((zLowerHalf) ? 1 : 0) << 0;

		if (nodes[ind] == nullptr) {
			double xPos = boundingCube.origin.x + (xLowerHalf ? 0.0 : boundingCube.width / 2.0);
			double yPos = boundingCube.origin.y + (yLowerHalf ? 0.0 : boundingCube.width / 2.0);
			double zPos = boundingCube.origin.z + (zLowerHalf ? 0.0 : boundingCube.width / 2.0);


			nodes[ind] = new OctreeNode(xPos, yPos, zPos, boundingCube.width / 2.0);
		}

		nodes[ind]->addBody(body);
	}

	/* Updates the COM for each OctreeNode recursively.
	*/
	void calculateCOM() {
		if (numAdded > 1) {

			Vec3 tempCenterOfMass = Vec3(0, 0, 0);
			double totalMass = 0.0;

			for (OctreeNode* checkingNode : nodes) {
				if (checkingNode != nullptr) {
					checkingNode->calculateCOM();
					tempCenterOfMass += (checkingNode->centerOfMass) * mass;
					totalMass += mass;
				}
			}

			centerOfMass = tempCenterOfMass / totalMass;
			mass = totalMass;
		}
	}

	bool isLeaf() {
		return numAdded == 1;
	}
};

struct Octree
{
	OctreeNode* root;

	Octree() {
		root = new OctreeNode();
	}

	/*
	The Octree's inital size is [-size/2 to size/2] ^ 3. Adds the bodies and calculates COM.
	*/
	Octree(std::vector<Body>& bodies, double size) {
		root = new OctreeNode(-size / 2, -size / 2, -size / 2, size);

		for (Body& each : bodies) {
			if (root->boundingCube.containsPoint(each.pos)) root->addBody(each);
		}

		for (Body& each : bodies) {
			if (root->boundingCube.containsPoint(each.pos)) root->calculateCOM();
		}
	}
};