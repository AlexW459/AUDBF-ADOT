#pragma once
#include "Cube.h"
#include "DataStructs.h"

namespace MeshReconstruction
{
	void Triangulate(
		IntersectInfo const& intersect,
		vector<Vec3> const& grad,
		glm::ivec3 SDFSize,
		Mesh& mesh);
}