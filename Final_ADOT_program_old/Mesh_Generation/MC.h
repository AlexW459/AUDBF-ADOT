//https://github.com/aparis69/MarchingCubeCpp
#pragma once

#include <vector>
#include <array>
#include <cmath>
#include <fstream>
#include <glm/glm.hpp>


namespace MC
{

	typedef unsigned int muint;

	typedef struct mcMesh
	{
	public:
		std::vector<glm::dvec3> vertices;
		std::vector<glm::dvec3> normals;
		std::vector<muint> indices;
	} mcMesh;

	
	muint mc_internalToIndex1D(muint i, muint j, muint k, const glm::ivec3& size);
	muint mc_internalToIndex1DSlab(muint i, muint j, muint k, const glm::ivec3& size);
	void mc_internalComputeEdge(std::vector<glm::ivec3>& slab_inds, mcMesh& mesh, double va, 
		double vb, int axis, muint x, muint y, muint z, const glm::ivec3& size);
	void setDefaultArraySizes(muint vertSize, muint normSize, muint triSize);

	void marching_cube(const std::vector<double>& field, muint nx, muint ny, muint nz, mcMesh& outputMesh);
}

void writeMeshToObj(std::string filename, MC::mcMesh mesh);
