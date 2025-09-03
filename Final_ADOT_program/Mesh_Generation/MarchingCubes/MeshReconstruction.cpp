#include "MeshReconstruction.h"
#include "Cube.h"
#include "Triangulation.h"

using namespace MeshReconstruction;
using namespace std;

// Adapted from here: http://paulbourke.net/geometry/polygonise/

namespace
{
	/*void NumGrad(vector<float> const& SDF, glm::ivec3 SDFSize, vector<Vec3>& gradField)
	{
		gradField.resize(SDFSize[0] * SDFSize[1] * SDFSize[2]);

		for(int xi = 1; xi < SDFSize[0] - 1; xi++){
			for(int yi = 1; yi < SDFSize[1] - 1; yi++){
				for(int zi = 1; zi < SDFSize[0] - 1; zi++){
					auto gx = SDF[meshIndexTo1DIndex(xi + 1, yi, zi, SDFSize[0], SDFSize[1])] - 
							  SDF[meshIndexTo1DIndex(xi - 1, yi, zi, SDFSize[0], SDFSize[1])];
					auto gy = SDF[meshIndexTo1DIndex(xi, yi + 1, zi, SDFSize[0], SDFSize[1])] - 
							  SDF[meshIndexTo1DIndex(xi, yi - 1, zi, SDFSize[0], SDFSize[1])];
					auto gz = SDF[meshIndexTo1DIndex(xi, yi, zi + 1, SDFSize[0], SDFSize[1])] - 
							  SDF[meshIndexTo1DIndex(xi, yi, zi - 1, SDFSize[0], SDFSize[1])];
					gradField[meshIndexTo1DIndex(xi, yi, zi, SDFSize[0], SDFSize[1])] = {gx, gy, gz};

				}
			}
		}
	}*/
}


Mesh MeshReconstruction::MarchCube(
	vector<double> const& SDF,
	vector<glm::dvec3> XYZ,
	glm::ivec3 SDFSize,
	double isoLevel)
{

	vector<Vec3> sdfGrad;
	//NumGrad(SDF, SDFSize, sdfGrad);

	auto const NumX = SDFSize[0];
	auto const NumY = SDFSize[1];
	auto const NumZ = SDFSize[2];

	auto const cubeSize = XYZ[1][0] - XYZ[0][0];

	auto const HalfCubeDiag = sqrt(3*cubeSize*cubeSize) / 2.0;

	Mesh mesh;

	for (auto ix = 0; ix < NumX - 1; ++ix)
	{

		for (auto iy = 0; iy < NumY - 1; ++iy)
		{

			for (auto iz = 0; iz < NumZ - 1; ++iz)
			{
				auto x = XYZ[meshIndexTo1DIndex(ix, iy, iz, NumX, NumY)][0];
				auto y = XYZ[meshIndexTo1DIndex(ix, iy, iz, NumX, NumY)][1];
				auto  z = XYZ[meshIndexTo1DIndex(ix, iy, iz, NumX, NumY)][2];

				Vec3 min{ x, y, z };

				double cornerSDF[8] = {
					SDF[meshIndexTo1DIndex(ix, iy, iz, NumX, NumY)],
					SDF[meshIndexTo1DIndex(ix + 1, iy, iz, NumX, NumY)],
					SDF[meshIndexTo1DIndex(ix + 1, iy, iz + 1, NumX, NumY)],
					SDF[meshIndexTo1DIndex(ix, iy, iz + 1, NumX, NumY)],
					SDF[meshIndexTo1DIndex(ix, iy + 1, iz, NumX, NumY)],
					SDF[meshIndexTo1DIndex(ix + 1, iy + 1, iz, NumX, NumY)],
					SDF[meshIndexTo1DIndex(ix + 1, iy + 1, iz + 1, NumX, NumY)],
					SDF[meshIndexTo1DIndex(ix, iy + 1, iz + 1, NumX, NumY)]};


				/*cout << cornerSDF[0] << ", " << cornerSDF[1] << ", " << cornerSDF[2] << 
					", " << cornerSDF[3] << ", " << cornerSDF[4] << ", " << cornerSDF[5] <<
					", " << cornerSDF[6] << ", " << cornerSDF[7] << endl;*/

				auto centreSDF = (cornerSDF[0] + cornerSDF[1] + cornerSDF[2] + 
								  cornerSDF[3] + cornerSDF[4] + cornerSDF[5] +
								  cornerSDF[6] + cornerSDF[7])/8;

				// Process only if cube lies within narrow band around surface.
				auto dist = abs(centreSDF - isoLevel);
				if (dist > HalfCubeDiag) continue;

				Cube cube({ min, {cubeSize, cubeSize, cubeSize} }, cornerSDF);
				
				auto intersect = cube.Intersect(isoLevel);
				Triangulate(intersect, sdfGrad, SDFSize, mesh);
			}
		}
	}

	return mesh;
}
