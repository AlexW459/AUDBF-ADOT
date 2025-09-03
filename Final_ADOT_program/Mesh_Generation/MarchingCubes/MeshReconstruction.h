#pragma once
#include <glm/glm.hpp>
#include "DataStructs.h"

namespace MeshReconstruction
{
	/// Reconstructs a triangle mesh from a given signed distance function using <a href="https://en.wikipedia.org/wiki/Marching_cubes">Marching Cubes</a>.
	/// @param sdf The <a href="http://www.iquilezles.org/www/articles/distfunctions/distfunctions.htm">Signed Distance Function</a>.
	/// @param domain Domain of reconstruction.
	/// @returns The reconstructed mesh.
	Mesh MarchCube(
		Fun3s const& sdf,
		Rect3 const& domain);

	/// Reconstructs a triangle mesh from a given signed distance function using <a href="https://en.wikipedia.org/wiki/Marching_cubes">Marching Cubes</a>.
	/// @param sdf The <a href="http://www.iquilezles.org/www/articles/distfunctions/distfunctions.htm">Signed Distance Function</a>.
	/// @param cubeSize Size of marching cubes. Smaller cubes yields meshes of higher resolution.
	/// @param isoLevel Level set of the SDF for which triangulation should be done. Changing this value moves the reconstructed surface.
	/// @returns The reconstructed mesh.
	Mesh MarchCube(
		vector<double> const& SDF,
		vector<glm::dvec3> XYZ,
		glm::ivec3 SDFSize,
		double isoLevel = 0);
}