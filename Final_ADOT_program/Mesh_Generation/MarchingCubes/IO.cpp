#include "IO.h"
using namespace std;

void MeshReconstruction::WriteObjFile(Mesh const& mesh, string const& fileName)
{
	// FILE faster than streams.

	std::ofstream file;

	file.open(fileName);

	if (file.is_open() == false)
	{
		throw runtime_error("Could not write obj file.");
	}

	// write stats
	file << "# " << static_cast<int>(mesh.vertices.size()) << " vertices, " << static_cast<int>(mesh.triangles.size()) << " triangles\n\n";

	// vertices
	for (int vi = 0; vi < (int)mesh.vertices.size(); ++vi)
	{
		auto const& v = mesh.vertices.at(vi);
		file << "v " << v.x << " " << v.y << " " << v.z << "\n";
	}

	// vertex normals
	/*file << "\n";
	for (auto ni = 0; ni < mesh.vertices.size(); ++ni)
	{
		auto const& vn = mesh.vertexNormals.at(ni);
		file << "vn " << vn.x << " " << vn.y << " " << vn.z << "\n";
	}*/

	// triangles (1-based)
	file << "\n";
	/*for (auto ti = 0; ti < mesh.triangles.size(); ++ti)
	{
		auto const& t = mesh.triangles.at(ti);
		file << "f " << t[0] + 1 << "//" << t[0] + 1 << " " <<
			 			t[1] + 1 << "//" << t[1] + 1 << " " <<
						t[2] + 1 << "//" << t[2] + 1 << "\n"; 
	}*/

	for (int ti = 0; ti < (int)mesh.triangles.size(); ++ti)
	{
		auto const& t = mesh.triangles.at(ti);
		file << "f " << t[0] + 1 << " " <<
			 			t[1] + 1 << " " <<
						t[2] + 1 << "\n"; 
	}

	file.close();
}