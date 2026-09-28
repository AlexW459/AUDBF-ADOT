#ifndef MESHINFO_H
#define MESHINFO_H

#include "fvc.H"
#include <utility>

namespace sdfibm {

class MeshInfo
{
protected:
    const Foam::fvMesh& m_mesh;
    const Foam::labelListList& m_c2c; // cell to cell connectivity
    const Foam::labelListList& m_c2p; // cell to point connectivity
    const Foam::pointField   & m_pp; // mesh points
    const Foam::vectorField  & m_cc; // mesh centers
    const Foam::scalarField  & m_cv; // cell volumes
    const Foam::vectorField  & m_fc; // face centers
    const Foam::vectorField  & m_fa; // face areas
public:
    MeshInfo(const Foam::fvMesh& mesh):
        m_mesh(mesh),
        m_c2c(mesh.cellCells()),
        m_c2p(mesh.cellPoints()),
        m_pp(mesh.points()),
        m_cc(mesh.cellCentres()),
        m_cv(mesh.V()),
        m_fc(mesh.faceCentres()),
        m_fa(mesh.faceAreas())
    {}

    // Function to get mesh bounding box min and max points
    std::pair<Foam::point, Foam::point> getMeshBounds() const;
};

}

#endif // MESHINFO_H
