#include "meshinfo.h"
#include "boundBox.H"
#include "point.H"

namespace sdfibm {

std::pair<Foam::point, Foam::point> MeshInfo::getMeshBounds() const
{
    const auto& meshBoundingBox = m_mesh.bounds();
    const auto& minPoint = meshBoundingBox.min();
    const auto& maxPoint = meshBoundingBox.max();
    
    return std::make_pair(minPoint, maxPoint);
}

}
