#include "BSplineFaceMesher.h"

#include "MyBRep/Geometry/Surface/SurfaceKind.h"

namespace MyBRep
{

BSplineFaceMeshOptions::BSplineFaceMeshOptions()
    : ParametricFaceMeshOptions()
{
}

bool BSplineFaceMesher::canMesh(const Topology_Face& face)
{
    if (!face.isValid() || face.geometry().kind() != SurfaceKind::BSpline)
    {
        return false;
    }

    ParametricFaceMeshPolicy policy;
    policy.periodicU = false;
    policy.periodicV = false;
    policy.rejectSingularParameters = true;

    return ParametricFaceMesherCore::canMesh(face, policy);
}

FaceMesh BSplineFaceMesher::mesh(const Topology_Face& face, const BSplineFaceMeshOptions& options)
{
    if (!canMesh(face) || !options.isValid())
    {
        return FaceMesh();
    }

    ParametricFaceMeshPolicy policy;
    policy.periodicU = false;
    policy.periodicV = false;
    policy.rejectSingularParameters = true;

    return ParametricFaceMesherCore::mesh(face, options, policy);
}

}