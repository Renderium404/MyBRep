#include "BezierFaceMesher.h"

#include "MyBRep/Geometry/Surface/SurfaceKind.h"

namespace MyBRep
{

BezierFaceMeshOptions::BezierFaceMeshOptions()
    : ParametricFaceMeshOptions()
{
}

bool BezierFaceMesher::canMesh(const Topology_Face& face)
{
    if (!face.isValid() || face.geometry().kind() != SurfaceKind::Bezier)
    {
        return false;
    }

    ParametricFaceMeshPolicy policy;
    policy.periodicU = false;
    policy.periodicV = false;
    policy.rejectSingularParameters = true;

    return ParametricFaceMesherCore::canMesh(face, policy);
}

FaceMesh BezierFaceMesher::mesh(const Topology_Face& face, const BezierFaceMeshOptions& options)
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