#include "RevolvedFaceMesher.h"

#include "MyBRep/Geometry/Surface/SurfaceKind.h"

namespace MyBRep
{

RevolvedFaceMeshOptions::RevolvedFaceMeshOptions()
    : ParametricFaceMeshOptions()
{
}

bool RevolvedFaceMesher::canMesh(const Topology_Face& face)
{
    if (!face.isValid() || face.geometry().kind() != SurfaceKind::Revolution)
    {
        return false;
    }

    ParametricFaceMeshPolicy policy;
    policy.periodicU = true;
    policy.periodicV = face.geometry().isVPeriodic();
    policy.rejectSingularParameters = true;

    return ParametricFaceMesherCore::canMesh(face, policy);
}

FaceMesh RevolvedFaceMesher::mesh(const Topology_Face& face, const RevolvedFaceMeshOptions& options)
{
    if (!canMesh(face) || !options.isValid())
    {
        return FaceMesh();
    }

    ParametricFaceMeshPolicy policy;
    policy.periodicU = true;
    policy.periodicV = face.geometry().isVPeriodic();
    policy.rejectSingularParameters = true;

    return ParametricFaceMesherCore::mesh(face, options, policy);
}

}
