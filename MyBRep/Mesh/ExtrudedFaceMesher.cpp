#include "ExtrudedFaceMesher.h"

#include "MyBRep/Geometry/Surface/SurfaceKind.h"

namespace MyBRep
{

ExtrudedFaceMeshOptions::ExtrudedFaceMeshOptions()
    : ParametricFaceMeshOptions()
{
}

bool ExtrudedFaceMesher::canMesh(const Topology_Face& face)
{
    if (!face.isValid() || face.geometry().kind() != SurfaceKind::Extrusion)
    {
        return false;
    }

    ParametricFaceMeshPolicy policy;
    policy.periodicU = face.geometry().isUPeriodic();
    policy.periodicV = false;
    policy.rejectSingularParameters = true;

    return ParametricFaceMesherCore::canMesh(face, policy);
}

FaceMesh ExtrudedFaceMesher::mesh(const Topology_Face& face, const ExtrudedFaceMeshOptions& options)
{
    if (!canMesh(face) || !options.isValid())
    {
        return FaceMesh();
    }

    ParametricFaceMeshPolicy policy;
    policy.periodicU = face.geometry().isUPeriodic();
    policy.periodicV = false;
    policy.rejectSingularParameters = true;

    return ParametricFaceMesherCore::mesh(face, options, policy);
}

}
