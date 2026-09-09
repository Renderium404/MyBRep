#include "FaceMesher.h"

#include "MyBRep/Geometry/Surface/SurfaceKind.h"

namespace MyBRep
{

FaceMeshOptions::FaceMeshOptions()
{
}

bool FaceMeshOptions::isValid() const
{
    return planar.isValid() &&
           cylindrical.isValid() &&
           spherical.isValid() &&
           conical.isValid() &&
           extruded.isValid() &&
           revolved.isValid();
}

bool FaceMesher::canMesh(const Topology_Face& face)
{
    if (!face.isValid())
    {
        return false;
    }

    switch (face.geometry().kind())
    {
    case SurfaceKind::Plane:
        return PlanarFaceMesher::canMesh(face);

    case SurfaceKind::Cylindrical:
        return CylindricalFaceMesher::canMesh(face);

    case SurfaceKind::Spherical:
        return SphericalFaceMesher::canMesh(face);

    case SurfaceKind::Conical:
        return ConicalFaceMesher::canMesh(face);

    case SurfaceKind::Extrusion:
        return ExtrudedFaceMesher::canMesh(face);

    case SurfaceKind::Revolution:
        return RevolvedFaceMesher::canMesh(face);

    default:
        return false;
    }
}

FaceMesh FaceMesher::mesh(const Topology_Face& face, const FaceMeshOptions& options)
{
    if (!face.isValid() || !options.isValid())
    {
        return FaceMesh();
    }

    switch (face.geometry().kind())
    {
    case SurfaceKind::Plane:
        return PlanarFaceMesher::mesh(face, options.planar);

    case SurfaceKind::Cylindrical:
        return CylindricalFaceMesher::mesh(face, options.cylindrical);

    case SurfaceKind::Spherical:
        return SphericalFaceMesher::mesh(face, options.spherical);

    case SurfaceKind::Conical:
        return ConicalFaceMesher::mesh(face, options.conical);

    case SurfaceKind::Extrusion:
        return ExtrudedFaceMesher::mesh(face, options.extruded);

    case SurfaceKind::Revolution:
        return RevolvedFaceMesher::mesh(face, options.revolved);

    default:
        return FaceMesh();
    }
}

}
