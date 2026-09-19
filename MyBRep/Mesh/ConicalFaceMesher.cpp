#include "ConicalFaceMesher.h"

#include <cmath>

#include "MyBRep/Geometry/Surface/Geometry_ConicalSurface.h"
#include "MyBRep/Geometry/Surface/SurfaceKind.h"

namespace
{

class ConeSingularityHandler : public MyBRep::ParametricFaceSingularityHandler
{
public:
    bool isSingular(const MyBRep::Geometry_Surface& surface, const MyMath::Vector2& parameter, double tolerance) const override
    {
        if (surface.kind() != MyBRep::SurfaceKind::Conical)
        {
            return false;
        }

        const MyBRep::Geometry_ConicalSurface& cone = static_cast<const MyBRep::Geometry_ConicalSurface&>(surface);
        return parameter.y() <= cone.vDomainStart() + tolerance;
    }

    bool parametersMeetAtSingularity(const MyBRep::Geometry_Surface& surface, const MyMath::Vector2& first,
                                     const MyMath::Vector2& second, double tolerance) const override
    {
        return isSingular(surface, first, tolerance) && isSingular(surface, second, tolerance);
    }

    bool normalAt(const MyBRep::Topology_Face& face, const MyMath::Vector2& singularParameter,
                  const MyMath::Vector2& regularApproachParameter, double tolerance, MyMath::Vector3& normal) const override
    {
        if (face.geometry().kind() != MyBRep::SurfaceKind::Conical)
        {
            return false;
        }

        const MyBRep::Geometry_ConicalSurface& cone = static_cast<const MyBRep::Geometry_ConicalSurface&>(face.geometry());

        if (!isSingular(cone, singularParameter, tolerance) || isSingular(cone, regularApproachParameter, tolerance))
        {
            return false;
        }

        const MyMath::Vector3 radialDir = cone.xDir() * std::cos(singularParameter.x()) + cone.yDir() * std::sin(singularParameter.x());
        normal = (radialDir - cone.axisDir() * cone.radialSlope()).normalized(0.0);

        if (face.isReversed())
        {
            normal *= -1.0;
        }

        return normal.isUnit();
    }
};

}

namespace MyBRep
{

ConicalFaceMeshOptions::ConicalFaceMeshOptions()
    : ParametricFaceMeshOptions()
{
}

bool ConicalFaceMesher::canMesh(const Topology_Face& face)
{
    if (!face.isValid() || face.geometry().kind() != SurfaceKind::Conical)
    {
        return false;
    }

    ConeSingularityHandler singularityHandler;
    ParametricFaceMeshPolicy policy;
    policy.periodicU = true;
    policy.periodicV = false;
    policy.rejectSingularParameters = false;
    policy.singularityHandler = &singularityHandler;

    return ParametricFaceMesherCore::canMesh(face, policy);
}

FaceMesh ConicalFaceMesher::mesh(const Topology_Face& face, const ConicalFaceMeshOptions& options)
{
    if (!canMesh(face) || !options.isValid())
    {
        return FaceMesh();
    }

    ConeSingularityHandler singularityHandler;
    ParametricFaceMeshPolicy policy;
    policy.periodicU = true;
    policy.periodicV = false;
    policy.rejectSingularParameters = false;
    policy.singularityHandler = &singularityHandler;

    return ParametricFaceMesherCore::mesh(face, options, policy);
}

}