#include "RevolvedFaceMesher.h"

#include <algorithm>
#include <cmath>

#include "MyBRep/Geometry/Surface/Geometry_SurfaceOfRevolution.h"
#include "MyBRep/Geometry/Surface/SurfaceKind.h"

namespace
{

double nearestInteger(double value)
{
    return value >= 0.0 ? std::floor(value + 0.5) : std::ceil(value - 0.5);
}

double periodicDelta(double first, double second, double period)
{
    double delta = second - first;

    if (period > 0.0)
    {
        delta -= nearestInteger(delta / period) * period;
    }

    return delta;
}

MyMath::Vector3 rotatePerpendicularVector(const MyMath::Vector3& value,
                                          const MyMath::Vector3& axis,
                                          double angle)
{
    return value * std::cos(angle) +MyMath::Vector3::cross(axis, value) * std::sin(angle);
}

class RevolutionSingularityHandler : public MyBRep::ParametricFaceSingularityHandler
{
public:
    bool isSingular(const MyBRep::Geometry_Surface& surface, const MyMath::Vector2& parameter,double tolerance) const override
    {
        if (surface.kind() != MyBRep::SurfaceKind::Revolution)
        {
            return false;
        }

        const MyBRep::Geometry_SurfaceOfRevolution& revolution =static_cast<const MyBRep::Geometry_SurfaceOfRevolution&>(surface);
        const MyMath::Vector3 relative =revolution.profileCurve().pointAt(parameter.y()) - revolution.axisOrigin();
        const MyMath::Vector3 axisComponent =revolution.axisDirection() * MyMath::Vector3::dot(revolution.axisDirection(), relative);
        const MyMath::Vector3 radial = relative - axisComponent;
        const double scale = (std::max)(1.0, relative.length());

        return radial.length() <= tolerance * scale;
    }

    bool parametersMeetAtSingularity(const MyBRep::Geometry_Surface& surface,
                                     const MyMath::Vector2& first,
                                     const MyMath::Vector2& second,
                                     double tolerance) const override
    {
        if (!isSingular(surface, first, tolerance) ||!isSingular(surface, second, tolerance))
        {
            return false;
        }

        const double period = surface.isVPeriodic() ? surface.vPeriod() : 0.0;
        return std::fabs(periodicDelta(first.y(), second.y(), period)) <= tolerance;
    }

    bool normalAt(const MyBRep::Topology_Face& face,
                  const MyMath::Vector2& singularParameter,
                  const MyMath::Vector2& regularApproachParameter,
                  double tolerance,
                  MyMath::Vector3& normal) const override
    {
        if (face.geometry().kind() != MyBRep::SurfaceKind::Revolution)
        {
            return false;
        }

        const MyBRep::Geometry_SurfaceOfRevolution& revolution =static_cast<const MyBRep::Geometry_SurfaceOfRevolution&>(face.geometry());

        if (!isSingular(revolution, singularParameter, tolerance) ||isSingular(revolution, regularApproachParameter, tolerance))
        {
            return false;
        }

        const MyMath::Vector3 tangent =revolution.profileCurve().firstDerivativeAt(singularParameter.y());
        const double axialDerivative =MyMath::Vector3::dot(revolution.axisDirection(), tangent);
        const MyMath::Vector3 radialDerivative = tangent - revolution.axisDirection() * axialDerivative;
        const double radialLength = radialDerivative.length();
        const double tangentScale = (std::max)(1.0, tangent.length());

        // 一阶母线切向必须具有非零径向分量；纯轴向切触属于更高阶奇点，当前v2明确拒绝。
        if (radialLength <= tolerance * tangentScale)
        {
            return false;
        }

        const MyMath::Vector3 rotatedRadialDerivative =rotatePerpendicularVector(radialDerivative, revolution.axisDirection(), singularParameter.x());

        const double vPeriod = revolution.isVPeriodic() ? revolution.vPeriod() : 0.0;
        const double approachDelta =periodicDelta(singularParameter.y(), regularApproachParameter.y(), vPeriod);

        if (std::fabs(approachDelta) <= tolerance)
        {
            return false;
        }

        const double approachSign = approachDelta > 0.0 ? 1.0 : -1.0;

        // 设母线切向T = Tr + Ta*A，则轴点附近：
        // dS/du × dS/dv -> sign(dv) * (Ta*R(Tr) - |Tr|^2*A)。
        normal =(rotatedRadialDerivative * axialDerivative -revolution.axisDirection() * radialDerivative.lengthSquared()) *approachSign;

        if (!normal.isVector(0.0))
        {
            return false;
        }

        normal.normalize(0.0);

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

    RevolutionSingularityHandler singularityHandler;
    ParametricFaceMeshPolicy policy;
    policy.periodicU = true;
    policy.periodicV = face.geometry().isVPeriodic();
    policy.rejectSingularParameters = false;
    policy.singularityHandler = &singularityHandler;

    return ParametricFaceMesherCore::canMesh(face, policy);
}

FaceMesh RevolvedFaceMesher::mesh(const Topology_Face& face, const RevolvedFaceMeshOptions& options)
{
    if (!canMesh(face) || !options.isValid())
    {
        return FaceMesh();
    }

    RevolutionSingularityHandler singularityHandler;
    ParametricFaceMeshPolicy policy;
    policy.periodicU = true;
    policy.periodicV = face.geometry().isVPeriodic();
    policy.rejectSingularParameters = false;
    policy.singularityHandler = &singularityHandler;

    return ParametricFaceMesherCore::mesh(face, options, policy);
}

}
