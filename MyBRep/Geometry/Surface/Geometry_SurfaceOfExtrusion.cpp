#include "Geometry_SurfaceOfExtrusion.h"

#include <limits>

namespace MyBRep
{

Geometry_SurfaceOfExtrusion::Geometry_SurfaceOfExtrusion(
    const Foundation::RefPtr<const Geometry_Curve>& profileCurve,
    const MyMath::Vector3& direction)
    : m_profileCurve(profileCurve)
    , m_direction(direction)
{
    MYBREP_ASSERT_MESSAGE(m_profileCurve,
                          "Geometry_SurfaceOfExtrusion requires a non-null profile curve.");
    MYBREP_ASSERT_MESSAGE(direction.isUnit(MyMath::Vector3::DefaultEpsilon),
                          "Geometry_SurfaceOfExtrusion direction must be a unit vector.");
}

/// 拉伸几何数据

const Geometry_Curve& Geometry_SurfaceOfExtrusion::profileCurve() const
{
    MYBREP_ASSERT_MESSAGE(m_profileCurve,
                          "Geometry_SurfaceOfExtrusion profile curve must not be null.");

    return *m_profileCurve;
}

const Foundation::RefPtr<const Geometry_Curve>& Geometry_SurfaceOfExtrusion::profileCurveResource() const
{
    return m_profileCurve;
}

const MyMath::Vector3& Geometry_SurfaceOfExtrusion::direction() const
{
    return m_direction;
}

/// 曲面类型

SurfaceKind Geometry_SurfaceOfExtrusion::kind() const
{
    return SurfaceKind::Extrusion;
}

/// U定义域

bool Geometry_SurfaceOfExtrusion::isUDomainBounded() const
{
    return profileCurve().isDomainBounded();
}

double Geometry_SurfaceOfExtrusion::uDomainStart() const
{
    return profileCurve().domainStart();
}

double Geometry_SurfaceOfExtrusion::uDomainEnd() const
{
    return profileCurve().domainEnd();
}

/// V定义域

bool Geometry_SurfaceOfExtrusion::isVDomainBounded() const
{
    return false;
}

double Geometry_SurfaceOfExtrusion::vDomainStart() const
{
    return -(std::numeric_limits<double>::infinity)();
}

double Geometry_SurfaceOfExtrusion::vDomainEnd() const
{
    return (std::numeric_limits<double>::infinity)();
}

/// 周期性

double Geometry_SurfaceOfExtrusion::uPeriod() const
{
    return profileCurve().period();
}

double Geometry_SurfaceOfExtrusion::vPeriod() const
{
    return 0.0;
}

/// 参数查询

MyMath::Vector3 Geometry_SurfaceOfExtrusion::pointAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                          "Geometry_SurfaceOfExtrusion parameters are outside the natural parameter domain.");

    return profileCurve().pointAt(u) + m_direction * v;
}

/// 一阶偏导

MyMath::Vector3 Geometry_SurfaceOfExtrusion::firstDerivativeUAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                          "Geometry_SurfaceOfExtrusion derivative parameters are outside the natural parameter domain.");

    return profileCurve().firstDerivativeAt(u);
}

MyMath::Vector3 Geometry_SurfaceOfExtrusion::firstDerivativeVAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                          "Geometry_SurfaceOfExtrusion derivative parameters are outside the natural parameter domain.");

    return m_direction;
}

/// 二阶偏导

MyMath::Vector3 Geometry_SurfaceOfExtrusion::secondDerivativeUUAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                          "Geometry_SurfaceOfExtrusion derivative parameters are outside the natural parameter domain.");

    return profileCurve().secondDerivativeAt(u);
}

MyMath::Vector3 Geometry_SurfaceOfExtrusion::secondDerivativeUVAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                          "Geometry_SurfaceOfExtrusion derivative parameters are outside the natural parameter domain.");

    return MyMath::Vector3::zero();
}

MyMath::Vector3 Geometry_SurfaceOfExtrusion::secondDerivativeVVAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                          "Geometry_SurfaceOfExtrusion derivative parameters are outside the natural parameter domain.");

    return MyMath::Vector3::zero();
}

}