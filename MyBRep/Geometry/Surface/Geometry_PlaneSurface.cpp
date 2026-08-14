#include "Geometry_PlaneSurface.h"

#include <cmath>
#include <limits>

namespace MyBRep
{

Geometry_PlaneSurface::Geometry_PlaneSurface(const MyMath::Vector3& origin)
    : m_origin(origin)
    , m_uDir(MyMath::Vector3::unitX())
    , m_vDir(MyMath::Vector3::unitY())
{
    MYBREP_ASSERT_MESSAGE(origin.isFinite(), "Geometry_PlaneSurface origin must be finite.");
}

Geometry_PlaneSurface::Geometry_PlaneSurface(const MyMath::Vector3& origin, const MyMath::Vector3& uDir, const MyMath::Vector3& vDir)
    : m_origin(origin)
    , m_uDir(uDir)
    , m_vDir(vDir)
{
    const double epsilon = MyMath::Vector3::DefaultEpsilon; // 平面参数方向的单位性和正交性使用数学库默认误差。

    MYBREP_ASSERT_MESSAGE(origin.isFinite(), "Geometry_PlaneSurface origin must be finite.");
    MYBREP_ASSERT_MESSAGE(uDir.isUnit(epsilon), "Geometry_PlaneSurface uDir must be a unit vector.");
    MYBREP_ASSERT_MESSAGE(vDir.isUnit(epsilon), "Geometry_PlaneSurface vDir must be a unit vector.");
    MYBREP_ASSERT_MESSAGE(std::fabs(MyMath::Vector3::dot(uDir, vDir)) <= epsilon,
                          "Geometry_PlaneSurface uDir and vDir must be orthogonal.");
}

Geometry_PlaneSurface::Geometry_PlaneSurface(const MyMath::CoordinateSystem& coordinateSystem)
    : m_origin(coordinateSystem.origin())
    , m_uDir(coordinateSystem.xAxis())
    , m_vDir(coordinateSystem.yAxis())
{
    MYBREP_ASSERT_MESSAGE(coordinateSystem.isValid(), "Geometry_PlaneSurface coordinate system must be valid.");
}

/// 平面几何数据

const MyMath::Vector3& Geometry_PlaneSurface::origin() const
{
    return m_origin;
}

const MyMath::Vector3& Geometry_PlaneSurface::uDir() const
{
    return m_uDir;
}

const MyMath::Vector3& Geometry_PlaneSurface::vDir() const
{
    return m_vDir;
}

MyMath::Vector3 Geometry_PlaneSurface::normal() const
{
    return MyMath::Vector3::cross(m_uDir, m_vDir);
}

/// 曲面类型

SurfaceKind Geometry_PlaneSurface::kind() const
{
    return SurfaceKind::Plane;
}

/// U定义域

bool Geometry_PlaneSurface::isUDomainBounded() const
{
    return false;
}

double Geometry_PlaneSurface::uDomainStart() const
{
    return -(std::numeric_limits<double>::infinity)();
}

double Geometry_PlaneSurface::uDomainEnd() const
{
    return (std::numeric_limits<double>::infinity)();
}

/// V定义域

bool Geometry_PlaneSurface::isVDomainBounded() const
{
    return false;
}

double Geometry_PlaneSurface::vDomainStart() const
{
    return -(std::numeric_limits<double>::infinity)();
}

double Geometry_PlaneSurface::vDomainEnd() const
{
    return (std::numeric_limits<double>::infinity)();
}

/// 周期性

double Geometry_PlaneSurface::uPeriod() const
{
    return 0.0;
}

double Geometry_PlaneSurface::vPeriod() const
{
    return 0.0;
}

/// 参数查询

MyMath::Vector3 Geometry_PlaneSurface::pointAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v), "Geometry_PlaneSurface parameters are outside the natural parameter domain.");
    return m_origin + m_uDir * u + m_vDir * v;
}

/// 一阶偏导

MyMath::Vector3 Geometry_PlaneSurface::firstDerivativeUAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v), "Geometry_PlaneSurface derivative parameters are outside the natural parameter domain.");
    return m_uDir;
}

MyMath::Vector3 Geometry_PlaneSurface::firstDerivativeVAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v), "Geometry_PlaneSurface derivative parameters are outside the natural parameter domain.");
    return m_vDir;
}

/// 二阶偏导

MyMath::Vector3 Geometry_PlaneSurface::secondDerivativeUUAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v), "Geometry_PlaneSurface derivative parameters are outside the natural parameter domain.");
    return MyMath::Vector3::zero();
}

MyMath::Vector3 Geometry_PlaneSurface::secondDerivativeUVAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v), "Geometry_PlaneSurface derivative parameters are outside the natural parameter domain.");
    return MyMath::Vector3::zero();
}

MyMath::Vector3 Geometry_PlaneSurface::secondDerivativeVVAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v), "Geometry_PlaneSurface derivative parameters are outside the natural parameter domain.");
    return MyMath::Vector3::zero();
}

/// 曲面方向

MyMath::Vector3 Geometry_PlaneSurface::normalAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v), "Geometry_PlaneSurface normal parameters are outside the natural parameter domain.");
    return normal();
}

}