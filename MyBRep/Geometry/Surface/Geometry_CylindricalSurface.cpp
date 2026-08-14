#include "Geometry_CylindricalSurface.h"

#include <cmath>
#include <limits>

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 圆周率，圆柱U参数统一使用弧度制。
const double TwoPi = Pi * 2.0; // 完整圆柱绕轴方向的参数周期。

}

namespace MyBRep
{

Geometry_CylindricalSurface::Geometry_CylindricalSurface(const MyMath::Vector3& axisOrigin, double radius)
    : m_axisOrigin(axisOrigin)
    , m_xDir(MyMath::Vector3::unitX())
    , m_yDir(MyMath::Vector3::unitY())
    , m_radius(radius)
{
    MYBREP_ASSERT_MESSAGE(axisOrigin.isFinite(), "Geometry_CylindricalSurface axis origin must be finite.");
    MYBREP_ASSERT_MESSAGE(radius > 0.0, "Geometry_CylindricalSurface radius must be positive.");
}

Geometry_CylindricalSurface::Geometry_CylindricalSurface(const MyMath::Vector3& axisOrigin, double radius,
                                     const MyMath::Vector3& xDir, const MyMath::Vector3& yDir)
    : m_axisOrigin(axisOrigin)
    , m_xDir(xDir)
    , m_yDir(yDir)
    , m_radius(radius)
{
    const double epsilon = MyMath::Vector3::DefaultEpsilon; // 圆柱径向基向量的单位性和正交性使用数学库默认误差。

    MYBREP_ASSERT_MESSAGE(axisOrigin.isFinite(), "Geometry_CylindricalSurface axis origin must be finite.");
    MYBREP_ASSERT_MESSAGE(radius > 0.0, "Geometry_CylindricalSurface radius must be positive.");
    MYBREP_ASSERT_MESSAGE(xDir.isUnit(epsilon), "Geometry_CylindricalSurface xDir must be a unit vector.");
    MYBREP_ASSERT_MESSAGE(yDir.isUnit(epsilon), "Geometry_CylindricalSurface yDir must be a unit vector.");
    MYBREP_ASSERT_MESSAGE(std::fabs(MyMath::Vector3::dot(xDir, yDir)) <= epsilon,
                          "Geometry_CylindricalSurface xDir and yDir must be orthogonal.");
}

Geometry_CylindricalSurface::Geometry_CylindricalSurface(const MyMath::CoordinateSystem& coordinateSystem, double radius)
    : m_axisOrigin(coordinateSystem.origin())
    , m_xDir(coordinateSystem.xAxis())
    , m_yDir(coordinateSystem.yAxis())
    , m_radius(radius)
{
    MYBREP_ASSERT_MESSAGE(coordinateSystem.isValid(), "Geometry_CylindricalSurface coordinate system must be valid.");
    MYBREP_ASSERT_MESSAGE(radius > 0.0, "Geometry_CylindricalSurface radius must be positive.");
}

/// 圆柱几何数据

const MyMath::Vector3& Geometry_CylindricalSurface::axisOrigin() const
{
    return m_axisOrigin;
}

const MyMath::Vector3& Geometry_CylindricalSurface::xDir() const
{
    return m_xDir;
}

const MyMath::Vector3& Geometry_CylindricalSurface::yDir() const
{
    return m_yDir;
}

MyMath::Vector3 Geometry_CylindricalSurface::axisDir() const
{
    return MyMath::Vector3::cross(m_xDir, m_yDir);
}

double Geometry_CylindricalSurface::radius() const
{
    return m_radius;
}

/// 曲面类型

SurfaceKind Geometry_CylindricalSurface::kind() const
{
    return SurfaceKind::Cylinder;
}

/// U定义域

bool Geometry_CylindricalSurface::isUDomainBounded() const
{
    return false;
}

double Geometry_CylindricalSurface::uDomainStart() const
{
    return -(std::numeric_limits<double>::infinity)();
}

double Geometry_CylindricalSurface::uDomainEnd() const
{
    return (std::numeric_limits<double>::infinity)();
}

/// V定义域

bool Geometry_CylindricalSurface::isVDomainBounded() const
{
    return false;
}

double Geometry_CylindricalSurface::vDomainStart() const
{
    return -(std::numeric_limits<double>::infinity)();
}

double Geometry_CylindricalSurface::vDomainEnd() const
{
    return (std::numeric_limits<double>::infinity)();
}

/// 周期性

double Geometry_CylindricalSurface::uPeriod() const
{
    return TwoPi;
}

double Geometry_CylindricalSurface::vPeriod() const
{
    return 0.0;
}

/// 参数查询

MyMath::Vector3 Geometry_CylindricalSurface::pointAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v), "Geometry_CylindricalSurface parameters are outside the natural parameter domain.");

    return m_axisOrigin +
           m_xDir * (m_radius * std::cos(u)) +
           m_yDir * (m_radius * std::sin(u)) +
           axisDir() * v;
}

/// 一阶偏导

MyMath::Vector3 Geometry_CylindricalSurface::firstDerivativeUAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v), "Geometry_CylindricalSurface derivative parameters are outside the natural parameter domain.");

    return m_xDir * (-m_radius * std::sin(u)) +
           m_yDir * (m_radius * std::cos(u));
}

MyMath::Vector3 Geometry_CylindricalSurface::firstDerivativeVAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v), "Geometry_CylindricalSurface derivative parameters are outside the natural parameter domain.");
    return axisDir();
}

/// 二阶偏导

MyMath::Vector3 Geometry_CylindricalSurface::secondDerivativeUUAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v), "Geometry_CylindricalSurface derivative parameters are outside the natural parameter domain.");

    return m_xDir * (-m_radius * std::cos(u)) +
           m_yDir * (-m_radius * std::sin(u));
}

MyMath::Vector3 Geometry_CylindricalSurface::secondDerivativeUVAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v), "Geometry_CylindricalSurface derivative parameters are outside the natural parameter domain.");
    return MyMath::Vector3::zero();
}

MyMath::Vector3 Geometry_CylindricalSurface::secondDerivativeVVAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v), "Geometry_CylindricalSurface derivative parameters are outside the natural parameter domain.");
    return MyMath::Vector3::zero();
}

/// 曲面方向

MyMath::Vector3 Geometry_CylindricalSurface::normalAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v), "Geometry_CylindricalSurface normal parameters are outside the natural parameter domain.");
    return m_xDir * std::cos(u) + m_yDir * std::sin(u);
}

}