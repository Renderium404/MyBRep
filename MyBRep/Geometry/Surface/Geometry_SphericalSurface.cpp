#include "Geometry_SphericalSurface.h"

#include <cmath>
#include <limits>

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 圆周率，球面角参数统一使用弧度制。
const double HalfPi = Pi * 0.5; // 球面纬度自然参数域的绝对边界。
const double TwoPi = Pi * 2.0; // 球面经度方向的参数周期。

}

namespace MyBRep
{

Geometry_SphericalSurface::Geometry_SphericalSurface(const MyMath::Vector3& center, double radius)
    : m_center(center)
    , m_xDir(MyMath::Vector3::unitX())
    , m_yDir(MyMath::Vector3::unitY())
    , m_radius(radius)
{
    MYBREP_ASSERT_MESSAGE(center.isFinite(), "Geometry_SphericalSurface center must be finite.");
    MYBREP_ASSERT_MESSAGE(radius > 0.0, "Geometry_SphericalSurface radius must be positive.");
}

Geometry_SphericalSurface::Geometry_SphericalSurface(const MyMath::Vector3& center, double radius,
                                 const MyMath::Vector3& xDir, const MyMath::Vector3& yDir)
    : m_center(center)
    , m_xDir(xDir)
    , m_yDir(yDir)
    , m_radius(radius)
{
    const double epsilon = MyMath::Vector3::DefaultEpsilon; // 球面赤道基向量的单位性和正交性使用数学库默认误差。

    MYBREP_ASSERT_MESSAGE(center.isFinite(), "Geometry_SphericalSurface center must be finite.");
    MYBREP_ASSERT_MESSAGE(radius > 0.0, "Geometry_SphericalSurface radius must be positive.");
    MYBREP_ASSERT_MESSAGE(xDir.isUnit(epsilon), "Geometry_SphericalSurface xDir must be a unit vector.");
    MYBREP_ASSERT_MESSAGE(yDir.isUnit(epsilon), "Geometry_SphericalSurface yDir must be a unit vector.");
    MYBREP_ASSERT_MESSAGE(std::fabs(MyMath::Vector3::dot(xDir, yDir)) <= epsilon,
                          "Geometry_SphericalSurface xDir and yDir must be orthogonal.");
}

Geometry_SphericalSurface::Geometry_SphericalSurface(const MyMath::CoordinateSystem& coordinateSystem, double radius)
    : m_center(coordinateSystem.origin())
    , m_xDir(coordinateSystem.xAxis())
    , m_yDir(coordinateSystem.yAxis())
    , m_radius(radius)
{
    MYBREP_ASSERT_MESSAGE(coordinateSystem.isValid(), "Geometry_SphericalSurface coordinate system must be valid.");
    MYBREP_ASSERT_MESSAGE(radius > 0.0, "Geometry_SphericalSurface radius must be positive.");
}

/// 球面几何数据

const MyMath::Vector3& Geometry_SphericalSurface::center() const
{
    return m_center;
}

const MyMath::Vector3& Geometry_SphericalSurface::xDir() const
{
    return m_xDir;
}

const MyMath::Vector3& Geometry_SphericalSurface::yDir() const
{
    return m_yDir;
}

MyMath::Vector3 Geometry_SphericalSurface::zDir() const
{
    return MyMath::Vector3::cross(m_xDir, m_yDir);
}

double Geometry_SphericalSurface::radius() const
{
    return m_radius;
}

/// 曲面类型

SurfaceKind Geometry_SphericalSurface::kind() const
{
    return SurfaceKind::Spherical;
}

/// U定义域

bool Geometry_SphericalSurface::isUDomainBounded() const
{
    return false;
}

double Geometry_SphericalSurface::uDomainStart() const
{
    return -(std::numeric_limits<double>::infinity)();
}

double Geometry_SphericalSurface::uDomainEnd() const
{
    return (std::numeric_limits<double>::infinity)();
}

/// V定义域

bool Geometry_SphericalSurface::isVDomainBounded() const
{
    return true;
}

double Geometry_SphericalSurface::vDomainStart() const
{
    return -HalfPi;
}

double Geometry_SphericalSurface::vDomainEnd() const
{
    return HalfPi;
}

/// 周期性

double Geometry_SphericalSurface::uPeriod() const
{
    return TwoPi;
}

double Geometry_SphericalSurface::vPeriod() const
{
    return 0.0;
}

/// 参数查询

MyMath::Vector3 Geometry_SphericalSurface::pointAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v), "Geometry_SphericalSurface parameters are outside the natural parameter domain.");

    const MyMath::Vector3 equatorDirection = m_xDir * std::cos(u) + m_yDir * std::sin(u);

    return m_center +
           equatorDirection * (m_radius * std::cos(v)) +
           zDir() * (m_radius * std::sin(v));
}

/// 一阶偏导

MyMath::Vector3 Geometry_SphericalSurface::firstDerivativeUAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v), "Geometry_SphericalSurface derivative parameters are outside the natural parameter domain.");

    return (m_xDir * (-std::sin(u)) + m_yDir * std::cos(u)) *
           (m_radius * std::cos(v));
}

MyMath::Vector3 Geometry_SphericalSurface::firstDerivativeVAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v), "Geometry_SphericalSurface derivative parameters are outside the natural parameter domain.");

    const MyMath::Vector3 equatorDirection = m_xDir * std::cos(u) + m_yDir * std::sin(u);

    return equatorDirection * (-m_radius * std::sin(v)) +
           zDir() * (m_radius * std::cos(v));
}

/// 二阶偏导

MyMath::Vector3 Geometry_SphericalSurface::secondDerivativeUUAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v), "Geometry_SphericalSurface derivative parameters are outside the natural parameter domain.");

    return (m_xDir * (-std::cos(u)) + m_yDir * (-std::sin(u))) *
           (m_radius * std::cos(v));
}

MyMath::Vector3 Geometry_SphericalSurface::secondDerivativeUVAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v), "Geometry_SphericalSurface derivative parameters are outside the natural parameter domain.");

    return (m_xDir * std::sin(u) + m_yDir * (-std::cos(u))) *
           (m_radius * std::sin(v));
}

MyMath::Vector3 Geometry_SphericalSurface::secondDerivativeVVAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v), "Geometry_SphericalSurface derivative parameters are outside the natural parameter domain.");

    const MyMath::Vector3 equatorDirection = m_xDir * std::cos(u) + m_yDir * std::sin(u);

    return equatorDirection * (-m_radius * std::cos(v)) +
           zDir() * (-m_radius * std::sin(v));
}

/// 曲面方向

MyMath::Vector3 Geometry_SphericalSurface::normalAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v), "Geometry_SphericalSurface normal parameters are outside the natural parameter domain.");

    return (m_xDir * std::cos(u) + m_yDir * std::sin(u)) * std::cos(v) +
           zDir() * std::sin(v);
}

}