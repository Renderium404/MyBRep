#include "Geometry_Circle.h"

#include <cmath>
#include <limits>

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 圆周率，圆曲线参数统一使用弧度制。
const double TwoPi = Pi * 2.0; // 完整圆的参数周期。

}

namespace MyBRep
{

Geometry_Circle::Geometry_Circle(const MyMath::Vector3& center, double radius)
    : m_center(center)
    , m_xDir(MyMath::Vector3::unitX())
    , m_yDir(MyMath::Vector3::unitY())
    , m_radius(radius)
{
    MYBREP_ASSERT_MESSAGE(center.isFinite(), "Geometry_Circle center must be finite.");
    MYBREP_ASSERT_MESSAGE(radius > 0.0, "Geometry_Circle radius must be positive.");
}

Geometry_Circle::Geometry_Circle(const MyMath::Vector3& center,
                                 double radius,
                                 const MyMath::Vector3& xDir,
                                 const MyMath::Vector3& yDir)
    : m_center(center)
    , m_xDir(xDir)
    , m_yDir(yDir)
    , m_radius(radius)
{
    const double epsilon = MyMath::Vector3::DefaultEpsilon; // 圆平面方向的单位性和正交性断言使用数学库默认误差。

    MYBREP_ASSERT_MESSAGE(center.isFinite(), "Geometry_Circle center must be finite.");
    MYBREP_ASSERT_MESSAGE(radius > 0.0, "Geometry_Circle radius must be positive.");
    MYBREP_ASSERT_MESSAGE(xDir.isUnit(epsilon), "Geometry_Circle xDir must be a unit vector.");
    MYBREP_ASSERT_MESSAGE(yDir.isUnit(epsilon), "Geometry_Circle yDir must be a unit vector.");
    MYBREP_ASSERT_MESSAGE(std::fabs(MyMath::Vector3::dot(xDir, yDir)) <= epsilon,
                          "Geometry_Circle xDir and yDir must be orthogonal.");
}

Geometry_Circle::Geometry_Circle(const MyMath::CoordinateSystem& coordinateSystem, double radius)
    : m_center(coordinateSystem.origin())
    , m_xDir(coordinateSystem.xAxis())
    , m_yDir(coordinateSystem.yAxis())
    , m_radius(radius)
{
    MYBREP_ASSERT_MESSAGE(coordinateSystem.isValid(), "Geometry_Circle coordinate system must be valid.");
    MYBREP_ASSERT_MESSAGE(radius > 0.0, "Geometry_Circle radius must be positive.");
}

/// 圆几何数据

const MyMath::Vector3& Geometry_Circle::center() const
{
    return m_center;
}

const MyMath::Vector3& Geometry_Circle::xDir() const
{
    return m_xDir;
}

const MyMath::Vector3& Geometry_Circle::yDir() const
{
    return m_yDir;
}

MyMath::Vector3 Geometry_Circle::normal() const
{
    return MyMath::Vector3::cross(m_xDir, m_yDir);
}

double Geometry_Circle::radius() const
{
    return m_radius;
}

/// 曲线类型

CurveKind Geometry_Circle::kind() const
{
    return CurveKind::Circle;
}

/// 定义域

bool Geometry_Circle::isDomainBounded() const
{
    return false;
}

double Geometry_Circle::domainStart() const
{
    return -(std::numeric_limits<double>::infinity)();
}

double Geometry_Circle::domainEnd() const
{
    return (std::numeric_limits<double>::infinity)();
}

/// 周期性

double Geometry_Circle::period() const
{
    return TwoPi;
}

/// 参数查询

MyMath::Vector3 Geometry_Circle::pointAt(double parameter) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(parameter),
                          "Geometry_Circle parameter is outside the natural parameter domain.");

    return m_center +
           m_xDir * (m_radius * std::cos(parameter)) +
           m_yDir * (m_radius * std::sin(parameter));
}

MyMath::Vector3 Geometry_Circle::firstDerivativeAt(double parameter) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(parameter),
                          "Geometry_Circle derivative parameter is outside the natural parameter domain.");

    return m_xDir * (-m_radius * std::sin(parameter)) +
           m_yDir * (m_radius * std::cos(parameter));
}

MyMath::Vector3 Geometry_Circle::secondDerivativeAt(double parameter) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(parameter),
                          "Geometry_Circle derivative parameter is outside the natural parameter domain.");

    return m_xDir * (-m_radius * std::cos(parameter)) +
           m_yDir * (-m_radius * std::sin(parameter));
}

}