#include "Geometry_Circle2D.h"

#include <cmath>
#include <limits>

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 圆周率，二维圆参数统一使用弧度制。
const double TwoPi = Pi * 2.0; // 完整二维圆参数周期。

// 判断标量是否为有限值。
bool isFiniteValue(double value)
{
    const double infinity = (std::numeric_limits<double>::infinity)();
    return value == value && value != infinity && value != -infinity;
}

}

namespace MyBRep
{

Geometry_Circle2D::Geometry_Circle2D(const MyMath::Vector2& center, double radius)
    : m_center(center)
    , m_xDir(MyMath::Vector2::unitX())
    , m_yDir(MyMath::Vector2::unitY())
    , m_radius(radius)
{
    MYBREP_ASSERT_MESSAGE(center.isFinite(),
                          "Geometry_Circle2D center must be finite.");
    MYBREP_ASSERT_MESSAGE(isFiniteValue(radius) && radius > 0.0,
                          "Geometry_Circle2D radius must be finite and positive.");
}

Geometry_Circle2D::Geometry_Circle2D(const MyMath::Vector2& center,
                                     double radius,
                                     const MyMath::Vector2& xDir,
                                     const MyMath::Vector2& yDir)
    : m_center(center)
    , m_xDir(xDir)
    , m_yDir(yDir)
    , m_radius(radius)
{
    const double epsilon = MyMath::Vector2::DefaultEpsilon; // 二维参数基方向的单位性和正交性使用数学库默认误差。

    MYBREP_ASSERT_MESSAGE(center.isFinite(),
                          "Geometry_Circle2D center must be finite.");
    MYBREP_ASSERT_MESSAGE(isFiniteValue(radius) && radius > 0.0,
                          "Geometry_Circle2D radius must be finite and positive.");
    MYBREP_ASSERT_MESSAGE(xDir.isUnit(epsilon),
                          "Geometry_Circle2D xDir must be a unit vector.");
    MYBREP_ASSERT_MESSAGE(yDir.isUnit(epsilon),
                          "Geometry_Circle2D yDir must be a unit vector.");
    MYBREP_ASSERT_MESSAGE(std::fabs(MyMath::Vector2::dot(xDir, yDir)) <= epsilon,
                          "Geometry_Circle2D xDir and yDir must be orthogonal.");
    MYBREP_ASSERT_MESSAGE(std::fabs(std::fabs(MyMath::Vector2::cross(xDir, yDir)) - 1.0) <= epsilon,
                          "Geometry_Circle2D xDir and yDir must form an orthonormal basis.");
}

/// 圆几何数据

const MyMath::Vector2& Geometry_Circle2D::center() const
{
    return m_center;
}

const MyMath::Vector2& Geometry_Circle2D::xDir() const
{
    return m_xDir;
}

const MyMath::Vector2& Geometry_Circle2D::yDir() const
{
    return m_yDir;
}

double Geometry_Circle2D::orientationSign() const
{
    return MyMath::Vector2::cross(m_xDir, m_yDir);
}

double Geometry_Circle2D::radius() const
{
    return m_radius;
}

/// 曲线类型

CurveKind Geometry_Circle2D::kind() const
{
    return CurveKind::Circle;
}

/// 定义域

bool Geometry_Circle2D::isDomainBounded() const
{
    return false;
}

double Geometry_Circle2D::domainStart() const
{
    return -(std::numeric_limits<double>::infinity)();
}

double Geometry_Circle2D::domainEnd() const
{
    return (std::numeric_limits<double>::infinity)();
}

/// 周期性

double Geometry_Circle2D::period() const
{
    return TwoPi;
}

/// 参数查询

MyMath::Vector2 Geometry_Circle2D::pointAt(double parameter) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(parameter),
                          "Geometry_Circle2D parameter is outside the natural parameter domain.");

    return m_center +
           m_xDir * (m_radius * std::cos(parameter)) +
           m_yDir * (m_radius * std::sin(parameter));
}

MyMath::Vector2 Geometry_Circle2D::firstDerivativeAt(double parameter) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(parameter),
                          "Geometry_Circle2D derivative parameter is outside the natural parameter domain.");

    return m_xDir * (-m_radius * std::sin(parameter)) +
           m_yDir * (m_radius * std::cos(parameter));
}

MyMath::Vector2 Geometry_Circle2D::secondDerivativeAt(double parameter) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(parameter),
                          "Geometry_Circle2D derivative parameter is outside the natural parameter domain.");

    return m_xDir * (-m_radius * std::cos(parameter)) +
           m_yDir * (-m_radius * std::sin(parameter));
}

}
