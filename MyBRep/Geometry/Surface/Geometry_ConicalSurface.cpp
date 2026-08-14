#include "Geometry_ConicalSurface.h"

#include <cmath>
#include <limits>

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 圆周率，圆锥角参数统一使用弧度制。
const double HalfPi = Pi * 0.5; // 合法圆锥半顶角的开区间上界。
const double TwoPi = Pi * 2.0; // 完整圆锥绕轴方向的参数周期。

}

namespace MyBRep
{

Geometry_ConicalSurface::Geometry_ConicalSurface(const MyMath::Vector3& apex, double semiAngle)
    : m_apex(apex)
    , m_xDir(MyMath::Vector3::unitX())
    , m_yDir(MyMath::Vector3::unitY())
    , m_semiAngle(semiAngle)
    , m_radialSlope(std::tan(semiAngle))
{
    MYBREP_ASSERT_MESSAGE(apex.isFinite(),
                          "Geometry_ConicalSurface apex must be finite.");
    MYBREP_ASSERT_MESSAGE(semiAngle > 0.0 && semiAngle < HalfPi,
                          "Geometry_ConicalSurface semi-angle must be in (0, pi/2).");
}

Geometry_ConicalSurface::Geometry_ConicalSurface(const MyMath::Vector3& apex,
                                                 double semiAngle,
                                                 const MyMath::Vector3& xDir,
                                                 const MyMath::Vector3& yDir)
    : m_apex(apex)
    , m_xDir(xDir)
    , m_yDir(yDir)
    , m_semiAngle(semiAngle)
    , m_radialSlope(std::tan(semiAngle))
{
    const double epsilon = MyMath::Vector3::DefaultEpsilon; // 圆锥径向基向量的单位性和正交性使用数学库默认误差。

    MYBREP_ASSERT_MESSAGE(apex.isFinite(),
                          "Geometry_ConicalSurface apex must be finite.");
    MYBREP_ASSERT_MESSAGE(semiAngle > 0.0 && semiAngle < HalfPi,
                          "Geometry_ConicalSurface semi-angle must be in (0, pi/2).");
    MYBREP_ASSERT_MESSAGE(xDir.isUnit(epsilon),
                          "Geometry_ConicalSurface xDir must be a unit vector.");
    MYBREP_ASSERT_MESSAGE(yDir.isUnit(epsilon),
                          "Geometry_ConicalSurface yDir must be a unit vector.");
    MYBREP_ASSERT_MESSAGE(std::fabs(MyMath::Vector3::dot(xDir, yDir)) <= epsilon,
                          "Geometry_ConicalSurface xDir and yDir must be orthogonal.");
}

Geometry_ConicalSurface::Geometry_ConicalSurface(const MyMath::CoordinateSystem& coordinateSystem, double semiAngle)
    : m_apex(coordinateSystem.origin())
    , m_xDir(coordinateSystem.xAxis())
    , m_yDir(coordinateSystem.yAxis())
    , m_semiAngle(semiAngle)
    , m_radialSlope(std::tan(semiAngle))
{
    MYBREP_ASSERT_MESSAGE(coordinateSystem.isValid(),
                          "Geometry_ConicalSurface coordinate system must be valid.");
    MYBREP_ASSERT_MESSAGE(semiAngle > 0.0 && semiAngle < HalfPi,
                          "Geometry_ConicalSurface semi-angle must be in (0, pi/2).");
}

/// 圆锥几何数据

const MyMath::Vector3& Geometry_ConicalSurface::apex() const
{
    return m_apex;
}

const MyMath::Vector3& Geometry_ConicalSurface::xDir() const
{
    return m_xDir;
}

const MyMath::Vector3& Geometry_ConicalSurface::yDir() const
{
    return m_yDir;
}

MyMath::Vector3 Geometry_ConicalSurface::axisDir() const
{
    return MyMath::Vector3::cross(m_xDir, m_yDir);
}

double Geometry_ConicalSurface::semiAngle() const
{
    return m_semiAngle;
}

double Geometry_ConicalSurface::radialSlope() const
{
    return m_radialSlope;
}

/// 曲面类型

SurfaceKind Geometry_ConicalSurface::kind() const
{
    return SurfaceKind::Conical;
}

/// U定义域

bool Geometry_ConicalSurface::isUDomainBounded() const
{
    return false;
}

double Geometry_ConicalSurface::uDomainStart() const
{
    return -(std::numeric_limits<double>::infinity)();
}

double Geometry_ConicalSurface::uDomainEnd() const
{
    return (std::numeric_limits<double>::infinity)();
}

/// V定义域

bool Geometry_ConicalSurface::isVDomainBounded() const
{
    return false;
}

double Geometry_ConicalSurface::vDomainStart() const
{
    return 0.0;
}

double Geometry_ConicalSurface::vDomainEnd() const
{
    return (std::numeric_limits<double>::infinity)();
}

/// 周期性

double Geometry_ConicalSurface::uPeriod() const
{
    return TwoPi;
}

double Geometry_ConicalSurface::vPeriod() const
{
    return 0.0;
}

/// 参数查询

MyMath::Vector3 Geometry_ConicalSurface::pointAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                          "Geometry_ConicalSurface parameters are outside the natural parameter domain.");

    const MyMath::Vector3 radialDir = m_xDir * std::cos(u) + m_yDir * std::sin(u);

    return m_apex +
           axisDir() * v +
           radialDir * (v * m_radialSlope);
}

/// 一阶偏导

MyMath::Vector3 Geometry_ConicalSurface::firstDerivativeUAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                          "Geometry_ConicalSurface derivative parameters are outside the natural parameter domain.");

    const MyMath::Vector3 tangentDir = m_xDir * (-std::sin(u)) + m_yDir * std::cos(u);
    return tangentDir * (v * m_radialSlope);
}

MyMath::Vector3 Geometry_ConicalSurface::firstDerivativeVAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                          "Geometry_ConicalSurface derivative parameters are outside the natural parameter domain.");

    const MyMath::Vector3 radialDir = m_xDir * std::cos(u) + m_yDir * std::sin(u);
    return axisDir() + radialDir * m_radialSlope;
}

/// 二阶偏导

MyMath::Vector3 Geometry_ConicalSurface::secondDerivativeUUAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                          "Geometry_ConicalSurface derivative parameters are outside the natural parameter domain.");

    const MyMath::Vector3 radialDir = m_xDir * std::cos(u) + m_yDir * std::sin(u);
    return radialDir * (-v * m_radialSlope);
}

MyMath::Vector3 Geometry_ConicalSurface::secondDerivativeUVAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                          "Geometry_ConicalSurface derivative parameters are outside the natural parameter domain.");

    const MyMath::Vector3 tangentDir = m_xDir * (-std::sin(u)) + m_yDir * std::cos(u);
    return tangentDir * m_radialSlope;
}

MyMath::Vector3 Geometry_ConicalSurface::secondDerivativeVVAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                          "Geometry_ConicalSurface derivative parameters are outside the natural parameter domain.");

    return MyMath::Vector3::zero();
}

/// 曲面方向

MyMath::Vector3 Geometry_ConicalSurface::normalAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                          "Geometry_ConicalSurface normal parameters are outside the natural parameter domain.");
    MYBREP_ASSERT_MESSAGE(v > 0.0,
                          "Geometry_ConicalSurface normal is undefined at the cone apex.");

    const MyMath::Vector3 radialDir = m_xDir * std::cos(u) + m_yDir * std::sin(u);
    const MyMath::Vector3 normal = radialDir - axisDir() * m_radialSlope;
    return normal.normalized(0.0);
}

}