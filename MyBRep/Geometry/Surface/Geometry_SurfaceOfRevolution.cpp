#include "Geometry_SurfaceOfRevolution.h"

#include <cmath>
#include <limits>

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 圆周率，旋转曲面U参数统一使用弧度制。
const double TwoPi = Pi * 2.0; // 完整旋转曲面的旋转角参数周期。

}

namespace MyBRep
{

Geometry_SurfaceOfRevolution::Geometry_SurfaceOfRevolution(
    const Foundation::RefPtr<const Geometry_Curve>& profileCurve,
    const MyMath::Vector3& axisOrigin,
    const MyMath::Vector3& axisDirection)
    : m_profileCurve(profileCurve)
    , m_axisOrigin(axisOrigin)
    , m_axisDirection(axisDirection)
{
    MYBREP_ASSERT_MESSAGE(m_profileCurve,
                          "Geometry_SurfaceOfRevolution requires a non-null profile curve.");
    MYBREP_ASSERT_MESSAGE(axisOrigin.isFinite(),
                          "Geometry_SurfaceOfRevolution axis origin must be finite.");
    MYBREP_ASSERT_MESSAGE(axisDirection.isUnit(MyMath::Vector3::DefaultEpsilon),
                          "Geometry_SurfaceOfRevolution axis direction must be a unit vector.");
}

/// 旋转几何数据

const Geometry_Curve& Geometry_SurfaceOfRevolution::profileCurve() const
{
    MYBREP_ASSERT_MESSAGE(m_profileCurve,
                          "Geometry_SurfaceOfRevolution profile curve must not be null.");

    return *m_profileCurve;
}

const Foundation::RefPtr<const Geometry_Curve>& Geometry_SurfaceOfRevolution::profileCurveResource() const
{
    return m_profileCurve;
}

const MyMath::Vector3& Geometry_SurfaceOfRevolution::axisOrigin() const
{
    return m_axisOrigin;
}

const MyMath::Vector3& Geometry_SurfaceOfRevolution::axisDirection() const
{
    return m_axisDirection;
}

/// 曲面类型

SurfaceKind Geometry_SurfaceOfRevolution::kind() const
{
    return SurfaceKind::Revolution;
}

/// U定义域

bool Geometry_SurfaceOfRevolution::isUDomainBounded() const
{
    return false;
}

double Geometry_SurfaceOfRevolution::uDomainStart() const
{
    return -(std::numeric_limits<double>::infinity)();
}

double Geometry_SurfaceOfRevolution::uDomainEnd() const
{
    return (std::numeric_limits<double>::infinity)();
}

/// V定义域

bool Geometry_SurfaceOfRevolution::isVDomainBounded() const
{
    return profileCurve().isDomainBounded();
}

double Geometry_SurfaceOfRevolution::vDomainStart() const
{
    return profileCurve().domainStart();
}

double Geometry_SurfaceOfRevolution::vDomainEnd() const
{
    return profileCurve().domainEnd();
}

/// 周期性

double Geometry_SurfaceOfRevolution::uPeriod() const
{
    return TwoPi;
}

double Geometry_SurfaceOfRevolution::vPeriod() const
{
    return profileCurve().period();
}

/// 参数查询

MyMath::Vector3 Geometry_SurfaceOfRevolution::pointAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                          "Geometry_SurfaceOfRevolution parameters are outside the natural parameter domain.");

    const MyMath::Vector3 relativePoint = profileCurve().pointAt(v) - m_axisOrigin;

    return m_axisOrigin + rotateVector(relativePoint, u);
}

/// 一阶偏导

MyMath::Vector3 Geometry_SurfaceOfRevolution::firstDerivativeUAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                          "Geometry_SurfaceOfRevolution derivative parameters are outside the natural parameter domain.");

    const MyMath::Vector3 relativePoint = profileCurve().pointAt(v) - m_axisOrigin;

    return rotateVectorFirstDerivative(relativePoint, u);
}

MyMath::Vector3 Geometry_SurfaceOfRevolution::firstDerivativeVAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                          "Geometry_SurfaceOfRevolution derivative parameters are outside the natural parameter domain.");

    return rotateVector(profileCurve().firstDerivativeAt(v), u);
}

/// 二阶偏导

MyMath::Vector3 Geometry_SurfaceOfRevolution::secondDerivativeUUAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                          "Geometry_SurfaceOfRevolution derivative parameters are outside the natural parameter domain.");

    const MyMath::Vector3 relativePoint = profileCurve().pointAt(v) - m_axisOrigin;

    return rotateVectorSecondDerivative(relativePoint, u);
}

MyMath::Vector3 Geometry_SurfaceOfRevolution::secondDerivativeUVAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                          "Geometry_SurfaceOfRevolution derivative parameters are outside the natural parameter domain.");

    return rotateVectorFirstDerivative(profileCurve().firstDerivativeAt(v), u);
}

MyMath::Vector3 Geometry_SurfaceOfRevolution::secondDerivativeVVAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                          "Geometry_SurfaceOfRevolution derivative parameters are outside the natural parameter domain.");

    return rotateVector(profileCurve().secondDerivativeAt(v), u);
}

/// 内部辅助

MyMath::Vector3 Geometry_SurfaceOfRevolution::rotateVector(const MyMath::Vector3& value, double angle) const
{
    const MyMath::Vector3 parallel =
        m_axisDirection * MyMath::Vector3::dot(m_axisDirection, value);
    const MyMath::Vector3 perpendicular = value - parallel;
    const MyMath::Vector3 tangent =
        MyMath::Vector3::cross(m_axisDirection, perpendicular);

    return parallel +
           perpendicular * std::cos(angle) +
           tangent * std::sin(angle);
}

MyMath::Vector3 Geometry_SurfaceOfRevolution::rotateVectorFirstDerivative(
    const MyMath::Vector3& value,
    double angle) const
{
    const MyMath::Vector3 parallel =
        m_axisDirection * MyMath::Vector3::dot(m_axisDirection, value);
    const MyMath::Vector3 perpendicular = value - parallel;
    const MyMath::Vector3 tangent =
        MyMath::Vector3::cross(m_axisDirection, perpendicular);

    return perpendicular * (-std::sin(angle)) +
           tangent * std::cos(angle);
}

MyMath::Vector3 Geometry_SurfaceOfRevolution::rotateVectorSecondDerivative(
    const MyMath::Vector3& value,
    double angle) const
{
    const MyMath::Vector3 parallel =
        m_axisDirection * MyMath::Vector3::dot(m_axisDirection, value);
    const MyMath::Vector3 perpendicular = value - parallel;
    const MyMath::Vector3 tangent =
        MyMath::Vector3::cross(m_axisDirection, perpendicular);

    return perpendicular * (-std::cos(angle)) +
           tangent * (-std::sin(angle));
}

}