#include "Geometry_Line.h"

#include <limits>

namespace
{

// 校验并规范化直线方向。
MyMath::Vector3 normalizedDirection(const MyMath::Vector3& direction)
{
    MYBREP_ASSERT_MESSAGE(direction.isVector(0.0),
                          "Geometry_Line direction must be a finite non-zero vector.");
    return direction.normalized(0.0);
}

}

namespace MyBRep
{

Geometry_Line::Geometry_Line(const MyMath::Vector3& origin, const MyMath::Vector3& direction)
    : m_origin(origin)
    , m_direction(normalizedDirection(direction))
{
    MYBREP_ASSERT_MESSAGE(origin.isFinite(), "Geometry_Line origin must be finite.");
}

/// 直线数据

const MyMath::Vector3& Geometry_Line::origin() const
{
    return m_origin;
}

const MyMath::Vector3& Geometry_Line::direction() const
{
    return m_direction;
}

/// 曲线类型

CurveKind Geometry_Line::kind() const
{
    return CurveKind::Line;
}

/// 定义域

bool Geometry_Line::isDomainBounded() const
{
    return false;
}

double Geometry_Line::domainStart() const
{
    return -(std::numeric_limits<double>::infinity)();
}

double Geometry_Line::domainEnd() const
{
    return (std::numeric_limits<double>::infinity)();
}

/// 周期性

double Geometry_Line::period() const
{
    return 0.0;
}

/// 参数查询

MyMath::Vector3 Geometry_Line::pointAt(double parameter) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(parameter),
                          "Geometry_Line parameter is outside the natural parameter domain.");
    return m_origin + m_direction * parameter;
}

MyMath::Vector3 Geometry_Line::firstDerivativeAt(double parameter) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(parameter),
                          "Geometry_Line derivative parameter is outside the natural parameter domain.");
    return m_direction;
}

MyMath::Vector3 Geometry_Line::secondDerivativeAt(double parameter) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(parameter),
                          "Geometry_Line derivative parameter is outside the natural parameter domain.");
    return MyMath::Vector3(0.0, 0.0, 0.0);
}

}