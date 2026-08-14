#include "Geometry_Line2D.h"

#include <limits>

namespace MyBRep
{

Geometry_Line2D::Geometry_Line2D(const MyMath::Vector2& origin,
                                 const MyMath::Vector2& direction)
    : m_origin(origin)
    , m_direction(direction.normalized(0.0))
{
    MYBREP_ASSERT_MESSAGE(origin.isFinite(),
                          "Geometry_Line2D origin must be finite.");
    MYBREP_ASSERT_MESSAGE(direction.isVector(0.0),
                          "Geometry_Line2D direction must be a finite non-zero vector.");
}

/// 直线数据

const MyMath::Vector2& Geometry_Line2D::origin() const
{
    return m_origin;
}

const MyMath::Vector2& Geometry_Line2D::direction() const
{
    return m_direction;
}

/// 曲线类型

CurveKind Geometry_Line2D::kind() const
{
    return CurveKind::Line;
}

/// 定义域

bool Geometry_Line2D::isDomainBounded() const
{
    return false;
}

double Geometry_Line2D::domainStart() const
{
    return -(std::numeric_limits<double>::infinity)();
}

double Geometry_Line2D::domainEnd() const
{
    return (std::numeric_limits<double>::infinity)();
}

/// 周期性

double Geometry_Line2D::period() const
{
    return 0.0;
}

/// 参数查询

MyMath::Vector2 Geometry_Line2D::pointAt(double parameter) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(parameter),
                          "Geometry_Line2D parameter is outside the natural parameter domain.");

    return m_origin + m_direction * parameter;
}

MyMath::Vector2 Geometry_Line2D::firstDerivativeAt(double parameter) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(parameter),
                          "Geometry_Line2D derivative parameter is outside the natural parameter domain.");

    return m_direction;
}

MyMath::Vector2 Geometry_Line2D::secondDerivativeAt(double parameter) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(parameter),
                          "Geometry_Line2D derivative parameter is outside the natural parameter domain.");

    return MyMath::Vector2::zero();
}

}
