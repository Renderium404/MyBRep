// Arc.cpp
#include "Arc.h"

#include <cassert>
#include <cmath>

#include "MathUtils.h"

namespace MyMath
{

Arc::Arc(const Vector2& center, double radius,
         double startAngle, double endAngle)
    : m_center(center)
    , m_radius(radius)
    , m_startAngle(startAngle)
    , m_endAngle(endAngle)
    , m_length(0.0)
    , m_sweepSign(1.0)
{
    assert(center.isFinite());
    assert(radius > 0.0);
    assert(isFinite(startAngle) && isFinite(endAngle));

    const double sweep = endAngle - startAngle;
    m_sweepSign = (sweep >= 0.0) ? 1.0 : -1.0;
    m_length = std::fabs(sweep) * radius;
}

double Arc::angleAtL(double l) const
{
    assert(l >= 0.0 && l <= m_length);
    return m_startAngle + m_sweepSign * (l / m_radius);
}

Vector2 Arc::normalAtL(double l, double epsilon) const
{
    const double angle = angleAtL(l);
    const double sign = m_sweepSign;
    // 切向量 = sign * (-sinθ, cosθ)
    // 法向量 = 切向量顺时针旋转 90° = sign * (cosθ, sinθ)
    return Vector2(sign * std::cos(angle), sign * std::sin(angle));
}

double Arc::r(double l) const
{
    const double angle = angleAtL(l);
    return m_center.x() + m_radius * std::cos(angle);
}

double Arc::h(double l) const
{
    const double angle = angleAtL(l);
    return m_center.y() + m_radius * std::sin(angle);
}

}