// Line.cpp
#include "Line.h"

#include <cassert>

namespace MyMath
{

Line::Line(const Vector2& start, const Vector2& end)
    : m_start(start)
    , m_end(end)
    , m_direction()
    , m_normal()
    , m_length(0.0)
{
    assert(start.isFinite() && end.isFinite());

    const Vector2 delta = end - start;
    m_length = delta.length();

    if (m_length > 0.0)
    {
        m_direction = delta / m_length;
        m_normal = Vector2(m_direction.y(), -m_direction.x());
    }
}

Vector2 Line::normalAtL(double l, double epsilon) const
{
    assert(l >= 0.0 && l <= m_length);
    return m_normal;
}

double Line::r(double l) const
{
    assert(l >= 0.0 && l <= m_length);
    return m_start.x() + m_direction.x() * l;
}

double Line::h(double l) const
{
    assert(l >= 0.0 && l <= m_length);
    return m_start.y() + m_direction.y() * l;
}

}