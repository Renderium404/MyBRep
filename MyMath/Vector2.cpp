#include "Vector2.h"

#include <cassert>
#include <cmath>

#include "MathUtils.h"

namespace MyMath
{

const double Vector2::DefaultEpsilon = 1.0e-12;

Vector2::Vector2()
    : m_x(0.0), m_y(0.0)
{
}

Vector2::Vector2(double x, double y)
    : m_x(x), m_y(y)
{
}

/// 状态判断

bool Vector2::isFinite() const
{
    return MyMath::isFinite(m_x) && MyMath::isFinite(m_y);
}

bool Vector2::isVector(double epsilon) const
{
    assert(epsilon >= 0.0);
    return isFinite() && length() > epsilon;
}

bool Vector2::isZero(double epsilon) const
{
    assert(epsilon >= 0.0);
    return isFinite() && length() <= epsilon;
}

bool Vector2::isUnit(double epsilon) const
{
    assert(epsilon >= 0.0);
    return isFinite() && std::fabs(length() - 1.0) <= epsilon;
}

bool Vector2::isEqualTo(const Vector2& other, double epsilon) const
{
    assert(epsilon >= 0.0);
    return isFinite() && other.isFinite() && distanceTo(other) <= epsilon;
}

/// 长度与距离

double Vector2::lengthSquared() const
{
    return m_x * m_x + m_y * m_y;
}

double Vector2::length() const
{
    return MyMath::norm(m_x, m_y);
}

double Vector2::distanceSquaredTo(const Vector2& other) const
{
    const double deltaX = m_x - other.m_x;
    const double deltaY = m_y - other.m_y;
    return deltaX * deltaX + deltaY * deltaY;
}

double Vector2::distanceTo(const Vector2& other) const
{
    return MyMath::norm(m_x - other.m_x, m_y - other.m_y);
}

/// 向量计算

Vector2 Vector2::normalized(double epsilon) const
{
    assert(epsilon >= 0.0);

    Vector2 result(*this);
    if (!result.normalize(epsilon)) return Vector2::zero();

    return result;
}

bool Vector2::normalize(double epsilon)
{
    assert(epsilon >= 0.0);

    if (!isFinite()) return false;

    const double scale = maximumAbsolute(m_x, m_y);
    if (scale == 0.0) return false;

    const double normalizedLength = scaledNorm(m_x, m_y, scale);
    if (scale <= epsilon / normalizedLength) return false;

    m_x = m_x / scale / normalizedLength;
    m_y = m_y / scale / normalizedLength;

    return true;
}

double Vector2::dot(const Vector2& first, const Vector2& second)
{
    return first.m_x * second.m_x + first.m_y * second.m_y;
}

double Vector2::cross(const Vector2& first, const Vector2& second)
{
    return first.m_x * second.m_y - first.m_y * second.m_x;
}

/// 算术运算

Vector2 Vector2::operator+(const Vector2& other) const
{
    return Vector2(m_x + other.m_x, m_y + other.m_y);
}

Vector2 Vector2::operator-(const Vector2& other) const
{
    return Vector2(m_x - other.m_x, m_y - other.m_y);
}

Vector2 Vector2::operator-() const
{
    return Vector2(-m_x, -m_y);
}

Vector2 Vector2::operator*(double scalar) const
{
    return Vector2(m_x * scalar, m_y * scalar);
}

Vector2 Vector2::operator/(double scalar) const
{
    assert(scalar != 0.0);
    return Vector2(m_x / scalar, m_y / scalar);
}

Vector2& Vector2::operator+=(const Vector2& other)
{
    m_x += other.m_x;
    m_y += other.m_y;
    return *this;
}

Vector2& Vector2::operator-=(const Vector2& other)
{
    m_x -= other.m_x;
    m_y -= other.m_y;
    return *this;
}

Vector2& Vector2::operator*=(double scalar)
{
    m_x *= scalar;
    m_y *= scalar;
    return *this;
}

Vector2& Vector2::operator/=(double scalar)
{
    assert(scalar != 0.0);

    m_x /= scalar;
    m_y /= scalar;
    return *this;
}

Vector2 operator*(double scalar, const Vector2& vector)
{
    return vector * scalar;
}

}