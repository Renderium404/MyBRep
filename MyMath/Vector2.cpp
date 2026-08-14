#include "Vector2.h"

#include <cassert>
#include <cmath>
#include <limits>

namespace
{

// 判断浮点数是否为有限值。
bool isFiniteValue(double value)
{
    const double infinity = (std::numeric_limits<double>::infinity)();
    return value == value && value != infinity && value != -infinity;
}

// 使用最大分量缩放计算二维长度，避免中间平方溢出。
double scaledLength(double x, double y)
{
    const double absoluteX = std::fabs(x);
    const double absoluteY = std::fabs(y);
    const double scale = absoluteX > absoluteY ? absoluteX : absoluteY;

    if (scale == 0.0)
    {
        return 0.0;
    }

    if (!isFiniteValue(scale))
    {
        return scale;
    }

    const double scaledX = x / scale;
    const double scaledY = y / scale;
    return scale * std::sqrt(scaledX * scaledX + scaledY * scaledY);
}

}

namespace MyMath
{

const double Vector2::DefaultEpsilon = 1.0e-12; // 与Vector3保持一致的默认浮点比较误差。

Vector2::Vector2()
    : m_x(0.0)
    , m_y(0.0)
{
}

Vector2::Vector2(double x, double y)
    : m_x(x)
    , m_y(y)
{
}

/// 数据创建

Vector2 Vector2::zero()
{
    return Vector2();
}

Vector2 Vector2::unitX()
{
    return Vector2(1.0, 0.0);
}

Vector2 Vector2::unitY()
{
    return Vector2(0.0, 1.0);
}

/// 分量访问

double Vector2::x() const
{
    return m_x;
}

double Vector2::y() const
{
    return m_y;
}

void Vector2::setX(double x)
{
    m_x = x;
}

void Vector2::setY(double y)
{
    m_y = y;
}

void Vector2::set(double x, double y)
{
    m_x = x;
    m_y = y;
}

/// 状态判断

bool Vector2::isFinite() const
{
    return isFiniteValue(m_x) && isFiniteValue(m_y);
}

bool Vector2::isVector(double epsilon) const
{
    return isFinite() && length() > epsilon;
}

bool Vector2::isZero(double epsilon) const
{
    return isFinite() && length() <= epsilon;
}

bool Vector2::isUnit(double epsilon) const
{
    return isFinite() && std::fabs(length() - 1.0) <= epsilon;
}

bool Vector2::isEqualTo(const Vector2& other, double epsilon) const
{
    return isFinite() && other.isFinite() && distanceTo(other) <= epsilon;
}

/// 长度与距离

double Vector2::lengthSquared() const
{
    return m_x * m_x + m_y * m_y;
}

double Vector2::length() const
{
    return scaledLength(m_x, m_y);
}

double Vector2::distanceSquaredTo(const Vector2& other) const
{
    const double deltaX = m_x - other.m_x;
    const double deltaY = m_y - other.m_y;
    return deltaX * deltaX + deltaY * deltaY;
}

double Vector2::distanceTo(const Vector2& other) const
{
    return scaledLength(m_x - other.m_x, m_y - other.m_y);
}

/// 向量计算

Vector2 Vector2::normalized(double epsilon) const
{
    if (!isVector(epsilon))
    {
        return Vector2::zero();
    }

    return *this / length();
}

bool Vector2::normalize(double epsilon)
{
    if (!isVector(epsilon))
    {
        return false;
    }

    *this /= length();
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
