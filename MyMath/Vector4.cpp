#include "Vector4.h"

#include <cassert>
#include <cmath>

#include "MathUtils.h"

namespace MyMath
{

const double Vector4::DefaultEpsilon = 1.0e-12;

Vector4::Vector4()
    : m_x(0.0), m_y(0.0), m_z(0.0), m_w(0.0)
{
}

Vector4::Vector4(double x, double y, double z, double w)
    : m_x(x), m_y(y), m_z(z), m_w(w)
{
}

/// 状态判断

bool Vector4::isFinite() const
{
    return MyMath::isFinite(m_x) && MyMath::isFinite(m_y) && MyMath::isFinite(m_z) && MyMath::isFinite(m_w);
}

bool Vector4::isVector(double epsilon) const
{
    assert(epsilon >= 0.0);
    return isFinite() && length() > epsilon;
}

bool Vector4::isZero(double epsilon) const
{
    assert(epsilon >= 0.0);
    return isFinite() && length() <= epsilon;
}

bool Vector4::isUnit(double epsilon) const
{
    assert(epsilon >= 0.0);
    return isFinite() && std::fabs(length() - 1.0) <= epsilon;
}

bool Vector4::isEqualTo(const Vector4& other, double epsilon) const
{
    assert(epsilon >= 0.0);
    return isFinite() && other.isFinite() && distanceTo(other) <= epsilon;
}

/// 长度与距离

double Vector4::lengthSquared() const
{
    return m_x * m_x + m_y * m_y + m_z * m_z + m_w * m_w;
}

double Vector4::length() const
{
    return MyMath::norm(m_x, m_y, m_z, m_w);
}

double Vector4::distanceSquaredTo(const Vector4& other) const
{
    const double deltaX = m_x - other.m_x;
    const double deltaY = m_y - other.m_y;
    const double deltaZ = m_z - other.m_z;
    const double deltaW = m_w - other.m_w;
    return deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ + deltaW * deltaW;
}

double Vector4::distanceTo(const Vector4& other) const
{
    return MyMath::norm(m_x - other.m_x, m_y - other.m_y, m_z - other.m_z, m_w - other.m_w);
}

/// 向量计算

Vector4 Vector4::normalized(double epsilon) const
{
    assert(epsilon >= 0.0);

    Vector4 result(*this);
    if (!result.normalize(epsilon)) return Vector4::zero();

    return result;
}

bool Vector4::normalize(double epsilon)
{
    assert(epsilon >= 0.0);

    if (!isFinite()) return false;

    const double scale = maximumAbsolute(m_x, m_y, m_z, m_w);
    if (scale == 0.0) return false;

    const double normalizedLength = scaledNorm(m_x, m_y, m_z, m_w, scale);
    if (scale <= epsilon / normalizedLength) return false;

    m_x = m_x / scale / normalizedLength;
    m_y = m_y / scale / normalizedLength;
    m_z = m_z / scale / normalizedLength;
    m_w = m_w / scale / normalizedLength;

    return true;
}

double Vector4::dot(const Vector4& first, const Vector4& second)
{
    return first.m_x * second.m_x + first.m_y * second.m_y + first.m_z * second.m_z + first.m_w * second.m_w;
}

/// 算术运算

Vector4 Vector4::operator+(const Vector4& other) const
{
    return Vector4(m_x + other.m_x, m_y + other.m_y, m_z + other.m_z, m_w + other.m_w);
}

Vector4 Vector4::operator-(const Vector4& other) const
{
    return Vector4(m_x - other.m_x, m_y - other.m_y, m_z - other.m_z, m_w - other.m_w);
}

Vector4 Vector4::operator-() const
{
    return Vector4(-m_x, -m_y, -m_z, -m_w);
}

Vector4 Vector4::operator*(double scalar) const
{
    return Vector4(m_x * scalar, m_y * scalar, m_z * scalar, m_w * scalar);
}

Vector4 Vector4::operator/(double scalar) const
{
    assert(scalar != 0.0);
    return Vector4(m_x / scalar, m_y / scalar, m_z / scalar, m_w / scalar);
}

Vector4& Vector4::operator+=(const Vector4& other)
{
    m_x += other.m_x;
    m_y += other.m_y;
    m_z += other.m_z;
    m_w += other.m_w;
    return *this;
}

Vector4& Vector4::operator-=(const Vector4& other)
{
    m_x -= other.m_x;
    m_y -= other.m_y;
    m_z -= other.m_z;
    m_w -= other.m_w;
    return *this;
}

Vector4& Vector4::operator*=(double scalar)
{
    m_x *= scalar;
    m_y *= scalar;
    m_z *= scalar;
    m_w *= scalar;
    return *this;
}

Vector4& Vector4::operator/=(double scalar)
{
    assert(scalar != 0.0);

    m_x /= scalar;
    m_y /= scalar;
    m_z /= scalar;
    m_w /= scalar;

    return *this;
}

Vector4 operator*(double scalar, const Vector4& vector)
{
    return vector * scalar;
}

}