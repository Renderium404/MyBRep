#ifndef MYMATH_VECTOR2_H
#define MYMATH_VECTOR2_H

namespace MyMath
{

// 表示二维双精度向量，同时用于二维点和自由向量数据。
class Vector2
{
public:
    static const double DefaultEpsilon; // 默认浮点比较误差。

public:
    // 构造零向量。
    Vector2();
    // 使用两个分量构造二维数据。
    Vector2(double x, double y);

    /// 数据创建

    // 返回零向量。
    static Vector2 zero();
    // 返回X方向单位向量。
    static Vector2 unitX();
    // 返回Y方向单位向量。
    static Vector2 unitY();

    /// 分量访问

    // 返回X分量。
    double x() const;
    // 返回Y分量。
    double y() const;
    // 设置X分量。
    void setX(double x);
    // 设置Y分量。
    void setY(double y);
    // 同时设置两个分量。
    void set(double x, double y);

    /// 状态判断

    // 判断两个分量是否均为有限值。
    bool isFinite() const;
    // 判断当前数据是否为有限非零向量。
    bool isVector(double epsilon = DefaultEpsilon) const;
    // 判断当前数据是否为有限零向量。
    bool isZero(double epsilon = DefaultEpsilon) const;
    // 判断当前数据是否为有限单位向量。
    bool isUnit(double epsilon = DefaultEpsilon) const;
    // 判断当前数据与指定二维数据是否在给定误差内相等。
    bool isEqualTo(const Vector2& other, double epsilon = DefaultEpsilon) const;

    /// 长度与距离

    // 返回向量长度平方。
    double lengthSquared() const;
    // 返回向量长度。
    double length() const;
    // 返回当前数据到指定数据的距离平方。
    double distanceSquaredTo(const Vector2& other) const;
    // 返回当前数据到指定数据的距离。
    double distanceTo(const Vector2& other) const;

    /// 向量计算

    // 返回单位向量，当前数据不能作为向量时返回零向量。
    Vector2 normalized(double epsilon = DefaultEpsilon) const;
    // 将当前数据归一化，当前数据不能作为向量时保持不变并返回false。
    bool normalize(double epsilon = DefaultEpsilon);
    // 返回两个二维向量的点积。
    static double dot(const Vector2& first, const Vector2& second);
    // 返回两个二维向量的有向叉积标量first.x*second.y-first.y*second.x。
    static double cross(const Vector2& first, const Vector2& second);

    /// 算术运算

    Vector2 operator+(const Vector2& other) const;
    Vector2 operator-(const Vector2& other) const;
    Vector2 operator-() const;
    Vector2 operator*(double scalar) const;
    Vector2 operator/(double scalar) const;

    Vector2& operator+=(const Vector2& other);
    Vector2& operator-=(const Vector2& other);
    Vector2& operator*=(double scalar);
    Vector2& operator/=(double scalar);

private:
    double m_x; // X分量。
    double m_y; // Y分量。
};

// 返回标量与二维向量的乘积。
Vector2 operator*(double scalar, const Vector2& vector);

}

#endif // MYMATH_VECTOR2_H
