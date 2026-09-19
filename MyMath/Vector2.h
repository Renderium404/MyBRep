#ifndef MYMATH_VECTOR2_H
#define MYMATH_VECTOR2_H

namespace MyMath
{

// 表示二维点或二维向量的双精度数据。
class Vector2
{
public:
    static const double DefaultEpsilon;

    Vector2();
    Vector2(double x, double y);

    static Vector2 zero(){return Vector2();}
    static Vector2 unitX(){return Vector2(1.0, 0.0);}
    static Vector2 unitY(){return Vector2(0.0, 1.0);}

    double x() const{return m_x;}
    double y() const{return m_y;}

    void setX(double x){m_x = x;}
    void setY(double y){m_y = y;}
    void set(double x, double y){m_x = x; m_y = y;}

    /// 状态判断

    // 判断所有分量是否为有限值。
    bool isFinite() const;
    // 判断当前数据是否能够作为非零向量参与向量运算。
    bool isVector(double epsilon = DefaultEpsilon) const;
    // 判断当前数据是否近似为零。
    bool isZero(double epsilon = DefaultEpsilon) const;
    // 判断当前数据是否近似为单位向量。
    bool isUnit(double epsilon = DefaultEpsilon) const;

    /// 比较运算
    bool isEqualTo(const Vector2& other, double epsilon = DefaultEpsilon) const;
    bool operator==(const Vector2& other) const{return isEqualTo(other);}
    bool operator!=(const Vector2& other) const{return !isEqualTo(other);}

    /// 长度与距离

    // 返回向量长度平方，结果可能因数值范围而溢出。
    double lengthSquared() const;
    // 返回使用缩放计算的向量长度。
    double length() const;

    // 返回当前数据与目标数据之间的距离平方，结果可能因数值范围而溢出。
    double distanceSquaredTo(const Vector2& other) const;
    // 返回当前数据与目标数据之间使用缩放计算的距离。
    double distanceTo(const Vector2& other) const;

    /// 向量计算

    // 返回单位向量，当前数据不能作为向量时返回零向量。
    Vector2 normalized(double epsilon = DefaultEpsilon) const;
    // 将当前数据归一化，当前数据不能作为向量时保持不变并返回false。
    bool normalize(double epsilon = DefaultEpsilon);

    // 计算两个二维向量的点积。
    static double dot(const Vector2& first, const Vector2& second);
    // 计算两个二维向量的有向叉积标量。
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
    double m_x;
    double m_y;
};

// 计算标量与二维向量的乘积。
Vector2 operator*(double scalar, const Vector2& vector);

}

#endif // MYMATH_VECTOR2_H