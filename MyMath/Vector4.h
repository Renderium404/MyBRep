#ifndef MYMATH_VECTOR4_H
#define MYMATH_VECTOR4_H

namespace MyMath
{

class Matrix3;
class Matrix4;
class Quaternion;
class CoordinateSystem;

// 表示四维双精度向量数据。
class Vector4
{
    friend class Matrix3;
    friend class Matrix4;
    friend class Quaternion;
    friend class CoordinateSystem;

public:
    static const double DefaultEpsilon;

    Vector4();
    Vector4(double x, double y, double z, double w);

    static Vector4 zero(){return Vector4();}
    static Vector4 unitX(){return Vector4(1.0, 0.0, 0.0, 0.0);}
    static Vector4 unitY(){return Vector4(0.0, 1.0, 0.0, 0.0);}
    static Vector4 unitZ(){return Vector4(0.0, 0.0, 1.0, 0.0);}
    static Vector4 unitW(){return Vector4(0.0, 0.0, 0.0, 1.0);}

    double x() const{return m_x;}
    double y() const{return m_y;}
    double z() const{return m_z;}
    double w() const{return m_w;}

    void setX(double x){m_x = x;}
    void setY(double y){m_y = y;}
    void setZ(double z){m_z = z;}
    void setW(double w){m_w = w;}
    void set(double x, double y, double z, double w){m_x = x; m_y = y; m_z = z; m_w = w;}

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
    bool isEqualTo(const Vector4& other, double epsilon = DefaultEpsilon) const;
    bool operator==(const Vector4& other) const{return isEqualTo(other);}
    bool operator!=(const Vector4& other) const{return !isEqualTo(other);}

    /// 长度与距离

    // 返回向量长度平方，结果可能因数值范围而溢出。
    double lengthSquared() const;
    // 返回使用缩放计算的向量长度。
    double length() const;

    // 返回当前数据与目标数据之间的距离平方，结果可能因数值范围而溢出。
    double distanceSquaredTo(const Vector4& other) const;
    // 返回当前数据与目标数据之间使用缩放计算的距离。
    double distanceTo(const Vector4& other) const;

    /// 向量计算

    // 返回单位向量，当前数据不能作为向量时返回零向量。
    Vector4 normalized(double epsilon = DefaultEpsilon) const;
    // 将当前数据归一化，当前数据不能作为向量时保持不变并返回false。
    bool normalize(double epsilon = DefaultEpsilon);

    // 计算两个四维向量的点积。
    static double dot(const Vector4& first, const Vector4& second);

    /// 算术运算

    Vector4 operator+(const Vector4& other) const;
    Vector4 operator-(const Vector4& other) const;
    Vector4 operator-() const;
    Vector4 operator*(double scalar) const;
    Vector4 operator/(double scalar) const;

    Vector4& operator+=(const Vector4& other);
    Vector4& operator-=(const Vector4& other);
    Vector4& operator*=(double scalar);
    Vector4& operator/=(double scalar);

private:
    double m_x;
    double m_y;
    double m_z;
    double m_w;
};

// 计算标量与四维向量的乘积。
Vector4 operator*(double scalar, const Vector4& vector);

}

#endif // MYMATH_VECTOR4_H