#ifndef MYMATH_VECTOR3_H
#define MYMATH_VECTOR3_H

namespace MyMath
{

class Matrix3;
class Matrix4;
class Quaternion;
class CoordinateSystem;

// 表示三维点或三维向量的双精度数据。
class Vector3
{
    friend class Matrix3;
    friend class Matrix4;
    friend class Quaternion;
    friend class CoordinateSystem;

public:
    static const double DefaultEpsilon;

    Vector3();
    Vector3(double x, double y, double z);

    static Vector3 zero(){return Vector3();}
    static Vector3 unitX(){return Vector3(1.0, 0.0, 0.0);}
    static Vector3 unitY(){return Vector3(0.0, 1.0, 0.0);}
    static Vector3 unitZ(){return Vector3(0.0, 0.0, 1.0);}

    double x() const{return m_x;}
    double y() const{return m_y;}
    double z() const{return m_z;}

    void setX(double x){m_x = x;}
    void setY(double y){m_y = y;}
    void setZ(double z){m_z = z;}
    void set(double x, double y, double z){m_x = x; m_y = y; m_z = z;}

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

    bool isEqualTo(const Vector3& other, double epsilon = DefaultEpsilon) const;
    bool operator==(const Vector3& other) const{return isEqualTo(other);}
    bool operator!=(const Vector3& other) const{return !isEqualTo(other);}

    /// 长度与距离

    // 返回向量长度平方，结果可能因数值范围而溢出。
    double lengthSquared() const;
    // 返回使用缩放计算的向量长度。
    double length() const;

    // 返回当前数据与目标数据之间的距离平方，结果可能因数值范围而溢出。
    double distanceSquaredTo(const Vector3& other) const;
    // 返回当前数据与目标数据之间使用缩放计算的距离。
    double distanceTo(const Vector3& other) const;

    /// 向量计算

    // 返回单位向量，当前数据不能作为向量时返回零向量。
    Vector3 normalized(double epsilon = DefaultEpsilon) const;
    // 将当前数据归一化，当前数据不能作为向量时保持不变并返回false。
    bool normalize(double epsilon = DefaultEpsilon);

    // 计算两个三维向量的点积。
    static double dot(const Vector3& first, const Vector3& second);
    // 计算两个三维向量的叉积。
    static Vector3 cross(const Vector3& first, const Vector3& second);

    /// 算术运算

    Vector3 operator+(const Vector3& other) const;
    Vector3 operator-(const Vector3& other) const;
    Vector3 operator-() const;
    Vector3 operator*(double scalar) const;
    Vector3 operator/(double scalar) const;

    Vector3& operator+=(const Vector3& other);
    Vector3& operator-=(const Vector3& other);
    Vector3& operator*=(double scalar);
    Vector3& operator/=(double scalar);

private:
    double m_x;
    double m_y;
    double m_z;
};

// 计算标量与三维向量的乘积。
Vector3 operator*(double scalar, const Vector3& vector);

}

#endif // MYMATH_VECTOR3_H