#ifndef MYBREP_GEOMETRY_SURFACE_GEOMETRY_SPHERICALSURFACE_H
#define MYBREP_GEOMETRY_SURFACE_GEOMETRY_SPHERICALSURFACE_H

#include "MyMath/CoordinateSystem.h"
#include "Geometry_Surface.h"

namespace MyBRep
{

// 表示三维几何空间中的完整球面，U为经度弧度角，V为[-π/2,π/2]范围内的纬度弧度角。
class Geometry_SphericalSurface : public Geometry_Surface
{
public:
    // 使用世界坐标方向、指定球心和正半径创建完整球面。
    Geometry_SphericalSurface(const MyMath::Vector3& center, double radius);
    // 使用球心、正半径和两个正交赤道单位方向创建完整球面。
    Geometry_SphericalSurface(const MyMath::Vector3& center, double radius,
                    const MyMath::Vector3& xDir, const MyMath::Vector3& yDir);
    // 使用指定有效正交坐标系的原点、X/Y轴和正半径创建完整球面。
    Geometry_SphericalSurface(const MyMath::CoordinateSystem& coordinateSystem, double radius);
    // 通过侵入式引用计数管理球面几何生命周期。
    ~Geometry_SphericalSurface() override = default;
    /// 球面几何数据

    // 返回球心。
    const MyMath::Vector3& center() const;
    // 返回U参数为0时对应的赤道单位方向。
    const MyMath::Vector3& xDir() const;
    // 返回U参数为π/2时对应的赤道单位方向。
    const MyMath::Vector3& yDir() const;
    // 返回由xDir叉乘yDir确定的北极单位方向。
    MyMath::Vector3 zDir() const;
    // 返回球半径。
    double radius() const;

    /// 曲面类型

    // 返回球面类型。
    SurfaceKind kind() const override;

    /// U定义域

    // 返回false，完整球面的经度参数可沿整个实数域周期延拓。
    bool isUDomainBounded() const override;
    // 返回负无穷。
    double uDomainStart() const override;
    // 返回正无穷。
    double uDomainEnd() const override;

    /// V定义域

    // 返回true，纬度自然参数域为[-π/2,π/2]。
    bool isVDomainBounded() const override;
    // 返回南极对应纬度参数-π/2。
    double vDomainStart() const override;
    // 返回北极对应纬度参数π/2。
    double vDomainEnd() const override;

    /// 周期性

    // 返回经度方向参数周期2π。
    double uPeriod() const override;
    // 返回0，纬度方向不是周期参数方向。
    double vPeriod() const override;

    /// 参数查询

    // 返回指定经度和纬度参数对应的球面三维点。
    MyMath::Vector3 pointAt(double u, double v) const override;

    /// 一阶偏导

    // 返回指定参数处相对于经度参数U的一阶偏导，南北极处为零向量。
    MyMath::Vector3 firstDerivativeUAt(double u, double v) const override;
    // 返回指定参数处相对于纬度参数V的一阶偏导。
    MyMath::Vector3 firstDerivativeVAt(double u, double v) const override;

    /// 二阶偏导

    // 返回指定参数处相对于U参数的二阶偏导。
    MyMath::Vector3 secondDerivativeUUAt(double u, double v) const override;
    // 返回指定参数处相对于U和V参数的混合二阶偏导。
    MyMath::Vector3 secondDerivativeUVAt(double u, double v) const override;
    // 返回指定参数处相对于V参数的二阶偏导。
    MyMath::Vector3 secondDerivativeVVAt(double u, double v) const override;

    /// 曲面方向

    // 返回从球心指向指定参数点的外侧单位法向，包含南北极参数奇点。
    MyMath::Vector3 normalAt(double u, double v) const override;

protected:


private:
    MyMath::Vector3 m_center;       // 球心。
    MyMath::Vector3 m_xDir;         // U参数为0时对应的赤道单位方向。
    MyMath::Vector3 m_yDir;         // U参数为π/2时对应的赤道单位方向。
    double m_radius;                // 球半径。
};

}

#endif // MYBREP_GEOMETRY_SURFACE_GEOMETRY_SPHERICALSURFACE_H