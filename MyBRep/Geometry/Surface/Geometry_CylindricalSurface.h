#ifndef MYBREP_GEOMETRY_SURFACE_GEOMETRY_CYLINDRICALSURFACE_H
#define MYBREP_GEOMETRY_SURFACE_GEOMETRY_CYLINDRICALSURFACE_H

#include "MyMath/CoordinateSystem.h"
#include "Geometry_Surface.h"

namespace MyBRep
{

// 表示三维几何空间中的完整无限圆柱面，U为绕轴弧度角，V为沿单位轴方向的有符号距离。
class Geometry_CylindricalSurface : public Geometry_Surface
{
public:
    // 使用世界Z轴方向、指定轴原点和正半径创建无限圆柱面。
    Geometry_CylindricalSurface(const MyMath::Vector3& axisOrigin, double radius);
    // 使用轴原点、正半径和两个正交径向单位方向创建无限圆柱面。
    Geometry_CylindricalSurface(const MyMath::Vector3& axisOrigin, double radius,
                      const MyMath::Vector3& xDir, const MyMath::Vector3& yDir);
    // 使用指定有效正交坐标系的原点、X/Y轴和正半径创建无限圆柱面。
    Geometry_CylindricalSurface(const MyMath::CoordinateSystem& coordinateSystem, double radius);
    // 通过侵入式引用计数管理圆柱面几何生命周期。
    ~Geometry_CylindricalSurface() override = default;
    /// 圆柱几何数据

    // 返回V参数为0时对应的圆柱轴参考点。
    const MyMath::Vector3& axisOrigin() const;
    // 返回U参数为0时对应的径向单位方向。
    const MyMath::Vector3& xDir() const;
    // 返回U参数为π/2时对应的径向单位方向。
    const MyMath::Vector3& yDir() const;
    // 返回由xDir叉乘yDir确定的圆柱单位轴方向。
    MyMath::Vector3 axisDir() const;
    // 返回圆柱半径。
    double radius() const;

    /// 曲面类型

    // 返回圆柱面类型。
    SurfaceKind kind() const override;

    /// U定义域

    // 返回false，完整圆柱面的U自然参数域为整个实数域。
    bool isUDomainBounded() const override;
    // 返回负无穷。
    double uDomainStart() const override;
    // 返回正无穷。
    double uDomainEnd() const override;

    /// V定义域

    // 返回false，完整无限圆柱面的V自然参数域为整个实数域。
    bool isVDomainBounded() const override;
    // 返回负无穷。
    double vDomainStart() const override;
    // 返回正无穷。
    double vDomainEnd() const override;

    /// 周期性

    // 返回U方向参数周期2π。
    double uPeriod() const override;
    // 返回0，V方向不是周期参数方向。
    double vPeriod() const override;

    /// 参数查询

    // 返回指定绕轴角度和轴向距离对应的圆柱面三维点。
    MyMath::Vector3 pointAt(double u, double v) const override;

    /// 一阶偏导

    // 返回指定参数处相对于绕轴角参数U的一阶偏导。
    MyMath::Vector3 firstDerivativeUAt(double u, double v) const override;
    // 返回恒定的圆柱单位轴方向。
    MyMath::Vector3 firstDerivativeVAt(double u, double v) const override;

    /// 二阶偏导

    // 返回指定参数处相对于U参数的二阶偏导。
    MyMath::Vector3 secondDerivativeUUAt(double u, double v) const override;
    // 返回零向量，圆柱面的UV混合二阶偏导恒为零。
    MyMath::Vector3 secondDerivativeUVAt(double u, double v) const override;
    // 返回零向量，圆柱面关于V的二阶偏导恒为零。
    MyMath::Vector3 secondDerivativeVVAt(double u, double v) const override;

    /// 曲面方向

    // 返回指定参数处从圆柱轴指向曲面的外侧单位法向。
    MyMath::Vector3 normalAt(double u, double v) const override;

protected:


private:
    MyMath::Vector3 m_axisOrigin; // V参数为0时对应的圆柱轴参考点。
    MyMath::Vector3 m_xDir; // U参数为0时对应的径向单位方向。
    MyMath::Vector3 m_yDir; // U参数为π/2时对应的径向单位方向。
    double m_radius; // 圆柱半径。
};

}

#endif // MYBREP_GEOMETRY_SURFACE_GEOMETRY_CYLINDRICALSURFACE_H