#ifndef MYBREP_GEOMETRY_SURFACE_GEOMETRY_PLANESURFACE_H
#define MYBREP_GEOMETRY_SURFACE_GEOMETRY_PLANESURFACE_H

#include "MyMath/CoordinateSystem.h"
#include "Geometry_Surface.h"

namespace MyBRep
{

// 表示三维几何空间中的完整无限平面，U/V参数分别表示沿两个正交单位方向的有符号距离。
class Geometry_PlaneSurface : public Geometry_Surface
{
public:
    // 在世界XY方向上使用指定原点创建无限平面。
    explicit Geometry_PlaneSurface(const MyMath::Vector3& origin);
    // 使用原点和两个正交单位方向创建无限平面。
    Geometry_PlaneSurface(const MyMath::Vector3& origin, const MyMath::Vector3& uDir, const MyMath::Vector3& vDir);
    // 使用指定有效正交坐标系的原点、X轴和Y轴创建无限平面。
    explicit Geometry_PlaneSurface(const MyMath::CoordinateSystem& coordinateSystem);

    /// 平面几何数据

    // 返回参数(u,v)=(0,0)对应的平面原点。
    const MyMath::Vector3& origin() const;
    // 返回U参数增大方向对应的单位方向。
    const MyMath::Vector3& uDir() const;
    // 返回V参数增大方向对应的单位方向。
    const MyMath::Vector3& vDir() const;
    // 返回由uDir叉乘vDir确定的单位法向。
    MyMath::Vector3 normal() const;

    /// 曲面类型

    // 返回平面类型。
    SurfaceKind kind() const override;

    /// U定义域

    // 返回false，平面的U自然参数域为整个实数域。
    bool isUDomainBounded() const override;
    // 返回负无穷。
    double uDomainStart() const override;
    // 返回正无穷。
    double uDomainEnd() const override;

    /// V定义域

    // 返回false，平面的V自然参数域为整个实数域。
    bool isVDomainBounded() const override;
    // 返回负无穷。
    double vDomainStart() const override;
    // 返回正无穷。
    double vDomainEnd() const override;

    /// 周期性

    // 返回0，平面的U方向不是周期参数方向。
    double uPeriod() const override;
    // 返回0，平面的V方向不是周期参数方向。
    double vPeriod() const override;

    /// 参数查询

    // 返回origin + uDir * u + vDir * v对应的三维点。
    MyMath::Vector3 pointAt(double u, double v) const override;

    /// 一阶偏导

    // 返回恒定的U单位方向。
    MyMath::Vector3 firstDerivativeUAt(double u, double v) const override;
    // 返回恒定的V单位方向。
    MyMath::Vector3 firstDerivativeVAt(double u, double v) const override;

    /// 二阶偏导

    // 返回零向量，平面关于U的二阶偏导恒为零。
    MyMath::Vector3 secondDerivativeUUAt(double u, double v) const override;
    // 返回零向量，平面的UV混合二阶偏导恒为零。
    MyMath::Vector3 secondDerivativeUVAt(double u, double v) const override;
    // 返回零向量，平面关于V的二阶偏导恒为零。
    MyMath::Vector3 secondDerivativeVVAt(double u, double v) const override;

    /// 曲面方向

    // 返回由uDir叉乘vDir确定的恒定单位法向。
    MyMath::Vector3 normalAt(double u, double v) const override;

protected:
    // 通过侵入式引用计数管理平面几何生命周期。
    ~Geometry_PlaneSurface() override = default;

private:
    MyMath::Vector3 m_origin; // 参数(u,v)=(0,0)对应的平面原点。
    MyMath::Vector3 m_uDir; // U参数增大方向对应的单位方向。
    MyMath::Vector3 m_vDir; // V参数增大方向对应的单位方向。
};

}

#endif // MYBREP_GEOMETRY_SURFACE_GEOMETRY_PLANESURFACE_H