#ifndef MYBREP_GEOMETRY_SURFACE_GEOMETRY_CONICALSURFACE_H
#define MYBREP_GEOMETRY_SURFACE_GEOMETRY_CONICALSURFACE_H

#include "MyMath/CoordinateSystem.h"
#include "Geometry_Surface.h"

namespace MyBRep
{

// 表示从顶点沿单位轴正方向无限延伸的完整单侧圆锥面，U为绕轴角度，V为沿轴方向的非负距离。
class Geometry_ConicalSurface : public Geometry_Surface
{
public:
    // 使用世界Z轴方向、指定顶点和半顶角创建完整单侧圆锥面。
    Geometry_ConicalSurface(const MyMath::Vector3& apex, double semiAngle);
    // 使用顶点、半顶角和两个正交径向单位方向创建完整单侧圆锥面。
    Geometry_ConicalSurface(const MyMath::Vector3& apex,
                            double semiAngle,
                            const MyMath::Vector3& xDir,
                            const MyMath::Vector3& yDir);
    // 使用指定有效正交坐标系的原点、X/Y轴和半顶角创建完整单侧圆锥面。
    Geometry_ConicalSurface(const MyMath::CoordinateSystem& coordinateSystem, double semiAngle);

    /// 圆锥几何数据

    // 返回圆锥顶点。
    const MyMath::Vector3& apex() const;
    // 返回U参数为0时对应的径向单位方向。
    const MyMath::Vector3& xDir() const;
    // 返回U参数为π/2时对应的径向单位方向。
    const MyMath::Vector3& yDir() const;
    // 返回由xDir叉乘yDir确定的圆锥单位轴方向。
    MyMath::Vector3 axisDir() const;
    // 返回圆锥半顶角，范围为(0,π/2)。
    double semiAngle() const;
    // 返回单位轴向距离对应的径向增长量tan(semiAngle)。
    double radialSlope() const;

    /// 曲面类型

    // 返回圆锥面类型。
    SurfaceKind kind() const override;

    /// U定义域

    // 返回false，完整圆锥面的U自然参数域可沿整个实数域周期延拓。
    bool isUDomainBounded() const override;
    // 返回负无穷。
    double uDomainStart() const override;
    // 返回正无穷。
    double uDomainEnd() const override;

    /// V定义域

    // 返回false，圆锥面的V自然参数域为[0,+∞)，不是双侧有限区间。
    bool isVDomainBounded() const override;
    // 返回顶点对应轴向参数0。
    double vDomainStart() const override;
    // 返回正无穷。
    double vDomainEnd() const override;

    /// 周期性

    // 返回绕轴方向参数周期2π。
    double uPeriod() const override;
    // 返回0，轴向参数不是周期参数。
    double vPeriod() const override;

    /// 参数查询

    // 返回指定绕轴角度和非负轴向距离对应的圆锥面三维点。
    MyMath::Vector3 pointAt(double u, double v) const override;

    /// 一阶偏导

    // 返回指定参数处相对于绕轴角参数U的一阶偏导，顶点处为零向量。
    MyMath::Vector3 firstDerivativeUAt(double u, double v) const override;
    // 返回指定参数处相对于轴向距离参数V的一阶偏导。
    MyMath::Vector3 firstDerivativeVAt(double u, double v) const override;

    /// 二阶偏导

    // 返回指定参数处相对于U参数的二阶偏导。
    MyMath::Vector3 secondDerivativeUUAt(double u, double v) const override;
    // 返回指定参数处相对于U和V参数的混合二阶偏导。
    MyMath::Vector3 secondDerivativeUVAt(double u, double v) const override;
    // 返回零向量，圆锥面关于V参数的二阶偏导恒为零。
    MyMath::Vector3 secondDerivativeVVAt(double u, double v) const override;

    /// 曲面方向

    // 返回指定非顶点参数处指向圆锥外侧的单位法向，V必须大于0。
    MyMath::Vector3 normalAt(double u, double v) const override;

protected:
    // 通过侵入式引用计数管理圆锥面几何生命周期。
    ~Geometry_ConicalSurface() override = default;

private:
    MyMath::Vector3 m_apex; // 圆锥顶点。
    MyMath::Vector3 m_xDir; // U参数为0时对应的径向单位方向。
    MyMath::Vector3 m_yDir; // U参数为π/2时对应的径向单位方向。
    double m_semiAngle; // 圆锥半顶角。
    double m_radialSlope; // 单位轴向距离对应的径向增长量tan(semiAngle)。
};

}

#endif // MYBREP_GEOMETRY_SURFACE_GEOMETRY_CONICALSURFACE_H