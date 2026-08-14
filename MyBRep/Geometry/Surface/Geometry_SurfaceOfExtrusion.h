#ifndef MYBREP_GEOMETRY_SURFACE_GEOMETRY_SURFACEOFEXTRUSION_H
#define MYBREP_GEOMETRY_SURFACE_GEOMETRY_SURFACEOFEXTRUSION_H

#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Curve/Geometry_Curve.h"
#include "Geometry_Surface.h"

namespace MyBRep
{

// 表示由完整参数曲线沿固定单位方向无限拉伸得到的完整参数曲面，U继承母线参数，V表示拉伸距离。
class Geometry_SurfaceOfExtrusion : public Geometry_Surface
{
public:
    // 使用非空母线曲线和单位拉伸方向创建完整无限拉伸曲面。
    Geometry_SurfaceOfExtrusion(const Foundation::RefPtr<const Geometry_Curve>& profileCurve,
                                const MyMath::Vector3& direction);

    /// 拉伸几何数据

    // 返回拉伸曲面引用的不可变母线曲线。
    const Geometry_Curve& profileCurve() const;
    // 返回拉伸曲面引用的不可变母线曲线资源。
    const Foundation::RefPtr<const Geometry_Curve>& profileCurveResource() const;
    // 返回V参数增大方向对应的单位拉伸方向。
    const MyMath::Vector3& direction() const;

    /// 曲面类型

    // 返回拉伸曲面类型。
    SurfaceKind kind() const override;

    /// U定义域

    // 返回母线曲线自然参数域是否有界。
    bool isUDomainBounded() const override;
    // 返回母线曲线自然参数域左边界。
    double uDomainStart() const override;
    // 返回母线曲线自然参数域右边界。
    double uDomainEnd() const override;

    /// V定义域

    // 返回false，完整拉伸曲面的V自然参数域为整个实数域。
    bool isVDomainBounded() const override;
    // 返回负无穷。
    double vDomainStart() const override;
    // 返回正无穷。
    double vDomainEnd() const override;

    /// 周期性

    // 返回母线曲线参数周期，母线非周期时返回0。
    double uPeriod() const override;
    // 返回0，拉伸距离方向不是周期参数方向。
    double vPeriod() const override;

    /// 参数查询

    // 返回profileCurve(u) + direction * v对应的三维曲面点。
    MyMath::Vector3 pointAt(double u, double v) const override;

    /// 一阶偏导

    // 返回母线曲线在U参数处的一阶导数。
    MyMath::Vector3 firstDerivativeUAt(double u, double v) const override;
    // 返回恒定的单位拉伸方向。
    MyMath::Vector3 firstDerivativeVAt(double u, double v) const override;

    /// 二阶偏导

    // 返回母线曲线在U参数处的二阶导数。
    MyMath::Vector3 secondDerivativeUUAt(double u, double v) const override;
    // 返回零向量，拉伸曲面的UV混合二阶偏导恒为零。
    MyMath::Vector3 secondDerivativeUVAt(double u, double v) const override;
    // 返回零向量，拉伸曲面关于V的二阶偏导恒为零。
    MyMath::Vector3 secondDerivativeVVAt(double u, double v) const override;

protected:
    // 通过侵入式引用计数管理拉伸曲面几何生命周期。
    ~Geometry_SurfaceOfExtrusion() override = default;

private:
    Foundation::RefPtr<const Geometry_Curve> m_profileCurve; // 定义U参数方向的不可变完整母线曲线。
    MyMath::Vector3 m_direction; // V参数增大方向对应的单位拉伸方向。
};

}

#endif // MYBREP_GEOMETRY_SURFACE_GEOMETRY_SURFACEOFEXTRUSION_H