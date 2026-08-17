#ifndef MYBREP_GEOMETRY_SURFACE_GEOMETRY_SURFACEOFREVOLUTION_H
#define MYBREP_GEOMETRY_SURFACE_GEOMETRY_SURFACEOFREVOLUTION_H

#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Curve/Geometry_Curve.h"
#include "Geometry_Surface.h"

namespace MyBRep
{

// 表示由完整参数曲线绕固定单位轴无限旋转得到的完整参数曲面，U为旋转角，V继承母线参数。
class Geometry_SurfaceOfRevolution : public Geometry_Surface
{
public:
    // 使用非空母线曲线、有限轴参考点和单位轴方向创建完整旋转曲面。
    Geometry_SurfaceOfRevolution(const Foundation::RefPtr<const Geometry_Curve>& profileCurve,
                                 const MyMath::Vector3& axisOrigin,
                                 const MyMath::Vector3& axisDirection);
    // 通过侵入式引用计数管理旋转曲面几何生命周期。
    ~Geometry_SurfaceOfRevolution() override = default;
    /// 旋转几何数据

    // 返回旋转曲面引用的不可变母线曲线。
    const Geometry_Curve& profileCurve() const;
    // 返回旋转曲面引用的不可变母线曲线资源。
    const Foundation::RefPtr<const Geometry_Curve>& profileCurveResource() const;
    // 返回旋转轴参考点。
    const MyMath::Vector3& axisOrigin() const;
    // 返回按右手定则确定U参数正方向的单位旋转轴方向。
    const MyMath::Vector3& axisDirection() const;

    /// 曲面类型

    // 返回旋转曲面类型。
    SurfaceKind kind() const override;

    /// U定义域

    // 返回false，完整旋转曲面的U自然参数域可沿整个实数域周期延拓。
    bool isUDomainBounded() const override;
    // 返回负无穷。
    double uDomainStart() const override;
    // 返回正无穷。
    double uDomainEnd() const override;

    /// V定义域

    // 返回母线曲线自然参数域是否有界。
    bool isVDomainBounded() const override;
    // 返回母线曲线自然参数域左边界。
    double vDomainStart() const override;
    // 返回母线曲线自然参数域右边界。
    double vDomainEnd() const override;

    /// 周期性

    // 返回旋转角方向参数周期2π。
    double uPeriod() const override;
    // 返回母线曲线参数周期，母线非周期时返回0。
    double vPeriod() const override;

    /// 参数查询

    // 返回母线参数V对应点绕旋转轴按右手定则旋转U弧度后的三维曲面点。
    MyMath::Vector3 pointAt(double u, double v) const override;

    /// 一阶偏导

    // 返回指定参数处相对于旋转角参数U的一阶偏导。
    MyMath::Vector3 firstDerivativeUAt(double u, double v) const override;
    // 返回母线一阶导数绕旋转轴旋转U弧度后的V方向一阶偏导。
    MyMath::Vector3 firstDerivativeVAt(double u, double v) const override;

    /// 二阶偏导

    // 返回指定参数处相对于旋转角参数U的二阶偏导。
    MyMath::Vector3 secondDerivativeUUAt(double u, double v) const override;
    // 返回母线一阶导数旋转后相对于U参数的导数。
    MyMath::Vector3 secondDerivativeUVAt(double u, double v) const override;
    // 返回母线二阶导数绕旋转轴旋转U弧度后的V方向二阶偏导。
    MyMath::Vector3 secondDerivativeVVAt(double u, double v) const override;

protected:


private:
    // 使用Rodrigues公式将自由向量绕单位旋转轴旋转指定弧度。
    MyMath::Vector3 rotateVector(const MyMath::Vector3& value, double angle) const;
    // 返回自由向量绕轴旋转结果相对于旋转角的一阶导数。
    MyMath::Vector3 rotateVectorFirstDerivative(const MyMath::Vector3& value, double angle) const;
    // 返回自由向量绕轴旋转结果相对于旋转角的二阶导数。
    MyMath::Vector3 rotateVectorSecondDerivative(const MyMath::Vector3& value, double angle) const;

private:
    Foundation::RefPtr<const Geometry_Curve> m_profileCurve; // 定义V参数方向的不可变完整母线曲线。
    MyMath::Vector3 m_axisOrigin; // 固定旋转轴上的参考点。
    MyMath::Vector3 m_axisDirection; // 按右手定则确定U参数正方向的单位旋转轴方向。
};

}

#endif // MYBREP_GEOMETRY_SURFACE_GEOMETRY_SURFACEOFREVOLUTION_H