#ifndef MYBREP_GEOMETRY_CURVE2D_GEOMETRY_CIRCLE2D_H
#define MYBREP_GEOMETRY_CURVE2D_GEOMETRY_CIRCLE2D_H

#include "Geometry_Curve2D.h"

namespace MyBRep
{

// 表示二维参数空间中的完整周期圆，参数为从xDir向yDir旋转的弧度角。
class Geometry_Circle2D : public Geometry_Curve2D
{
public:
    // 使用标准+X/+Y参数方向、指定圆心和正半径创建完整二维圆。
    Geometry_Circle2D(const MyMath::Vector2& center, double radius);
    // 使用圆心、正半径和两个正交单位参数方向创建完整二维圆。
    Geometry_Circle2D(const MyMath::Vector2& center,
                      double radius,
                      const MyMath::Vector2& xDir,
                      const MyMath::Vector2& yDir);

    /// 圆几何数据

    // 返回二维圆心。
    const MyMath::Vector2& center() const;
    // 返回参数0对应的二维单位方向。
    const MyMath::Vector2& xDir() const;
    // 返回参数π/2对应的二维单位方向。
    const MyMath::Vector2& yDir() const;
    // 返回参数方向的二维有向基行列式，+1表示逆时针参数方向，-1表示顺时针参数方向。
    double orientationSign() const;
    // 返回圆半径。
    double radius() const;

    /// 曲线类型

    // 返回圆类型。
    CurveKind kind() const override;

    /// 定义域

    // 返回false，完整周期二维圆的自然参数域为整个实数域。
    bool isDomainBounded() const override;
    // 返回负无穷。
    double domainStart() const override;
    // 返回正无穷。
    double domainEnd() const override;

    /// 周期性

    // 返回完整二维圆的参数周期2π。
    double period() const override;

    /// 参数查询

    // 返回指定弧度参数对应的二维圆上点。
    MyMath::Vector2 pointAt(double parameter) const override;
    // 返回指定参数处相对于角参数的一阶导数。
    MyMath::Vector2 firstDerivativeAt(double parameter) const override;
    // 返回指定参数处相对于角参数的二阶导数。
    MyMath::Vector2 secondDerivativeAt(double parameter) const override;

protected:
    // 通过侵入式引用计数管理二维圆几何生命周期。
    ~Geometry_Circle2D() override = default;

private:
    MyMath::Vector2 m_center; // 二维圆心。
    MyMath::Vector2 m_xDir; // 参数0对应的二维单位方向。
    MyMath::Vector2 m_yDir; // 参数π/2对应的二维单位方向。
    double m_radius; // 圆半径。
};

}

#endif // MYBREP_GEOMETRY_CURVE2D_GEOMETRY_CIRCLE2D_H
