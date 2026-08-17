#ifndef MYBREP_GEOMETRY_CURVE_GEOMETRY_CIRCLE_H
#define MYBREP_GEOMETRY_CURVE_GEOMETRY_CIRCLE_H

#include "MyMath/CoordinateSystem.h"
#include "Geometry_Curve.h"

namespace MyBRep
{

// 表示三维几何空间中的完整周期圆，参数为圆平面内从xDir向yDir旋转的弧度角。
class Geometry_Circle : public Geometry_Curve
{
public:
    // 在世界XY平面中使用圆心和正半径创建完整圆。
    Geometry_Circle(const MyMath::Vector3& center, double radius);
    // 使用圆心、正半径和圆平面正交单位方向创建完整圆。
    Geometry_Circle(const MyMath::Vector3& center,
                    double radius,
                    const MyMath::Vector3& xDir,
                    const MyMath::Vector3& yDir);
    // 使用指定有效正交坐标系的原点、X轴和Y轴创建完整圆。
    Geometry_Circle(const MyMath::CoordinateSystem& coordinateSystem, double radius);
    // 通过侵入式引用计数管理圆几何生命周期。
    ~Geometry_Circle() override = default;
    /// 圆几何数据

    // 返回圆心。
    const MyMath::Vector3& center() const;
    // 返回参数0对应的圆平面单位方向。
    const MyMath::Vector3& xDir() const;
    // 返回参数π/2对应的圆平面单位方向。
    const MyMath::Vector3& yDir() const;
    // 返回由xDir叉乘yDir确定的圆平面单位法向。
    MyMath::Vector3 normal() const;
    // 返回圆半径。
    double radius() const;

    /// 曲线类型

    // 返回圆类型。
    CurveKind kind() const override;

    /// 定义域

    // 返回false，完整周期圆的自然参数域为整个实数域。
    bool isDomainBounded() const override;
    // 返回负无穷。
    double domainStart() const override;
    // 返回正无穷。
    double domainEnd() const override;

    /// 周期性

    // 返回圆参数周期2π。
    double period() const override;

    /// 参数查询

    // 返回指定弧度参数对应的圆上三维点。
    MyMath::Vector3 pointAt(double parameter) const override;
    // 返回指定参数处相对于弧度参数的一阶导数。
    MyMath::Vector3 firstDerivativeAt(double parameter) const override;
    // 返回指定参数处相对于弧度参数的二阶导数。
    MyMath::Vector3 secondDerivativeAt(double parameter) const override;

protected:


private:
    MyMath::Vector3 m_center; // 圆心。
    MyMath::Vector3 m_xDir; // 参数0对应的圆平面单位方向。
    MyMath::Vector3 m_yDir; // 参数π/2对应的圆平面单位方向。
    double m_radius; // 圆半径。
};

}

#endif // MYBREP_GEOMETRY_CURVE_GEOMETRY_CIRCLE_H
