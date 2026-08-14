#ifndef MYBREP_GEOMETRY_CURVE_GEOMETRY_LINE_H
#define MYBREP_GEOMETRY_CURVE_GEOMETRY_LINE_H

#include "Geometry_Curve.h"

namespace MyBRep
{

// 表示三维几何空间中的完整无限直线，参数表示沿单位方向相对参数原点的有符号距离。
class Geometry_Line : public Geometry_Curve
{
public:
    // 使用有限参数原点和有限非零方向创建无限直线，输入方向会规范化为单位方向。
    Geometry_Line(const MyMath::Vector3& origin, const MyMath::Vector3& direction);

    /// 直线数据

    // 返回参数值为0时对应的直线原点。
    const MyMath::Vector3& origin() const;
    // 返回参数增大方向对应的单位方向向量。
    const MyMath::Vector3& direction() const;

    /// 曲线类型

    // 返回直线类型。
    CurveKind kind() const override;

    /// 定义域

    // 返回false，直线自然参数域不是有限区间。
    bool isDomainBounded() const override;
    // 返回负无穷，直线自然参数域向负方向无限延伸。
    double domainStart() const override;
    // 返回正无穷，直线自然参数域向正方向无限延伸。
    double domainEnd() const override;

    /// 周期性

    // 返回0，直线不是周期参数曲线。
    double period() const override;

    /// 参数查询

    // 返回指定参数对应的直线点。
    MyMath::Vector3 pointAt(double parameter) const override;
    // 返回恒定的单位直线方向。
    MyMath::Vector3 firstDerivativeAt(double parameter) const override;
    // 返回零向量，直线二阶导数恒为零。
    MyMath::Vector3 secondDerivativeAt(double parameter) const override;

protected:
    // 通过侵入式引用计数管理直线几何生命周期。
    ~Geometry_Line() override = default;

private:
    MyMath::Vector3 m_origin; // 参数值为0时对应的直线原点。
    MyMath::Vector3 m_direction; // 参数增大方向对应的单位方向向量。
};

}

#endif // MYBREP_GEOMETRY_CURVE_GEOMETRY_LINE_H
