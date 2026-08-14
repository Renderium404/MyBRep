#ifndef MYBREP_GEOMETRY_CURVE2D_GEOMETRY_LINE2D_H
#define MYBREP_GEOMETRY_CURVE2D_GEOMETRY_LINE2D_H

#include "Geometry_Curve2D.h"

namespace MyBRep
{

// 表示二维参数空间中的完整无限直线，参数表示沿单位方向相对参数原点的有符号距离。
class Geometry_Line2D : public Geometry_Curve2D
{
public:
    // 使用参数原点和非零方向创建无限二维直线，输入方向会规范化为单位方向。
    Geometry_Line2D(const MyMath::Vector2& origin, const MyMath::Vector2& direction);

    /// 直线数据

    // 返回参数值为0时对应的二维直线原点。
    const MyMath::Vector2& origin() const;
    // 返回参数增大方向对应的二维单位方向。
    const MyMath::Vector2& direction() const;

    /// 曲线类型

    // 返回直线类型。
    CurveKind kind() const override;

    /// 定义域

    // 返回false，二维直线自然参数域向两侧无限延伸。
    bool isDomainBounded() const override;
    // 返回负无穷。
    double domainStart() const override;
    // 返回正无穷。
    double domainEnd() const override;

    /// 周期性

    // 返回0，二维直线不是周期参数曲线。
    double period() const override;

    /// 参数查询

    // 返回origin + direction * parameter对应的二维点。
    MyMath::Vector2 pointAt(double parameter) const override;
    // 返回恒定的二维单位直线方向。
    MyMath::Vector2 firstDerivativeAt(double parameter) const override;
    // 返回零向量，直线二阶导数恒为零。
    MyMath::Vector2 secondDerivativeAt(double parameter) const override;

protected:
    // 通过侵入式引用计数管理二维直线几何生命周期。
    ~Geometry_Line2D() override = default;

private:
    MyMath::Vector2 m_origin; // 参数值为0时对应的二维直线原点。
    MyMath::Vector2 m_direction; // 参数增大方向对应的二维单位方向。
};

}

#endif // MYBREP_GEOMETRY_CURVE2D_GEOMETRY_LINE2D_H
