#ifndef MYBREP_GEOMETRY_CURVE2D_GEOMETRY_BSPLINE2D_H
#define MYBREP_GEOMETRY_CURVE2D_GEOMETRY_BSPLINE2D_H

#include <cstddef>
#include <vector>

#include "Geometry_Curve2D.h"

namespace MyBRep
{

// 表示由次数、二维控制点和非递减节点向量定义的完整非周期B样条曲线。
class Geometry_BSpline2D : public Geometry_Curve2D
{
public:
    // 使用指定次数、二维控制点和节点向量创建非周期B样条曲线。
    Geometry_BSpline2D(std::size_t degree,
                       const std::vector<MyMath::Vector2>& controlPoints,
                       const std::vector<double>& knots);

    /// 样条数据

    // 返回B样条曲线次数。
    std::size_t degree() const;
    // 返回二维控制点数量。
    std::size_t controlPointCount() const;
    // 返回指定二维控制点。
    const MyMath::Vector2& controlPoint(std::size_t index) const;
    // 返回完整二维控制点序列。
    const std::vector<MyMath::Vector2>& controlPoints() const;
    // 返回节点数量。
    std::size_t knotCount() const;
    // 返回指定节点值。
    double knot(std::size_t index) const;
    // 返回完整非递减节点向量。
    const std::vector<double>& knots() const;

    /// 曲线类型

    // 返回B样条曲线类型。
    CurveKind kind() const override;

    /// 定义域

    // 返回true，当前非周期二维B样条曲线具有有限自然参数域。
    bool isDomainBounded() const override;
    // 返回有效自然参数域左边界knots[degree]。
    double domainStart() const override;
    // 返回有效自然参数域右边界knots[controlPointCount]。
    double domainEnd() const override;

    /// 周期性

    // 返回0，当前Geometry_BSpline2D只表示非周期B样条曲线。
    double period() const override;

    /// 参数查询

    // 使用de Boor算法返回指定参数对应的二维B样条曲线点。
    MyMath::Vector2 pointAt(double parameter) const override;
    // 返回指定参数处的一阶导数，内部节点处必须具有至少C1连续性。
    MyMath::Vector2 firstDerivativeAt(double parameter) const override;
    // 返回指定参数处的二阶导数，内部节点处必须具有至少C2连续性。
    MyMath::Vector2 secondDerivativeAt(double parameter) const override;

protected:
    // 通过侵入式引用计数管理二维B样条曲线几何生命周期。
    ~Geometry_BSpline2D() override = default;

private:
    // 返回指定参数所在的有效节点跨度编号，定义域右端点归属于最后有效跨度。
    static std::size_t findSpan(const std::vector<double>& knots,
                                std::size_t degree,
                                std::size_t controlPointCount,
                                double parameter);
    // 使用de Boor算法对指定二维B样条数据求值。
    static MyMath::Vector2 evaluate(const std::vector<MyMath::Vector2>& controlPoints,
                                    const std::vector<double>& knots,
                                    std::size_t degree,
                                    double parameter);
    // 构造当前二维B样条的一阶导数控制点和节点向量。
    static void buildDerivativeCurve(const std::vector<MyMath::Vector2>& controlPoints,
                                     const std::vector<double>& knots,
                                     std::size_t degree,
                                     std::vector<MyMath::Vector2>& derivativeControlPoints,
                                     std::vector<double>& derivativeKnots);
    // 返回指定内部节点的精确重复次数，非内部节点参数返回0。
    std::size_t internalKnotMultiplicity(double parameter) const;
    // 判断指定参数处是否存在要求阶数的唯一导数。
    bool isDerivativeDefined(double parameter, std::size_t derivativeOrder) const;

private:
    std::size_t m_degree; // B样条次数。
    std::vector<MyMath::Vector2> m_controlPoints; // 二维控制点序列。
    std::vector<double> m_knots; // 非递减节点向量。
};

}

#endif // MYBREP_GEOMETRY_CURVE2D_GEOMETRY_BSPLINE2D_H
