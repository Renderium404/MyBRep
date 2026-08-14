#ifndef MYBREP_GEOMETRY_SURFACE_GEOMETRY_BSPLINESURFACE_H
#define MYBREP_GEOMETRY_SURFACE_GEOMETRY_BSPLINESURFACE_H

#include <cstddef>
#include <vector>

#include "Geometry_Surface.h"

namespace MyBRep
{

// 表示由U/V次数、二维控制网格和两组非递减节点向量定义的完整非周期B样条曲面。
class Geometry_BSplineSurface : public Geometry_Surface
{
public:
    // 使用指定U/V次数、二维控制网格和节点向量创建非周期B样条曲面，控制点按V行、U列顺序展平存储。
    Geometry_BSplineSurface(std::size_t uDegree,
                            std::size_t vDegree,
                            std::size_t uControlPointCount,
                            std::size_t vControlPointCount,
                            const std::vector<MyMath::Vector3>& controlPoints,
                            const std::vector<double>& uKnots,
                            const std::vector<double>& vKnots);

    /// 样条数据

    // 返回U方向B样条次数。
    std::size_t uDegree() const;
    // 返回V方向B样条次数。
    std::size_t vDegree() const;
    // 返回U方向控制点数量。
    std::size_t uControlPointCount() const;
    // 返回V方向控制点数量。
    std::size_t vControlPointCount() const;
    // 返回指定U/V索引处的控制点。
    const MyMath::Vector3& controlPoint(std::size_t uIndex, std::size_t vIndex) const;
    // 返回按V行、U列顺序展平存储的完整控制点序列。
    const std::vector<MyMath::Vector3>& controlPoints() const;
    // 返回U方向节点数量。
    std::size_t uKnotCount() const;
    // 返回V方向节点数量。
    std::size_t vKnotCount() const;
    // 返回指定U方向节点值。
    double uKnot(std::size_t index) const;
    // 返回指定V方向节点值。
    double vKnot(std::size_t index) const;
    // 返回完整U方向节点向量。
    const std::vector<double>& uKnots() const;
    // 返回完整V方向节点向量。
    const std::vector<double>& vKnots() const;

    /// 曲面类型

    // 返回B样条曲面类型。
    SurfaceKind kind() const override;

    /// U定义域

    // 返回true，当前非周期B样条曲面的U方向具有有限自然参数域。
    bool isUDomainBounded() const override;
    // 返回U方向有效自然参数域左边界uKnots[uDegree]。
    double uDomainStart() const override;
    // 返回U方向有效自然参数域右边界uKnots[uControlPointCount]。
    double uDomainEnd() const override;

    /// V定义域

    // 返回true，当前非周期B样条曲面的V方向具有有限自然参数域。
    bool isVDomainBounded() const override;
    // 返回V方向有效自然参数域左边界vKnots[vDegree]。
    double vDomainStart() const override;
    // 返回V方向有效自然参数域右边界vKnots[vControlPointCount]。
    double vDomainEnd() const override;

    /// 周期性

    // 返回0，当前Geometry_BSplineSurface的U方向不是周期参数方向。
    double uPeriod() const override;
    // 返回0，当前Geometry_BSplineSurface的V方向不是周期参数方向。
    double vPeriod() const override;

    /// 参数查询

    // 使用张量积de Boor算法返回指定U/V参数对应的三维曲面点。
    MyMath::Vector3 pointAt(double u, double v) const override;

    /// 一阶偏导

    // 返回指定参数处相对于U参数的一阶偏导，内部U节点处必须具有至少C1连续性。
    MyMath::Vector3 firstDerivativeUAt(double u, double v) const override;
    // 返回指定参数处相对于V参数的一阶偏导，内部V节点处必须具有至少C1连续性。
    MyMath::Vector3 firstDerivativeVAt(double u, double v) const override;

    /// 二阶偏导

    // 返回指定参数处相对于U参数的二阶偏导，内部U节点处必须具有至少C2连续性。
    MyMath::Vector3 secondDerivativeUUAt(double u, double v) const override;
    // 返回指定参数处相对于U和V参数的混合二阶偏导，两方向内部节点处均必须至少具有C1连续性。
    MyMath::Vector3 secondDerivativeUVAt(double u, double v) const override;
    // 返回指定参数处相对于V参数的二阶偏导，内部V节点处必须具有至少C2连续性。
    MyMath::Vector3 secondDerivativeVVAt(double u, double v) const override;

protected:
    // 通过侵入式引用计数管理B样条曲面几何生命周期。
    ~Geometry_BSplineSurface() override = default;

private:
    // 返回展平控制点序列中指定U/V索引对应的位置。
    static std::size_t controlPointIndex(std::size_t uIndex,
                                         std::size_t vIndex,
                                         std::size_t uControlPointCount);
    // 返回指定参数所在的有效节点跨度编号，定义域右端点归属于最后有效跨度。
    static std::size_t findSpan(const std::vector<double>& knots,
                                std::size_t degree,
                                std::size_t controlPointCount,
                                double parameter);
    // 使用de Boor算法对一维B样条控制点序列进行参数求值。
    static MyMath::Vector3 evaluateCurve(const std::vector<MyMath::Vector3>& controlPoints,
                                         const std::vector<double>& knots,
                                         std::size_t degree,
                                         double parameter);
    // 使用张量积de Boor算法对指定二维B样条数据进行参数求值。
    static MyMath::Vector3 evaluateSurface(const std::vector<MyMath::Vector3>& controlPoints,
                                           std::size_t uControlPointCount,
                                           std::size_t vControlPointCount,
                                           const std::vector<double>& uKnots,
                                           const std::vector<double>& vKnots,
                                           std::size_t uDegree,
                                           std::size_t vDegree,
                                           double u,
                                           double v);
    // 构造相对于U参数的一阶导数B样条控制网格和U节点向量。
    static void buildUDerivativeSurface(const std::vector<MyMath::Vector3>& controlPoints,
                                        std::size_t uControlPointCount,
                                        std::size_t vControlPointCount,
                                        const std::vector<double>& uKnots,
                                        std::size_t uDegree,
                                        std::vector<MyMath::Vector3>& derivativeControlPoints,
                                        std::vector<double>& derivativeUKnots);
    // 构造相对于V参数的一阶导数B样条控制网格和V节点向量。
    static void buildVDerivativeSurface(const std::vector<MyMath::Vector3>& controlPoints,
                                        std::size_t uControlPointCount,
                                        std::size_t vControlPointCount,
                                        const std::vector<double>& vKnots,
                                        std::size_t vDegree,
                                        std::vector<MyMath::Vector3>& derivativeControlPoints,
                                        std::vector<double>& derivativeVKnots);
    // 返回指定内部U参数节点的精确重复次数，非内部节点参数返回0。
    std::size_t internalUKnotMultiplicity(double parameter) const;
    // 返回指定内部V参数节点的精确重复次数，非内部节点参数返回0。
    std::size_t internalVKnotMultiplicity(double parameter) const;
    // 判断指定U参数处是否存在要求阶数的唯一U方向导数。
    bool isUDerivativeDefined(double parameter, std::size_t derivativeOrder) const;
    // 判断指定V参数处是否存在要求阶数的唯一V方向导数。
    bool isVDerivativeDefined(double parameter, std::size_t derivativeOrder) const;

private:
    std::size_t m_uDegree; // U方向B样条次数。
    std::size_t m_vDegree; // V方向B样条次数。
    std::size_t m_uControlPointCount; // U方向控制点数量。
    std::size_t m_vControlPointCount; // V方向控制点数量。
    std::vector<MyMath::Vector3> m_controlPoints; // 按V行、U列顺序展平存储的二维控制点网格。
    std::vector<double> m_uKnots; // U方向非递减节点向量。
    std::vector<double> m_vKnots; // V方向非递减节点向量。
};

}

#endif // MYBREP_GEOMETRY_SURFACE_GEOMETRY_BSPLINESURFACE_H