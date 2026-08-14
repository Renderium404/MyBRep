#ifndef MYBREP_GEOMETRY_SURFACE_GEOMETRY_BEZIERSURFACE_H
#define MYBREP_GEOMETRY_SURFACE_GEOMETRY_BEZIERSURFACE_H

#include <cstddef>
#include <vector>

#include "Geometry_Surface.h"

namespace MyBRep
{

// 表示由二维控制点网格定义的完整非周期Bezier曲面，U/V自然参数域均固定为[0,1]。
class Geometry_BezierSurface : public Geometry_Surface
{
public:
    // 使用U/V方向至少各两个控制点的二维控制网格创建Bezier曲面，控制点按V行、U列顺序展平存储。
    Geometry_BezierSurface(std::size_t uControlPointCount,
                           std::size_t vControlPointCount,
                           const std::vector<MyMath::Vector3>& controlPoints);

    /// 控制数据

    // 返回U方向控制点数量。
    std::size_t uControlPointCount() const;
    // 返回V方向控制点数量。
    std::size_t vControlPointCount() const;
    // 返回U方向Bezier次数。
    std::size_t uDegree() const;
    // 返回V方向Bezier次数。
    std::size_t vDegree() const;
    // 返回指定U/V索引处的控制点。
    const MyMath::Vector3& controlPoint(std::size_t uIndex, std::size_t vIndex) const;
    // 返回按V行、U列顺序展平存储的完整控制点序列。
    const std::vector<MyMath::Vector3>& controlPoints() const;

    /// 曲面类型

    // 返回Bezier曲面类型。
    SurfaceKind kind() const override;

    /// U定义域

    // 返回true，Bezier曲面的U自然参数域为有限区间[0,1]。
    bool isUDomainBounded() const override;
    // 返回U自然参数域左边界0。
    double uDomainStart() const override;
    // 返回U自然参数域右边界1。
    double uDomainEnd() const override;

    /// V定义域

    // 返回true，Bezier曲面的V自然参数域为有限区间[0,1]。
    bool isVDomainBounded() const override;
    // 返回V自然参数域左边界0。
    double vDomainStart() const override;
    // 返回V自然参数域右边界1。
    double vDomainEnd() const override;

    /// 周期性

    // 返回0，Bezier曲面的U方向不是周期参数方向。
    double uPeriod() const override;
    // 返回0，Bezier曲面的V方向不是周期参数方向。
    double vPeriod() const override;

    /// 参数查询

    // 使用张量积de Casteljau算法返回指定U/V参数对应的三维曲面点。
    MyMath::Vector3 pointAt(double u, double v) const override;

    /// 一阶偏导

    // 返回指定参数处相对于U参数的一阶偏导。
    MyMath::Vector3 firstDerivativeUAt(double u, double v) const override;
    // 返回指定参数处相对于V参数的一阶偏导。
    MyMath::Vector3 firstDerivativeVAt(double u, double v) const override;

    /// 二阶偏导

    // 返回指定参数处相对于U参数的二阶偏导，一次U方向Bezier曲面恒返回零向量。
    MyMath::Vector3 secondDerivativeUUAt(double u, double v) const override;
    // 返回指定参数处相对于U和V参数的混合二阶偏导。
    MyMath::Vector3 secondDerivativeUVAt(double u, double v) const override;
    // 返回指定参数处相对于V参数的二阶偏导，一次V方向Bezier曲面恒返回零向量。
    MyMath::Vector3 secondDerivativeVVAt(double u, double v) const override;

protected:
    // 通过侵入式引用计数管理Bezier曲面几何生命周期。
    ~Geometry_BezierSurface() override = default;

private:
    // 返回展平控制点序列中指定U/V索引对应的位置。
    static std::size_t controlPointIndex(std::size_t uIndex,
                                         std::size_t vIndex,
                                         std::size_t uControlPointCount);
    // 使用de Casteljau算法对一维控制点序列进行参数求值。
    static MyMath::Vector3 evaluateCurve(const std::vector<MyMath::Vector3>& points, double parameter);
    // 使用张量积de Casteljau算法对指定二维控制网格进行参数求值。
    static MyMath::Vector3 evaluateSurface(const std::vector<MyMath::Vector3>& controlPoints,
                                           std::size_t uControlPointCount,
                                           std::size_t vControlPointCount,
                                           double u,
                                           double v);
    // 构造相对于U参数的一阶导数控制网格。
    static void buildUDerivativeControlNet(const std::vector<MyMath::Vector3>& controlPoints,
                                           std::size_t uControlPointCount,
                                           std::size_t vControlPointCount,
                                           std::vector<MyMath::Vector3>& derivativeControlPoints);
    // 构造相对于V参数的一阶导数控制网格。
    static void buildVDerivativeControlNet(const std::vector<MyMath::Vector3>& controlPoints,
                                           std::size_t uControlPointCount,
                                           std::size_t vControlPointCount,
                                           std::vector<MyMath::Vector3>& derivativeControlPoints);

private:
    std::size_t m_uControlPointCount; // U方向控制点数量。
    std::size_t m_vControlPointCount; // V方向控制点数量。
    std::vector<MyMath::Vector3> m_controlPoints; // 按V行、U列顺序展平存储的二维控制点网格。
};

}

#endif // MYBREP_GEOMETRY_SURFACE_GEOMETRY_BEZIERSURFACE_H