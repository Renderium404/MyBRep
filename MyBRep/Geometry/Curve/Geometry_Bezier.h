#ifndef MYBREP_GEOMETRY_CURVE_GEOMETRY_BEZIER_H
#define MYBREP_GEOMETRY_CURVE_GEOMETRY_BEZIER_H

#include <cstddef>
#include <vector>

#include "Geometry_Curve.h"

namespace MyBRep
{

// 表示由至少两个控制点定义的完整非周期Bezier曲线，自然参数域固定为[0,1]。
class Geometry_Bezier : public Geometry_Curve
{
public:
    // 使用至少两个有限控制点创建Bezier曲线，控制点不能全部重合。
    explicit Geometry_Bezier(const std::vector<MyMath::Vector3>& controlPoints);
    // 通过侵入式引用计数管理Bezier曲线几何生命周期。
    ~Geometry_Bezier() override = default;
    /// 控制数据

    // 返回控制点数量。
    std::size_t controlPointCount() const;
    // 返回Bezier曲线次数，等于控制点数量减一。
    std::size_t degree() const;
    // 返回指定控制点。
    const MyMath::Vector3& controlPoint(std::size_t index) const;
    // 返回完整控制点序列。
    const std::vector<MyMath::Vector3>& controlPoints() const;

    /// 曲线类型

    // 返回Bezier曲线类型。
    CurveKind kind() const override;

    /// 定义域

    // 返回true，Bezier曲线自然参数域为有限区间[0,1]。
    bool isDomainBounded() const override;
    // 返回Bezier曲线自然参数域左边界0。
    double domainStart() const override;
    // 返回Bezier曲线自然参数域右边界1。
    double domainEnd() const override;

    /// 周期性

    // 返回0，Bezier曲线不是周期参数曲线。
    double period() const override;

    /// 参数查询

    // 使用de Casteljau算法返回指定参数对应的Bezier曲线点，parameter必须位于[0,1]内。
    MyMath::Vector3 pointAt(double parameter) const override;
    // 返回指定参数处相对于Bezier自然参数的一阶导数。
    MyMath::Vector3 firstDerivativeAt(double parameter) const override;
    // 返回指定参数处相对于Bezier自然参数的二阶导数，一次Bezier曲线恒返回零向量。
    MyMath::Vector3 secondDerivativeAt(double parameter) const override;

protected:


private:
    // 使用de Casteljau算法对给定控制点序列进行参数求值。
    static MyMath::Vector3 evaluate(const std::vector<MyMath::Vector3>& points, double parameter);

private:
    std::vector<MyMath::Vector3> m_controlPoints; // 按Bezier自然参数增大方向保存的控制点序列。
};

}

#endif // MYBREP_GEOMETRY_CURVE_GEOMETRY_BEZIER_H