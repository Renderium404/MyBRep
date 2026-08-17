#ifndef MYBREP_GEOMETRY_CURVE2D_GEOMETRY_BEZIER2D_H
#define MYBREP_GEOMETRY_CURVE2D_GEOMETRY_BEZIER2D_H

#include <cstddef>
#include <vector>

#include "Geometry_Curve2D.h"

namespace MyBRep
{

// 表示由至少两个二维控制点定义的完整非周期Bezier曲线，自然参数域固定为[0,1]。
class Geometry_Bezier2D : public Geometry_Curve2D
{
public:
    // 使用至少两个有限二维控制点创建Bezier曲线，控制点不能全部重合。
    explicit Geometry_Bezier2D(const std::vector<MyMath::Vector2>& controlPoints);
    // 通过侵入式引用计数管理二维Bezier曲线几何生命周期。
    ~Geometry_Bezier2D() override = default;
    /// 控制数据

    // 返回控制点数量。
    std::size_t controlPointCount() const;
    // 返回Bezier曲线次数，等于控制点数量减一。
    std::size_t degree() const;
    // 返回指定控制点。
    const MyMath::Vector2& controlPoint(std::size_t index) const;
    // 返回完整二维控制点序列。
    const std::vector<MyMath::Vector2>& controlPoints() const;

    /// 曲线类型

    // 返回Bezier曲线类型。
    CurveKind kind() const override;

    /// 定义域

    // 返回true，Bezier二维曲线自然参数域为有限区间[0,1]。
    bool isDomainBounded() const override;
    // 返回自然参数域左边界0。
    double domainStart() const override;
    // 返回自然参数域右边界1。
    double domainEnd() const override;

    /// 周期性

    // 返回0，普通Bezier二维曲线不是周期参数曲线。
    double period() const override;

    /// 参数查询

    // 使用de Casteljau算法返回指定参数对应的二维Bezier曲线点。
    MyMath::Vector2 pointAt(double parameter) const override;
    // 返回指定参数处相对于Bezier自然参数的一阶导数。
    MyMath::Vector2 firstDerivativeAt(double parameter) const override;
    // 返回指定参数处相对于Bezier自然参数的二阶导数，一次Bezier曲线恒返回零向量。
    MyMath::Vector2 secondDerivativeAt(double parameter) const override;

protected:


private:
    // 使用de Casteljau算法对指定二维控制点序列进行参数求值。
    static MyMath::Vector2 evaluate(const std::vector<MyMath::Vector2>& points, double parameter);
    // 校验控制点序列必须有限且定义非退化曲线。
    static void validateControlPoints(const std::vector<MyMath::Vector2>& controlPoints);

private:
    std::vector<MyMath::Vector2> m_controlPoints; // 按Bezier参数增大方向保存的完整二维控制点序列。
};

}

#endif // MYBREP_GEOMETRY_CURVE2D_GEOMETRY_BEZIER2D_H
