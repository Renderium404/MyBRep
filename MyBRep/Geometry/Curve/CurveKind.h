#ifndef MYBREP_GEOMETRY_CURVE_CURVEKIND_H
#define MYBREP_GEOMETRY_CURVE_CURVEKIND_H

namespace MyBRep
{

// 标识MyBRep当前支持的完整参数曲线几何类型。
enum class CurveKind
{
    Unknown, // 无法识别或尚未归类的曲线类型。
    Line, // 三维无限直线。
    Circle, // 三维完整周期圆。
    Bezier, // 三维有限非周期Bezier曲线。
    BSpline // 三维有限非周期B样条曲线。
};

}

#endif // MYBREP_GEOMETRY_CURVE_CURVEKIND_H