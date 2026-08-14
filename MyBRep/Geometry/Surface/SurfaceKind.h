#ifndef MYBREP_GEOMETRY_SURFACE_SURFACEKIND_H
#define MYBREP_GEOMETRY_SURFACE_SURFACEKIND_H

namespace MyBRep
{

// 标识MyBRep当前支持的完整参数曲面几何类型。
enum class SurfaceKind
{
    Unknown, // 无法识别或尚未归类的曲面类型。
    Plane, // 三维无限平面。
    Cylindrical, // 三维完整无限圆柱面。
    Conical, // 三维完整单侧无限圆锥面。
    Spherical, // 三维完整球面。
    Extrusion, // 由完整参数曲线沿固定方向生成的拉伸曲面。
    Revolution, // 由完整参数曲线绕固定轴生成的旋转曲面。
    Bezier, // 三维有限非周期Bezier曲面。
    BSpline // 三维有限非周期B样条曲面。
};

}

#endif // MYBREP_GEOMETRY_SURFACE_SURFACEKIND_H