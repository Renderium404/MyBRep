#ifndef MYBREP_GEOMETRY_SHAPEKIND_H
#define MYBREP_GEOMETRY_SHAPEKIND_H

namespace MyBRep
{

// 标识连续几何体的标准类型。
enum class ShapeKind
{
    Custom, // 自定义连续几何体。
    Box, // 长方体。
    Sphere, // 球体。
    Cylinder, // 圆柱体。
    ConeFrustum, // 圆锥台或圆锥体。
    Extruded, // 闭合平面轮廓沿局部Z轴拉伸形成的连续几何体。
    Revolved, // 闭合平面轮廓完整旋转形成的回转几何体。
    Mesh // 封闭三角网格几何体。
};

}

#endif // MYBREP_GEOMETRY_SHAPEKIND_H