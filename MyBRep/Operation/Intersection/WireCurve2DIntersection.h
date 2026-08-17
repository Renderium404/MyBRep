#ifndef MYBREP_OPERATION_INTERSECTION_WIRECURVE2DINTERSECTION_H
#define MYBREP_OPERATION_INTERSECTION_WIRECURVE2DINTERSECTION_H

#include <cstddef>
#include <vector>

#include "CurveCurve2DIntersection.h"
#include "MyBRep/Geometry/Surface/Geometry_Surface.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

namespace MyBRep
{
namespace Operation
{
namespace Intersection
{

// 描述一条cutting二维曲线与Face trimming Wire边界的一个离散命中。
struct WireCurve2DIntersectionPoint
{
    MyMath::Vector2 point; // Surface参数空间中的二维交点。
    std::size_t edgeIndex; // 当前Wire遍历方向下命中的Edge-use索引。
    double edgeParameter; // 当前Edge-use上的规范化参数[0,1]，可直接用于WireSplitLocation。
    double edgeCurveParameter; // 当前Edge在指定Surface上的P-Curve自然参数。
    double curveParameter; // cutting完整二维曲线参数。
    double curveNormalizedParameter; // cutting有限有向区间上的规范化参数[0,1]。
};

// 保存cutting二维曲线与单个闭合trimming Wire之间的离散交点或连续边界重合状态。
struct WireCurve2DIntersectionResult
{
    WireCurve2DIntersectionResult();

    CurveCurve2DIntersectionKind kind; // None、Points或Overlap。
    std::vector<WireCurve2DIntersectionPoint> points; // kind为Points时按cutting曲线当前使用方向排列。
};

// 将指定闭合Wire全部Edge在同一Surface上的P-Curve与cutting二维曲线有限有向区间求交。
WireCurve2DIntersectionResult intersectWireCurve2D(const Topology_Wire& wire, const Geometry_Surface& surface,
                                                    const Geometry_Curve2D& curve, double curveFirstParameter, double curveLastParameter,
                                                    double tolerance = MyMath::Vector2::DefaultEpsilon);

}
}
}

#endif // MYBREP_OPERATION_INTERSECTION_WIRECURVE2DINTERSECTION_H