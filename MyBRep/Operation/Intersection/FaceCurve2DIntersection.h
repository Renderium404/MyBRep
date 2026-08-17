#ifndef MYBREP_OPERATION_INTERSECTION_FACECURVE2DINTERSECTION_H
#define MYBREP_OPERATION_INTERSECTION_FACECURVE2DINTERSECTION_H

#include <cstddef>
#include <vector>

#include "MyBRep/Operation/Intersection/WireCurve2DIntersection.h"
#include "MyBRep/Topology/Face/Topology_Face.h"

namespace MyBRep
{
namespace Operation
{
namespace Intersection
{

// 描述一条cutting二维曲线与Face某个trimming Wire边界的一个离散命中。
struct FaceCurve2DIntersectionPoint
{
    MyMath::Vector2 point; // Surface参数空间中的二维交点。
    std::size_t wireIndex; // 当前Face使用方向下命中的trimming Wire索引。
    std::size_t edgeIndex; // 当前Wire遍历方向下命中的Edge-use索引。
    double edgeParameter; // 当前Edge-use上的规范化参数[0,1]，可直接用于WireSplitLocation。
    double edgeCurveParameter; // 当前Edge在Face Surface上的P-Curve自然参数。
    double curveParameter; // cutting完整二维曲线参数。
    double curveNormalizedParameter; // cutting有限有向区间上的规范化参数[0,1]。
};

// 保存cutting二维曲线与Face全部trimming Wire之间的离散命中或连续边界重合状态。
struct FaceCurve2DIntersectionResult
{
    FaceCurve2DIntersectionResult();

    CurveCurve2DIntersectionKind kind; // None、Points或Overlap。
    std::vector<FaceCurve2DIntersectionPoint> points; // kind为Points时按cutting曲线当前使用方向排列。
    std::vector<std::size_t> overlapWireIndices; // kind为Overlap时记录发生连续重合的Face Wire索引。
};

// 将Face全部trimming Wire与cutting二维曲线有限有向区间求交；不执行Face区域分类和Hole语义判断。
FaceCurve2DIntersectionResult intersectFaceCurve2D(const Topology_Face& face, const Geometry_Curve2D& curve,
                                                    double curveFirstParameter, double curveLastParameter,
                                                    double tolerance = MyMath::Vector2::DefaultEpsilon);

}
}
}

#endif // MYBREP_OPERATION_INTERSECTION_FACECURVE2DINTERSECTION_H