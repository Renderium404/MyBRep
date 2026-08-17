#ifndef MYBREP_OPERATION_INTERSECTION_CURVECURVE2DINTERSECTION_H
#define MYBREP_OPERATION_INTERSECTION_CURVECURVE2DINTERSECTION_H

#include <vector>

#include "MyMath/Vector2.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Curve2D.h"

namespace MyBRep
{
namespace Operation
{
namespace Intersection
{

// 描述两个有限二维曲线使用之间的求交结果类型。
enum class CurveCurve2DIntersectionKind
{
    None,
    Points,
    Overlap
};

// 描述一个离散二维交点及其在两条完整曲线和有限有向使用区间中的参数。
struct CurveCurve2DIntersectionPoint
{
    MyMath::Vector2 point; // 二维参数空间交点。
    double firstParameter; // 第一条完整二维曲线参数。
    double secondParameter; // 第二条完整二维曲线参数。
    double firstNormalizedParameter; // 第一有限有向区间上的规范化参数[0,1]。
    double secondNormalizedParameter; // 第二有限有向区间上的规范化参数[0,1]。
};

// 保存两个有限二维曲线使用的离散交点或连续重合状态。
struct CurveCurve2DIntersectionResult
{
    CurveCurve2DIntersectionResult();

    CurveCurve2DIntersectionKind kind; // 当前求交结果类型。
    std::vector<CurveCurve2DIntersectionPoint> points; // kind为Points时按第一曲线当前使用方向排列的离散交点。
};

// 对两条完整二维曲线的有限有向参数区间求交；当前精确支持Line和Circle，连续共线或共圆交集返回Overlap。
CurveCurve2DIntersectionResult intersectCurveCurve2D(const Geometry_Curve2D& firstCurve, double firstStartParameter, double firstEndParameter,
                                                      const Geometry_Curve2D& secondCurve, double secondStartParameter, double secondEndParameter,
                                                      double tolerance = MyMath::Vector2::DefaultEpsilon);

}
}
}

#endif // MYBREP_OPERATION_INTERSECTION_CURVECURVE2DINTERSECTION_H
