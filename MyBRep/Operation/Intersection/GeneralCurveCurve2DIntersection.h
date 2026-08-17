#ifndef MYBREP_OPERATION_INTERSECTION_GENERALCURVECURVE2DINTERSECTION_H
#define MYBREP_OPERATION_INTERSECTION_GENERALCURVECURVE2DINTERSECTION_H

#include "CurveCurve2DIntersection.h"

namespace MyBRep
{
namespace Operation
{
namespace Intersection
{

// 对至少包含一条Bezier或BSpline的有限二维曲线使用执行通用数值求交，仅供CurveCurve2DIntersection统一调度。
CurveCurve2DIntersectionResult intersectGeneralCurveCurve2D(const Geometry_Curve2D& firstCurve, double firstStartParameter, double firstEndParameter,
                                                             const Geometry_Curve2D& secondCurve, double secondStartParameter, double secondEndParameter,
                                                             double tolerance);

}
}
}

#endif // MYBREP_OPERATION_INTERSECTION_GENERALCURVECURVE2DINTERSECTION_H
