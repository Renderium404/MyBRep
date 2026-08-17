#include "FaceCurve2DIntersection.h"

#include <algorithm>
#include <limits>

#include "MyBRep/Foundation/Diagnostic.h"

namespace
{

// 判断标量是否为有限值。
bool isFiniteValue(double value)
{
    const double infinity = (std::numeric_limits<double>::infinity)();
    return value == value && value != infinity && value != -infinity;
}

// 判断标量是否为有限非负值。
bool isFiniteNonNegative(double value)
{
    return isFiniteValue(value) && value >= 0.0;
}

// 将Wire级离散命中提升为Face级命中并增加wireIndex。
MyBRep::Operation::Intersection::FaceCurve2DIntersectionPoint makeFacePoint(
    std::size_t wireIndex, const MyBRep::Operation::Intersection::WireCurve2DIntersectionPoint& point)
{
    MyBRep::Operation::Intersection::FaceCurve2DIntersectionPoint result;
    result.point = point.point;
    result.wireIndex = wireIndex;
    result.edgeIndex = point.edgeIndex;
    result.edgeParameter = point.edgeParameter;
    result.edgeCurveParameter = point.edgeCurveParameter;
    result.curveParameter = point.curveParameter;
    result.curveNormalizedParameter = point.curveNormalizedParameter;
    return result;
}

}

namespace MyBRep
{
namespace Operation
{
namespace Intersection
{

FaceCurve2DIntersectionResult::FaceCurve2DIntersectionResult()
    : kind(CurveCurve2DIntersectionKind::None)
{
}

FaceCurve2DIntersectionResult intersectFaceCurve2D(const Topology_Face& face, const Geometry_Curve2D& curve,
                                                    double curveFirstParameter, double curveLastParameter, double tolerance)
{
    MYBREP_ASSERT_MESSAGE(face.isValid(), "FaceCurve2DIntersection requires a valid Topology_Face.");
    MYBREP_ASSERT_MESSAGE(isFiniteValue(curveFirstParameter) && isFiniteValue(curveLastParameter) && curveFirstParameter != curveLastParameter,
                          "FaceCurve2DIntersection cutting curve finite-use parameters must be finite and non-degenerate.");
    MYBREP_ASSERT_MESSAGE(curve.isParameterInDomain(curveFirstParameter) && curve.isParameterInDomain(curveLastParameter),
                          "FaceCurve2DIntersection cutting curve parameters must lie in the natural parameter domain.");
    MYBREP_ASSERT_MESSAGE(isFiniteNonNegative(tolerance), "FaceCurve2DIntersection tolerance must be finite and non-negative.");

    FaceCurve2DIntersectionResult result;

    for (std::size_t wireIndex = 0; wireIndex < face.wireCount(); ++wireIndex)
    {
        const WireCurve2DIntersectionResult wireResult =
            intersectWireCurve2D(face.wire(wireIndex), face.geometry(), curve, curveFirstParameter, curveLastParameter, tolerance);

        if (wireResult.kind == CurveCurve2DIntersectionKind::Overlap)
        {
            result.overlapWireIndices.push_back(wireIndex);
            continue;
        }

        for (std::size_t pointIndex = 0; pointIndex < wireResult.points.size(); ++pointIndex)
        {
            result.points.push_back(makeFacePoint(wireIndex, wireResult.points[pointIndex]));
        }
    }

    if (!result.overlapWireIndices.empty())
    {
        result.kind = CurveCurve2DIntersectionKind::Overlap;
        result.points.clear();
        return result;
    }

    if (result.points.empty())
    {
        return result;
    }

    std::sort(result.points.begin(), result.points.end(),
              [](const FaceCurve2DIntersectionPoint& first, const FaceCurve2DIntersectionPoint& second)
              {
                  if (first.curveNormalizedParameter != second.curveNormalizedParameter)
                  {
                      return first.curveNormalizedParameter < second.curveNormalizedParameter;
                  }

                  if (first.wireIndex != second.wireIndex)
                  {
                      return first.wireIndex < second.wireIndex;
                  }

                  return first.edgeIndex < second.edgeIndex;
              });

    result.kind = CurveCurve2DIntersectionKind::Points;
    return result;
}

}
}
}