#include "WireCurve2DIntersection.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "MyBRep/Foundation/Diagnostic.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"

namespace
{

const double NormalizedParameterToleranceScale = 128.0; // 顶点重复命中的规范化参数去重覆盖数值求交舍入误差使用的固定倍数。

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

// 判断两个二维点是否在几何容差内重合。
bool pointsNear(const MyMath::Vector2& first, const MyMath::Vector2& second, double tolerance)
{
    return (first - second).length() <= tolerance;
}

// 将命中Edge终点的结果规范化到下一条Edge起点，保证闭合Wire共享顶点只有一个稳定WireSplitLocation表示。
void canonicalizeBoundaryLocation(const MyBRep::Topology_Wire& wire, const MyBRep::Geometry_Surface& surface,
                                  const MyBRep::Topology_Edge& edge, std::size_t sourceEdgeIndex,
                                  const MyMath::Vector2& point, double tolerance, std::size_t& edgeIndex, double& edgeParameter)
{
    edgeIndex = sourceEdgeIndex;

    if (pointsNear(point, edge.surfaceParameterAt(surface, 0.0), tolerance))
    {
        edgeParameter = 0.0;
        return;
    }

    if (pointsNear(point, edge.surfaceParameterAt(surface, 1.0), tolerance))
    {
        edgeIndex = (sourceEdgeIndex + 1) % wire.edgeCount();
        edgeParameter = 0.0;
        return;
    }

    edgeParameter = (std::max)(0.0, (std::min)(1.0, edgeParameter));
}

// 判断两个Wire命中是否为同一个cutting曲线参数位置上的同一边界点。
bool sameHit(const MyBRep::Operation::Intersection::WireCurve2DIntersectionPoint& first,
             const MyBRep::Operation::Intersection::WireCurve2DIntersectionPoint& second, double tolerance)
{
    const double normalizedTolerance = (std::sqrt)((std::numeric_limits<double>::epsilon)()) * NormalizedParameterToleranceScale;
    return pointsNear(first.point, second.point, tolerance) &&
           std::fabs(first.curveNormalizedParameter - second.curveNormalizedParameter) <= normalizedTolerance;
}

// 向Wire结果加入去重后的离散命中。
void addHit(MyBRep::Operation::Intersection::WireCurve2DIntersectionResult& result,
            const MyBRep::Topology_Wire& wire, const MyBRep::Geometry_Surface& surface,
            const MyBRep::Topology_Edge& edge, std::size_t sourceEdgeIndex,
            const MyBRep::Operation::Intersection::CurveCurve2DIntersectionPoint& intersection, double tolerance)
{
    MyBRep::Operation::Intersection::WireCurve2DIntersectionPoint hit;
    hit.point = intersection.point;
    hit.edgeIndex = sourceEdgeIndex;
    hit.edgeParameter = intersection.firstNormalizedParameter;
    hit.edgeCurveParameter = intersection.firstParameter;
    hit.curveParameter = intersection.secondParameter;
    hit.curveNormalizedParameter = intersection.secondNormalizedParameter;

    canonicalizeBoundaryLocation(wire, surface, edge, sourceEdgeIndex, hit.point, tolerance, hit.edgeIndex, hit.edgeParameter);

    if (hit.edgeParameter == 0.0)
    {
        const MyBRep::Topology_Edge canonicalEdge = wire.edge(hit.edgeIndex);
        hit.edgeCurveParameter = canonicalEdge.curveOnSurfaceFirstParameter(surface);
    }

    for (std::size_t index = 0; index < result.points.size(); ++index)
    {
        if (sameHit(result.points[index], hit, tolerance))
        {
            // 共享顶点由两条相邻Edge同时报告时，以规范化后的起点表示覆盖先前结果。
            if (hit.edgeParameter == 0.0)
            {
                result.points[index] = hit;
            }

            return;
        }
    }

    result.points.push_back(hit);
}

}

namespace MyBRep
{
namespace Operation
{
namespace Intersection
{

WireCurve2DIntersectionResult::WireCurve2DIntersectionResult()
    : kind(CurveCurve2DIntersectionKind::None)
{
}

WireCurve2DIntersectionResult intersectWireCurve2D(const Topology_Wire& wire, const Geometry_Surface& surface,
                                                    const Geometry_Curve2D& curve, double curveFirstParameter, double curveLastParameter,
                                                    double tolerance)
{
    MYBREP_ASSERT_MESSAGE(wire.isValid() && wire.isClosed(), "WireCurve2DIntersection requires a valid closed trimming Wire.");
    MYBREP_ASSERT_MESSAGE(wire.edgeCount() > 0, "WireCurve2DIntersection requires at least one boundary Edge.");
    MYBREP_ASSERT_MESSAGE(isFiniteValue(curveFirstParameter) && isFiniteValue(curveLastParameter) && curveFirstParameter != curveLastParameter,
                          "WireCurve2DIntersection cutting curve finite-use parameters must be finite and non-degenerate.");
    MYBREP_ASSERT_MESSAGE(curve.isParameterInDomain(curveFirstParameter) && curve.isParameterInDomain(curveLastParameter),
                          "WireCurve2DIntersection cutting curve parameters must lie in the natural parameter domain.");
    MYBREP_ASSERT_MESSAGE(isFiniteNonNegative(tolerance), "WireCurve2DIntersection tolerance must be finite and non-negative.");

    WireCurve2DIntersectionResult result;

    for (std::size_t edgeIndex = 0; edgeIndex < wire.edgeCount(); ++edgeIndex)
    {
        const Topology_Edge edge = wire.edge(edgeIndex);
        MYBREP_ASSERT_MESSAGE(edge.hasCurveOnSurface(surface),
                              "Every WireCurve2DIntersection boundary Edge requires a P-Curve on the requested Surface.");

        const Geometry_Curve2D& edgeCurve = edge.curveOnSurface(surface);
        const double edgeFirstParameter = edge.curveOnSurfaceFirstParameter(surface);
        const double edgeLastParameter = edge.curveOnSurfaceLastParameter(surface);
        const CurveCurve2DIntersectionResult edgeResult = intersectCurveCurve2D(edgeCurve, edgeFirstParameter, edgeLastParameter,
                                                                                curve, curveFirstParameter, curveLastParameter, tolerance);

        if (edgeResult.kind == CurveCurve2DIntersectionKind::Overlap)
        {
            result.kind = CurveCurve2DIntersectionKind::Overlap;
            result.points.clear();
            return result;
        }

        for (std::size_t pointIndex = 0; pointIndex < edgeResult.points.size(); ++pointIndex)
        {
            addHit(result, wire, surface, edge, edgeIndex, edgeResult.points[pointIndex], tolerance);
        }
    }

    if (result.points.empty())
    {
        return result;
    }

    std::sort(result.points.begin(), result.points.end(),
              [](const WireCurve2DIntersectionPoint& first, const WireCurve2DIntersectionPoint& second)
              {
                  return first.curveNormalizedParameter < second.curveNormalizedParameter;
              });

    result.kind = CurveCurve2DIntersectionKind::Points;
    return result;
}

}
}
}