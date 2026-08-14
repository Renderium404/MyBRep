#include "Topology_Builder.h"

#include <cmath>
#include <limits>

#include "MyBRep/Foundation/Diagnostic.h"

namespace
{

// 判断数值是否为有限值。
bool isFiniteValue(double value)
{
    const double infinity = (std::numeric_limits<double>::infinity)();
    return value == value && value != infinity && value != -infinity;
}

// 判断数值是否为有限非负数。
bool isFiniteNonNegative(double value)
{
    return isFiniteValue(value) && value >= 0.0;
}

// 将规范化Edge参数线性映射到指定二维曲线参数区间。
double mapParameter(double firstParameter, double lastParameter, double normalizedParameter)
{
    return firstParameter + (lastParameter - firstParameter) * normalizedParameter;
}

}

namespace MyBRep
{

void Topology_Builder::addCurveOnSurface(Topology_Edge& edge, const Foundation::RefPtr<const Geometry_Surface>& surface,
                                         const Foundation::RefPtr<const Geometry_Curve2D>& curve, double curveFirstParameter,
                                         double curveLastParameter, double tolerance)
{
    MYBREP_ASSERT_MESSAGE(edge.isValid(), "Topology_Builder requires a valid Topology_Edge.");
    MYBREP_ASSERT_MESSAGE(edge.isForward(), "Topology_Builder Curve-on-Surface input Edge must use the TEdge Forward orientation.");
    MYBREP_ASSERT_MESSAGE(surface, "Topology_Builder Curve-on-Surface requires a non-null surface.");
    MYBREP_ASSERT_MESSAGE(curve, "Topology_Builder Curve-on-Surface requires a non-null 2D curve.");
    MYBREP_ASSERT_MESSAGE(isFiniteNonNegative(tolerance), "Topology_Builder tolerance must be finite and non-negative.");

    Topology_TEdge& tEdge = edge.mutableTEdge();
    MYBREP_ASSERT_MESSAGE(!tEdge.hasCurveOnSurface(*surface), "Topology_Edge already has a Curve-on-Surface representation for this surface.");

    validateCurveOnSurface(tEdge, *surface, *curve, curveFirstParameter, curveLastParameter, tolerance);
    tEdge.addCurveOnSurface(surface, curve, curveFirstParameter, curveLastParameter);
}

void Topology_Builder::addCurveOnClosedSurface(Topology_Edge& edge, const Foundation::RefPtr<const Geometry_Surface>& surface,
                                               const Foundation::RefPtr<const Geometry_Curve2D>& firstCurve,
                                               double firstCurveFirstParameter, double firstCurveLastParameter,
                                               const Foundation::RefPtr<const Geometry_Curve2D>& secondCurve,
                                               double secondCurveFirstParameter, double secondCurveLastParameter,
                                               double tolerance)
{
    MYBREP_ASSERT_MESSAGE(edge.isValid(), "Topology_Builder requires a valid Topology_Edge.");
    MYBREP_ASSERT_MESSAGE(edge.isForward(), "Topology_Builder seam input Edge must use the TEdge Forward orientation.");
    MYBREP_ASSERT_MESSAGE(surface, "Topology_Builder seam representation requires a non-null surface.");
    MYBREP_ASSERT_MESSAGE(firstCurve && secondCurve, "Topology_Builder seam representation requires two non-null 2D curves.");
    MYBREP_ASSERT_MESSAGE(surface->isUPeriodic() || surface->isVPeriodic(), "Topology_Builder seam representation requires a periodic surface direction.");
    MYBREP_ASSERT_MESSAGE(isFiniteNonNegative(tolerance), "Topology_Builder tolerance must be finite and non-negative.");

    Topology_TEdge& tEdge = edge.mutableTEdge();
    MYBREP_ASSERT_MESSAGE(!tEdge.hasCurveOnSurface(*surface), "Topology_Edge already has a Curve-on-Surface representation for this surface.");

    validateCurveOnSurface(tEdge, *surface, *firstCurve, firstCurveFirstParameter, firstCurveLastParameter, tolerance);
    validateCurveOnSurface(tEdge, *surface, *secondCurve, secondCurveFirstParameter, secondCurveLastParameter, tolerance);
    tEdge.addCurveOnClosedSurface(surface, firstCurve, firstCurveFirstParameter, firstCurveLastParameter,
                                  secondCurve, secondCurveFirstParameter, secondCurveLastParameter);
}

void Topology_Builder::copyCurveOnSurfaceRange(const Topology_Edge& sourceEdge, double firstParameter, double lastParameter,
                                               Topology_Edge& targetEdge)
{
    MYBREP_ASSERT_MESSAGE(sourceEdge.isValid() && targetEdge.isValid(), "Topology_Builder Curve-on-Surface copy requires valid source and target Edges.");
    MYBREP_ASSERT_MESSAGE(sourceEdge.isForward() && targetEdge.isForward(), "Topology_Builder Curve-on-Surface copy requires Forward source and target Edges.");
    MYBREP_ASSERT_MESSAGE(isFiniteValue(firstParameter) && isFiniteValue(lastParameter) &&
                          firstParameter >= 0.0 && firstParameter < lastParameter && lastParameter <= 1.0,
                          "Topology_Builder Curve-on-Surface copy range must satisfy 0 <= first < last <= 1.");
    MYBREP_ASSERT_MESSAGE(sourceEdge.geometryResource().get() == targetEdge.geometryResource().get(),
                          "Topology_Builder Curve-on-Surface copy requires source and target Edges to share the same Geometry_Curve resource.");

    const Topology_TEdge& source = sourceEdge.tEdge();
    Topology_TEdge& target = targetEdge.mutableTEdge();

    const double expectedFirstCurveParameter = source.firstParameter() + (source.lastParameter() - source.firstParameter()) * firstParameter;
    const double expectedLastCurveParameter = source.firstParameter() + (source.lastParameter() - source.firstParameter()) * lastParameter;
    const double scale = (std::max)(1.0, (std::max)(std::fabs(expectedFirstCurveParameter), std::fabs(expectedLastCurveParameter)));
    const double parameterTolerance = (std::numeric_limits<double>::epsilon)() * 64.0 * scale; // 覆盖同一线性参数映射重复计算产生的舍入误差。

    MYBREP_ASSERT_MESSAGE(std::fabs(target.firstParameter() - expectedFirstCurveParameter) <= parameterTolerance &&
                          std::fabs(target.lastParameter() - expectedLastCurveParameter) <= parameterTolerance,
                          "Topology_Builder target Edge parameter range must match the requested source Edge subrange.");

    for (std::size_t index = 0; index < source.m_curveOnSurfaces.size(); ++index)
    {
        const Topology_TEdge::CurveOnSurfaceRepresentation& representation = source.m_curveOnSurfaces[index];
        MYBREP_ASSERT_MESSAGE(!target.hasCurveOnSurface(*representation.surface),
                              "Topology_Builder target Edge already has a Curve-on-Surface representation for the copied Surface.");

        const double firstCurveFirstParameter = mapParameter(representation.firstCurveFirstParameter, representation.firstCurveLastParameter, firstParameter);
        const double firstCurveLastParameter = mapParameter(representation.firstCurveFirstParameter, representation.firstCurveLastParameter, lastParameter);

        if (!representation.secondCurve)
        {
            target.addCurveOnSurface(representation.surface, representation.firstCurve, firstCurveFirstParameter, firstCurveLastParameter);
            continue;
        }

        const double secondCurveFirstParameter = mapParameter(representation.secondCurveFirstParameter, representation.secondCurveLastParameter, firstParameter);
        const double secondCurveLastParameter = mapParameter(representation.secondCurveFirstParameter, representation.secondCurveLastParameter, lastParameter);

        target.addCurveOnClosedSurface(representation.surface, representation.firstCurve, firstCurveFirstParameter, firstCurveLastParameter,
                                       representation.secondCurve, secondCurveFirstParameter, secondCurveLastParameter);
    }
}

void Topology_Builder::validateCurveOnSurface(const Topology_TEdge& edge, const Geometry_Surface& surface,
                                              const Geometry_Curve2D& curve, double curveFirstParameter,
                                              double curveLastParameter, double tolerance)
{
    MYBREP_ASSERT_MESSAGE(isFiniteValue(curveFirstParameter) && isFiniteValue(curveLastParameter),
                          "Topology_Builder Curve-on-Surface parameters must be finite.");
    MYBREP_ASSERT_MESSAGE(curveFirstParameter != curveLastParameter,
                          "Topology_Builder Curve-on-Surface parameter interval must be non-degenerate.");
    MYBREP_ASSERT_MESSAGE(curve.isParameterInDomain(curveFirstParameter),
                          "Topology_Builder Curve-on-Surface first parameter is outside the 2D curve domain.");
    MYBREP_ASSERT_MESSAGE(curve.isParameterInDomain(curveLastParameter),
                          "Topology_Builder Curve-on-Surface last parameter is outside the 2D curve domain.");

    if (curve.isPeriodic())
    {
        MYBREP_ASSERT_MESSAGE(std::fabs(curveLastParameter - curveFirstParameter) <= curve.period(),
                              "Topology_Builder Curve-on-Surface interval must not exceed one 2D curve period.");
    }

    const MyMath::Vector2 firstUV = curve.pointAt(curveFirstParameter);
    const MyMath::Vector2 lastUV = curve.pointAt(curveLastParameter);
    MYBREP_ASSERT_MESSAGE(surface.isParameterInDomain(firstUV.x(), firstUV.y()),
                          "Topology_Builder Curve-on-Surface start UV is outside the surface natural parameter domain.");
    MYBREP_ASSERT_MESSAGE(surface.isParameterInDomain(lastUV.x(), lastUV.y()),
                          "Topology_Builder Curve-on-Surface end UV is outside the surface natural parameter domain.");

    const MyMath::Vector3 surfaceStart = surface.pointAt(firstUV.x(), firstUV.y());
    const MyMath::Vector3 surfaceEnd = surface.pointAt(lastUV.x(), lastUV.y());
    MYBREP_ASSERT_MESSAGE(edge.startVertex().point().isEqualTo(surfaceStart, tolerance),
                          "Topology_Builder Curve-on-Surface start point must match the TEdge Forward start vertex.");
    MYBREP_ASSERT_MESSAGE(edge.endVertex().point().isEqualTo(surfaceEnd, tolerance),
                          "Topology_Builder Curve-on-Surface end point must match the TEdge Forward end vertex.");
}

}
