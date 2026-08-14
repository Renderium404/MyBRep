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

}

namespace MyBRep
{

void Topology_Builder::addCurveOnSurface(
    Topology_Edge& edge,
    const Foundation::RefPtr<const Geometry_Surface>& surface,
    const Foundation::RefPtr<const Geometry_Curve2D>& curve,
    double curveFirstParameter,
    double curveLastParameter,
    double tolerance)
{
    MYBREP_ASSERT_MESSAGE(edge.isValid(),
                          "Topology_Builder requires a valid Topology_Edge.");
    MYBREP_ASSERT_MESSAGE(edge.isForward(),
                          "Topology_Builder Curve-on-Surface input Edge must use the TEdge Forward orientation.");
    MYBREP_ASSERT_MESSAGE(surface,
                          "Topology_Builder Curve-on-Surface requires a non-null surface.");
    MYBREP_ASSERT_MESSAGE(curve,
                          "Topology_Builder Curve-on-Surface requires a non-null 2D curve.");
    MYBREP_ASSERT_MESSAGE(isFiniteNonNegative(tolerance),
                          "Topology_Builder tolerance must be finite and non-negative.");

    Topology_TEdge& tEdge = edge.mutableTEdge();

    MYBREP_ASSERT_MESSAGE(!tEdge.hasCurveOnSurface(*surface),
                          "Topology_Edge already has a Curve-on-Surface representation for this surface.");

    validateCurveOnSurface(tEdge,
                           *surface,
                           *curve,
                           curveFirstParameter,
                           curveLastParameter,
                           tolerance);

    tEdge.addCurveOnSurface(surface,
                            curve,
                            curveFirstParameter,
                            curveLastParameter);
}

void Topology_Builder::addCurveOnClosedSurface(
    Topology_Edge& edge,
    const Foundation::RefPtr<const Geometry_Surface>& surface,
    const Foundation::RefPtr<const Geometry_Curve2D>& firstCurve,
    double firstCurveFirstParameter,
    double firstCurveLastParameter,
    const Foundation::RefPtr<const Geometry_Curve2D>& secondCurve,
    double secondCurveFirstParameter,
    double secondCurveLastParameter,
    double tolerance)
{
    MYBREP_ASSERT_MESSAGE(edge.isValid(),
                          "Topology_Builder requires a valid Topology_Edge.");
    MYBREP_ASSERT_MESSAGE(edge.isForward(),
                          "Topology_Builder seam input Edge must use the TEdge Forward orientation.");
    MYBREP_ASSERT_MESSAGE(surface,
                          "Topology_Builder seam representation requires a non-null surface.");
    MYBREP_ASSERT_MESSAGE(firstCurve && secondCurve,
                          "Topology_Builder seam representation requires two non-null 2D curves.");
    MYBREP_ASSERT_MESSAGE(surface->isUPeriodic() || surface->isVPeriodic(),
                          "Topology_Builder seam representation requires a periodic surface direction.");
    MYBREP_ASSERT_MESSAGE(isFiniteNonNegative(tolerance),
                          "Topology_Builder tolerance must be finite and non-negative.");

    Topology_TEdge& tEdge = edge.mutableTEdge();

    MYBREP_ASSERT_MESSAGE(!tEdge.hasCurveOnSurface(*surface),
                          "Topology_Edge already has a Curve-on-Surface representation for this surface.");

    validateCurveOnSurface(tEdge,
                           *surface,
                           *firstCurve,
                           firstCurveFirstParameter,
                           firstCurveLastParameter,
                           tolerance);
    validateCurveOnSurface(tEdge,
                           *surface,
                           *secondCurve,
                           secondCurveFirstParameter,
                           secondCurveLastParameter,
                           tolerance);

    tEdge.addCurveOnClosedSurface(surface,
                                  firstCurve,
                                  firstCurveFirstParameter,
                                  firstCurveLastParameter,
                                  secondCurve,
                                  secondCurveFirstParameter,
                                  secondCurveLastParameter);
}

void Topology_Builder::validateCurveOnSurface(
    const Topology_TEdge& edge,
    const Geometry_Surface& surface,
    const Geometry_Curve2D& curve,
    double curveFirstParameter,
    double curveLastParameter,
    double tolerance)
{
    MYBREP_ASSERT_MESSAGE(isFiniteValue(curveFirstParameter) &&
                          isFiniteValue(curveLastParameter),
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
