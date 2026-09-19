#include "EdgeModeling.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "MyMath/MathUtils.h"
#include "MyBRep/Foundation/Diagnostic.h"
#include "MyBRep/Geometry/Curve/Geometry_BSpline.h"
#include "MyBRep/Geometry/Curve/Geometry_Bezier.h"
#include "MyBRep/Geometry/Curve/Geometry_Circle.h"
#include "MyBRep/Geometry/Curve/Geometry_Line.h"

namespace MyBRep
{
namespace Modeling
{

namespace EdgeModelingDetail
{

bool isFullPeriodInterval(const Geometry_Curve& geometry,double firstParameter,double lastParameter)
{
    if (!geometry.isPeriodic())
    {
        return false;
    }

    const double span = std::fabs(lastParameter - firstParameter);
    const double period = geometry.period();
    const double scale = (std::max)(1.0, std::fabs(period));
    const double tolerance = (std::numeric_limits<double>::epsilon)() * 64.0 * scale;
    return std::fabs(span - period) <= tolerance;
}

double automaticClosureTolerance(const MyMath::Vector3& firstPoint,const MyMath::Vector3& lastPoint)
{
    const double firstScale = MyMath::maximumAbsolute(firstPoint.x(), firstPoint.y(), firstPoint.z());
    const double lastScale = MyMath::maximumAbsolute(lastPoint.x(), lastPoint.y(), lastPoint.z());
    const double scale = MyMath::maximumAbsolute(1.0, firstScale, lastScale);
    const double numericalTolerance = scale * (std::numeric_limits<double>::epsilon)() * 64.0;
    return (std::max)(firstPoint.distanceTo(lastPoint), numericalTolerance);
}

void validateCurveInterval(const Geometry_Curve& geometry,double firstParameter,double lastParameter)
{
    MYBREP_ASSERT_MESSAGE(MyMath::isFinite(firstParameter) && MyMath::isFinite(lastParameter),
                          "Edge modeling curve parameters must be finite.");
    MYBREP_ASSERT_MESSAGE(firstParameter != lastParameter,
                          "Edge modeling curve parameter interval must be non-degenerate.");
    MYBREP_ASSERT_MESSAGE(geometry.isParameterInDomain(firstParameter) && geometry.isParameterInDomain(lastParameter),
                          "Edge modeling curve parameters must lie inside the Geometry_Curve natural parameter domain.");

    if (geometry.isPeriodic())
    {
        MYBREP_ASSERT_MESSAGE(std::fabs(lastParameter - firstParameter) <= geometry.period(),
                              "Edge modeling periodic curve interval must not exceed one complete period.");
    }
}

void validateArcInput(double radius,double startAngle,double sweepAngle)
{
    MYBREP_ASSERT_MESSAGE(MyMath::isFinite(radius) && radius > 0.0,
                          "Arc modeling radius must be finite and positive.");
    MYBREP_ASSERT_MESSAGE(MyMath::isFinite(startAngle),
                          "Arc modeling start angle must be finite.");
    MYBREP_ASSERT_MESSAGE(MyMath::isFinite(sweepAngle) && sweepAngle != 0.0 && std::fabs(sweepAngle) <= MyMath::TwoPi,
                          "Arc modeling sweep must be finite, non-zero and within one complete period.");
}

Topology_Edge createAutomaticEdge(
    const Foundation::RefPtr<const Geometry_Curve>& geometry,
    double firstParameter,
    double lastParameter)
{
    MYBREP_ASSERT_MESSAGE(geometry,
                          "Edge modeling requires a non-null Geometry_Curve.");

    validateCurveInterval(*geometry, firstParameter, lastParameter);

    const MyMath::Vector3 firstPoint = geometry->pointAt(firstParameter);
    const MyMath::Vector3 lastPoint = geometry->pointAt(lastParameter);
    const bool closed = firstPoint.isEqualTo(lastPoint, 0.0) || isFullPeriodInterval(*geometry, firstParameter, lastParameter);
    const Topology_Vertex firstVertex(firstPoint);

    if (closed)
    {
        return createEdge(firstVertex, firstVertex, geometry, firstParameter, lastParameter, automaticClosureTolerance(firstPoint, lastPoint));
    }

    const Topology_Vertex lastVertex(lastPoint);
    return createEdge(firstVertex, lastVertex, geometry, firstParameter, lastParameter, 0.0);
}

}

/// 局部Topology_Edge创建

/// 通用Geometry_Curve Edge

Topology_Edge createEdge(
    const Foundation::RefPtr<const Geometry_Curve>& geometry,
    double firstParameter,
    double lastParameter)
{
    return EdgeModelingDetail::createAutomaticEdge(geometry, firstParameter, lastParameter);
}

Topology_Edge createEdge(
    const Topology_Vertex& startVertex,
    const Topology_Vertex& endVertex,
    const Foundation::RefPtr<const Geometry_Curve>& geometry,
    double firstParameter,
    double lastParameter,
    double connectionTolerance)
{
    MYBREP_ASSERT_MESSAGE(startVertex.isValid() && endVertex.isValid(),
                          "Edge modeling requires valid start and end Topology_Vertex values.");
    MYBREP_ASSERT_MESSAGE(geometry,
                          "Edge modeling requires a non-null Geometry_Curve.");
    MYBREP_ASSERT_MESSAGE(MyMath::isFinite(connectionTolerance) && connectionTolerance >= 0.0,
                          "Edge modeling connection tolerance must be finite and non-negative.");

    EdgeModelingDetail::validateCurveInterval(*geometry, firstParameter, lastParameter);

    if (firstParameter < lastParameter)
    {
        return Topology_Edge(startVertex, endVertex, geometry, firstParameter, lastParameter, connectionTolerance);
    }

    const Topology_Edge forwardEdge(endVertex, startVertex, geometry, lastParameter, firstParameter, connectionTolerance);
    return forwardEdge.reversed();
}

/// Line

Topology_Edge createLine(const MyMath::Vector3& startPoint,const MyMath::Vector3& endPoint)
{
    MYBREP_ASSERT_MESSAGE(startPoint.isFinite() && endPoint.isFinite() && !startPoint.isEqualTo(endPoint, 0.0),
                          "Line modeling requires two different finite points.");

    const Topology_Vertex startVertex(startPoint);
    const Topology_Vertex endVertex(endPoint);
    return createLine(startVertex, endVertex);
}

Topology_Edge createLine(const Topology_Vertex& startVertex,const Topology_Vertex& endVertex)
{
    MYBREP_ASSERT_MESSAGE(startVertex.isValid() && endVertex.isValid(),
                          "Line modeling requires valid Topology_Vertex values.");
    MYBREP_ASSERT_MESSAGE(!startVertex.point().isEqualTo(endVertex.point(), 0.0),
                          "Line modeling requires two Topology_Vertex values at different positions.");

    const MyMath::Vector3 direction = endVertex.point() - startVertex.point();
    const double length = direction.length();
    const Foundation::RefPtr<const Geometry_Curve> geometry(new Geometry_Line(startVertex.point(), direction));
    return createEdge(startVertex, endVertex, geometry, 0.0, length, MyMath::Vector3::DefaultEpsilon);
}

/// Circle Arc

Topology_Edge createArc(const MyMath::Vector3& center,double radius,double startAngle,double sweepAngle)
{
    MYBREP_ASSERT_MESSAGE(center.isFinite(),
                          "Arc modeling center must be finite.");
    EdgeModelingDetail::validateArcInput(radius, startAngle, sweepAngle);

    const Foundation::RefPtr<const Geometry_Curve> geometry(new Geometry_Circle(center, radius));
    return createEdge(geometry, startAngle, startAngle + sweepAngle);
}

Topology_Edge createArc(const MyMath::CoordinateSystem& coordinateSystem,double radius,double startAngle,double sweepAngle)
{
    MYBREP_ASSERT_MESSAGE(coordinateSystem.isValid(),
                          "Arc modeling coordinate system must be valid.");
    EdgeModelingDetail::validateArcInput(radius, startAngle, sweepAngle);

    const Foundation::RefPtr<const Geometry_Curve> geometry(new Geometry_Circle(coordinateSystem, radius));
    return createEdge(geometry, startAngle, startAngle + sweepAngle);
}

Topology_Edge createArc(
    const Topology_Vertex& startVertex,
    const Topology_Vertex& endVertex,
    const MyMath::Vector3& center,
    double radius,
    double startAngle,
    double sweepAngle,
    double connectionTolerance)
{
    MYBREP_ASSERT_MESSAGE(center.isFinite(),
                          "Arc modeling center must be finite.");
    EdgeModelingDetail::validateArcInput(radius, startAngle, sweepAngle);

    const Foundation::RefPtr<const Geometry_Curve> geometry(new Geometry_Circle(center, radius));

    if (EdgeModelingDetail::isFullPeriodInterval(*geometry, startAngle, startAngle + sweepAngle))
    {
        MYBREP_ASSERT_MESSAGE(startVertex.isSame(endVertex),
                              "Full-circle Arc modeling requires startVertex and endVertex to share the same topology identity.");
    }

    return createEdge(startVertex, endVertex, geometry, startAngle, startAngle + sweepAngle, connectionTolerance);
}

Topology_Edge createArc(
    const Topology_Vertex& startVertex,
    const Topology_Vertex& endVertex,
    const MyMath::CoordinateSystem& coordinateSystem,
    double radius,
    double startAngle,
    double sweepAngle,
    double connectionTolerance)
{
    MYBREP_ASSERT_MESSAGE(coordinateSystem.isValid(),
                          "Arc modeling coordinate system must be valid.");
    EdgeModelingDetail::validateArcInput(radius, startAngle, sweepAngle);

    const Foundation::RefPtr<const Geometry_Curve> geometry(new Geometry_Circle(coordinateSystem, radius));

    if (EdgeModelingDetail::isFullPeriodInterval(*geometry, startAngle, startAngle + sweepAngle))
    {
        MYBREP_ASSERT_MESSAGE(startVertex.isSame(endVertex),
                              "Full-circle Arc modeling requires startVertex and endVertex to share the same topology identity.");
    }

    return createEdge(startVertex, endVertex, geometry, startAngle, startAngle + sweepAngle, connectionTolerance);
}

/// Bezier

Topology_Edge createBezier(const std::vector<MyMath::Vector3>& controlPoints)
{
    const Foundation::RefPtr<const Geometry_Curve> geometry(new Geometry_Bezier(controlPoints));
    return createEdge(geometry, geometry->domainStart(), geometry->domainEnd());
}

Topology_Edge createBezier(
    const Topology_Vertex& startVertex,
    const Topology_Vertex& endVertex,
    const std::vector<MyMath::Vector3>& controlPoints,
    double connectionTolerance)
{
    const Foundation::RefPtr<const Geometry_Curve> geometry(new Geometry_Bezier(controlPoints));
    return createEdge(startVertex, endVertex, geometry, geometry->domainStart(), geometry->domainEnd(), connectionTolerance);
}

/// B-Spline

Topology_Edge createBSpline(
    std::size_t degree,
    const std::vector<MyMath::Vector3>& controlPoints,
    const std::vector<double>& knots)
{
    const Foundation::RefPtr<const Geometry_Curve> geometry(new Geometry_BSpline(degree, controlPoints, knots));
    return createEdge(geometry, geometry->domainStart(), geometry->domainEnd());
}

Topology_Edge createBSpline(
    const Topology_Vertex& startVertex,
    const Topology_Vertex& endVertex,
    std::size_t degree,
    const std::vector<MyMath::Vector3>& controlPoints,
    const std::vector<double>& knots,
    double connectionTolerance)
{
    const Foundation::RefPtr<const Geometry_Curve> geometry(new Geometry_BSpline(degree, controlPoints, knots));
    return createEdge(startVertex, endVertex, geometry, geometry->domainStart(), geometry->domainEnd(), connectionTolerance);
}

/// 空间Edge实例创建

/// 通用Geometry_Curve Edge实例

Edge makeEdge(
    const Foundation::RefPtr<const Geometry_Curve>& geometry,
    double firstParameter,
    double lastParameter)
{
    return Edge(createEdge(geometry, firstParameter, lastParameter));
}

Edge makeEdge(
    const Foundation::RefPtr<const Geometry_Curve>& geometry,
    double firstParameter,
    double lastParameter,
    const MyMath::Matrix4& localToWorld)
{
    return Edge(createEdge(geometry, firstParameter, lastParameter), localToWorld);
}

Edge makeEdge(
    const Topology_Vertex& startVertex,
    const Topology_Vertex& endVertex,
    const Foundation::RefPtr<const Geometry_Curve>& geometry,
    double firstParameter,
    double lastParameter,
    double connectionTolerance)
{
    return Edge(createEdge(startVertex, endVertex, geometry, firstParameter, lastParameter, connectionTolerance));
}

Edge makeEdge(
    const Topology_Vertex& startVertex,
    const Topology_Vertex& endVertex,
    const Foundation::RefPtr<const Geometry_Curve>& geometry,
    double firstParameter,
    double lastParameter,
    const MyMath::Matrix4& localToWorld,
    double connectionTolerance)
{
    return Edge(createEdge(startVertex, endVertex, geometry, firstParameter, lastParameter, connectionTolerance), localToWorld);
}

/// Line实例

Edge makeLine(const MyMath::Vector3& startPoint,const MyMath::Vector3& endPoint)
{
    return Edge(createLine(startPoint, endPoint));
}

Edge makeLine(const MyMath::Vector3& startPoint,const MyMath::Vector3& endPoint,const MyMath::Matrix4& localToWorld)
{
    return Edge(createLine(startPoint, endPoint), localToWorld);
}

Edge makeLine(const Topology_Vertex& startVertex,const Topology_Vertex& endVertex)
{
    return Edge(createLine(startVertex, endVertex));
}

Edge makeLine(const Topology_Vertex& startVertex,const Topology_Vertex& endVertex,const MyMath::Matrix4& localToWorld)
{
    return Edge(createLine(startVertex, endVertex), localToWorld);
}

/// Circle Arc实例

Edge makeArc(const MyMath::Vector3& center,double radius,double startAngle,double sweepAngle)
{
    return Edge(createArc(center, radius, startAngle, sweepAngle));
}

Edge makeArc(const MyMath::Vector3& center,double radius,double startAngle,double sweepAngle,const MyMath::Matrix4& localToWorld)
{
    return Edge(createArc(center, radius, startAngle, sweepAngle), localToWorld);
}

Edge makeArc(const MyMath::CoordinateSystem& coordinateSystem,double radius,double startAngle,double sweepAngle)
{
    return Edge(createArc(coordinateSystem, radius, startAngle, sweepAngle));
}

Edge makeArc(
    const MyMath::CoordinateSystem& coordinateSystem,
    double radius,
    double startAngle,
    double sweepAngle,
    const MyMath::Matrix4& localToWorld)
{
    return Edge(createArc(coordinateSystem, radius, startAngle, sweepAngle), localToWorld);
}

Edge makeArc(
    const Topology_Vertex& startVertex,
    const Topology_Vertex& endVertex,
    const MyMath::Vector3& center,
    double radius,
    double startAngle,
    double sweepAngle,
    double connectionTolerance)
{
    return Edge(createArc(startVertex, endVertex, center, radius, startAngle, sweepAngle, connectionTolerance));
}

Edge makeArc(
    const Topology_Vertex& startVertex,
    const Topology_Vertex& endVertex,
    const MyMath::Vector3& center,
    double radius,
    double startAngle,
    double sweepAngle,
    const MyMath::Matrix4& localToWorld,
    double connectionTolerance)
{
    return Edge(createArc(startVertex, endVertex, center, radius, startAngle, sweepAngle, connectionTolerance), localToWorld);
}

Edge makeArc(
    const Topology_Vertex& startVertex,
    const Topology_Vertex& endVertex,
    const MyMath::CoordinateSystem& coordinateSystem,
    double radius,
    double startAngle,
    double sweepAngle,
    double connectionTolerance)
{
    return Edge(createArc(startVertex, endVertex, coordinateSystem, radius, startAngle, sweepAngle, connectionTolerance));
}

Edge makeArc(
    const Topology_Vertex& startVertex,
    const Topology_Vertex& endVertex,
    const MyMath::CoordinateSystem& coordinateSystem,
    double radius,
    double startAngle,
    double sweepAngle,
    const MyMath::Matrix4& localToWorld,
    double connectionTolerance)
{
    return Edge(createArc(startVertex, endVertex, coordinateSystem, radius, startAngle, sweepAngle, connectionTolerance), localToWorld);
}

/// Bezier实例

Edge makeBezier(const std::vector<MyMath::Vector3>& controlPoints)
{
    return Edge(createBezier(controlPoints));
}

Edge makeBezier(const std::vector<MyMath::Vector3>& controlPoints,const MyMath::Matrix4& localToWorld)
{
    return Edge(createBezier(controlPoints), localToWorld);
}

Edge makeBezier(
    const Topology_Vertex& startVertex,
    const Topology_Vertex& endVertex,
    const std::vector<MyMath::Vector3>& controlPoints,
    double connectionTolerance)
{
    return Edge(createBezier(startVertex, endVertex, controlPoints, connectionTolerance));
}

Edge makeBezier(
    const Topology_Vertex& startVertex,
    const Topology_Vertex& endVertex,
    const std::vector<MyMath::Vector3>& controlPoints,
    const MyMath::Matrix4& localToWorld,
    double connectionTolerance)
{
    return Edge(createBezier(startVertex, endVertex, controlPoints, connectionTolerance), localToWorld);
}

/// B-Spline实例

Edge makeBSpline(
    std::size_t degree,
    const std::vector<MyMath::Vector3>& controlPoints,
    const std::vector<double>& knots)
{
    return Edge(createBSpline(degree, controlPoints, knots));
}

Edge makeBSpline(
    std::size_t degree,
    const std::vector<MyMath::Vector3>& controlPoints,
    const std::vector<double>& knots,
    const MyMath::Matrix4& localToWorld)
{
    return Edge(createBSpline(degree, controlPoints, knots), localToWorld);
}

Edge makeBSpline(
    const Topology_Vertex& startVertex,
    const Topology_Vertex& endVertex,
    std::size_t degree,
    const std::vector<MyMath::Vector3>& controlPoints,
    const std::vector<double>& knots,
    double connectionTolerance)
{
    return Edge(createBSpline(startVertex, endVertex, degree, controlPoints, knots, connectionTolerance));
}

Edge makeBSpline(
    const Topology_Vertex& startVertex,
    const Topology_Vertex& endVertex,
    std::size_t degree,
    const std::vector<MyMath::Vector3>& controlPoints,
    const std::vector<double>& knots,
    const MyMath::Matrix4& localToWorld,
    double connectionTolerance)
{
    return Edge(createBSpline(startVertex, endVertex, degree, controlPoints, knots, connectionTolerance), localToWorld);
}

}
}