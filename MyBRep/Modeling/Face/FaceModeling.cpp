#include "FaceModeling.h"

#include <cmath>
#include <limits>
#include <vector>

#include "MyMath/Vector2.h"
#include "MyBRep/Foundation/Diagnostic.h"
#include "MyBRep/Geometry/Curve/Geometry_BSpline.h"
#include "MyBRep/Geometry/Curve/Geometry_Bezier.h"
#include "MyBRep/Geometry/Curve/Geometry_Circle.h"
#include "MyBRep/Geometry/Curve2D/Geometry_BSpline2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Bezier2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Circle2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Curve2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Line2D.h"
#include "MyBRep/Geometry/Surface/Geometry_Plane.h"
#include "MyBRep/Topology/Topology_Builder.h"

namespace
{

const double DirectionToleranceScale = 64.0; // 判断圆曲线基方向是否位于Plane时覆盖单位向量舍入误差的固定倍数。

// 判断标量是否为有限非负数。
bool isFiniteNonNegative(double value)
{
    const double infinity = (std::numeric_limits<double>::infinity)();
    return value == value &&
           value != infinity &&
           value != -infinity &&
           value >= 0.0;
}

// 返回世界XY方向且原点位于世界原点的标准正交坐标系。
MyMath::CoordinateSystem worldCoordinateSystem()
{
    return MyMath::CoordinateSystem::fromAxes(
        MyMath::Vector3(0.0, 0.0, 0.0),
        MyMath::Vector3::unitX(),
        MyMath::Vector3::unitY(),
        MyMath::Vector3::unitZ());
}

// 返回三维点在指定Plane参数空间中的(U,V)坐标。
MyMath::Vector2 pointToUV(const MyBRep::Geometry_Plane& plane,
                          const MyMath::Vector3& point,
                          double tolerance)
{
    const MyMath::Vector3 relative =point - plane.origin();
    const double normalDistance =MyMath::Vector3::dot(relative,plane.normal());

    MYBREP_ASSERT_MESSAGE(point.isFinite() &&
                          std::fabs(normalDistance) <= tolerance,
                          "Planar Face modeling requires every projected point to lie on the Geometry_Plane within tolerance.");

    return MyMath::Vector2(
        MyMath::Vector3::dot(relative, plane.uDir()),
        MyMath::Vector3::dot(relative, plane.vDir()));
}

// 返回三维向量在指定Plane U/V基中的二维分量。
MyMath::Vector2 vectorToUV(const MyBRep::Geometry_Plane& plane,
                           const MyMath::Vector3& vector)
{
    return MyMath::Vector2(
        MyMath::Vector3::dot(vector, plane.uDir()),
        MyMath::Vector3::dot(vector, plane.vDir()));
}

// 返回用于向Topology_Builder写入标准TEdge几何表示的Forward Edge句柄。
MyBRep::Topology_Edge forwardEdge(
    const MyBRep::Topology_Edge& edge)
{
    MYBREP_ASSERT_MESSAGE(edge.isValid(),
                          "Planar Face modeling requires valid Topology_Edge values.");

    return edge.isForward()
               ? edge
               : edge.reversed();
}

// 从平面内三维Edge创建对应完整二维参数曲线，并返回二维自然参数区间。
MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D>
createPlanarCurve2D(const MyBRep::Topology_Edge& edge,
                    const MyBRep::Geometry_Plane& plane,
                    double tolerance,
                    double& firstParameter,
                    double& lastParameter)
{
    const MyBRep::Topology_Edge standardEdge =
        forwardEdge(edge);
    const MyBRep::Geometry_Curve& geometry =
        standardEdge.geometry();

    if (geometry.kind() == MyBRep::CurveKind::Line)
    {
        const MyMath::Vector2 startUV =
            pointToUV(plane,
                      standardEdge.startVertex().point(),
                      tolerance);
        const MyMath::Vector2 endUV =
            pointToUV(plane,
                      standardEdge.endVertex().point(),
                      tolerance);
        const MyMath::Vector2 direction =
            endUV - startUV;
        const double length = direction.length();

        MYBREP_ASSERT_MESSAGE(length > 0.0,
                              "Planar Line Edge must have different UV endpoints.");

        firstParameter = 0.0;
        lastParameter = length;

        return MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D>(
            new MyBRep::Geometry_Line2D(startUV, direction));
    }

    if (geometry.kind() == MyBRep::CurveKind::Circle)
    {
        const MyBRep::Geometry_Circle& circle =
            static_cast<const MyBRep::Geometry_Circle&>(geometry);
        const double directionTolerance =
            (std::numeric_limits<double>::epsilon)() *
            DirectionToleranceScale;

        const MyMath::Vector2 centerUV =
            pointToUV(plane,
                      circle.center(),
                      tolerance);

        MYBREP_ASSERT_MESSAGE(
            std::fabs(MyMath::Vector3::dot(circle.xAxis(),
                                           plane.normal())) <= directionTolerance &&
            std::fabs(MyMath::Vector3::dot(circle.yAxis(),
                                           plane.normal())) <= directionTolerance,
            "Planar Circle Edge basis must lie in the Geometry_Plane.");

        const MyMath::Vector2 xDirection =
            vectorToUV(plane, circle.xAxis());
        const MyMath::Vector2 yDirection =
            vectorToUV(plane, circle.yAxis());

        firstParameter =
            standardEdge.firstParameter();
        lastParameter =
            standardEdge.lastParameter();

        return MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D>(
            new MyBRep::Geometry_Circle2D(centerUV,
                                          circle.radius(),
                                          xDirection,
                                          yDirection));
    }

    if (geometry.kind() == MyBRep::CurveKind::Bezier)
    {
        const MyBRep::Geometry_Bezier& bezier =
            static_cast<const MyBRep::Geometry_Bezier&>(geometry);

        std::vector<MyMath::Vector2> controlPoints;
        controlPoints.reserve(bezier.controlPointCount());

        for (std::size_t index = 0;
             index < bezier.controlPointCount();
             ++index)
        {
            controlPoints.push_back(
                pointToUV(plane,
                          bezier.controlPoint(index),
                          tolerance));
        }

        firstParameter =
            standardEdge.firstParameter();
        lastParameter =
            standardEdge.lastParameter();

        return MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D>(
            new MyBRep::Geometry_Bezier2D(controlPoints));
    }

    if (geometry.kind() == MyBRep::CurveKind::BSpline)
    {
        const MyBRep::Geometry_BSpline& spline =
            static_cast<const MyBRep::Geometry_BSpline&>(geometry);

        std::vector<MyMath::Vector2> controlPoints;
        controlPoints.reserve(spline.controlPointCount());

        for (std::size_t index = 0;
             index < spline.controlPointCount();
             ++index)
        {
            controlPoints.push_back(
                pointToUV(plane,
                          spline.controlPoint(index),
                          tolerance));
        }

        firstParameter =
            standardEdge.firstParameter();
        lastParameter =
            standardEdge.lastParameter();

        return MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D>(
            new MyBRep::Geometry_BSpline2D(spline.degree(),
                                           controlPoints,
                                           spline.knots()));
    }

    MYBREP_ASSERT_MESSAGE(false,
                          "Planar Face modeling does not support the current Geometry_Curve kind.");

    return MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D>();
}

// 为全部Wire Edge建立当前Plane上的Curve-on-Surface表示。
void attachPlanarCurveRepresentations(
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>& surface,
    const MyBRep::Geometry_Plane& plane,
    const std::vector<MyBRep::Topology_Wire>& wires,
    double tolerance)
{
    for (std::size_t wireIndex = 0;
         wireIndex < wires.size();
         ++wireIndex)
    {
        MYBREP_ASSERT_MESSAGE(wires[wireIndex].isValid() &&
                              wires[wireIndex].isClosed(),
                              "Planar Face modeling requires valid closed trimming Wires.");

        for (std::size_t edgeIndex = 0;
             edgeIndex < wires[wireIndex].edgeCount();
             ++edgeIndex)
        {
            MyBRep::Topology_Edge edge =
                forwardEdge(wires[wireIndex].edge(edgeIndex));

            if (edge.hasCurveOnSurface(*surface))
            {
                continue;
            }

            double firstParameter = 0.0;
            double lastParameter = 0.0;

            const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> curve =
                createPlanarCurve2D(edge,
                                    plane,
                                    tolerance,
                                    firstParameter,
                                    lastParameter);

            MyBRep::Topology_Builder::addCurveOnSurface(
                edge,
                surface,
                curve,
                firstParameter,
                lastParameter,
                tolerance);
        }
    }
}

}

namespace MyBRep
{
namespace Modeling
{

/// 通用Topology_Face创建

Topology_Face createFace(
    const Foundation::RefPtr<const Geometry_Surface>& surface,
    const std::vector<Topology_Wire>& wires)
{
    MYBREP_ASSERT_MESSAGE(surface,
                          "Face modeling requires a non-null Geometry_Surface.");

    return Topology_Face(surface, wires);
}

Topology_Face createFace(
    const Foundation::RefPtr<const Geometry_Surface>& surface)
{
    MYBREP_ASSERT_MESSAGE(surface,
                          "Face modeling requires a non-null Geometry_Surface.");

    return Topology_Face(surface);
}

/// 平面Topology_Face创建

Topology_Face createPlanarFace(
    const std::vector<Topology_Wire>& wires,
    double tolerance)
{
    return createPlanarFace(worldCoordinateSystem(),
                            wires,
                            tolerance);
}

Topology_Face createPlanarFace(
    const MyMath::CoordinateSystem& coordinateSystem,
    const std::vector<Topology_Wire>& wires,
    double tolerance)
{
    MYBREP_ASSERT_MESSAGE(coordinateSystem.isValid(),
                          "Planar Face modeling requires a valid coordinate system.");
    MYBREP_ASSERT_MESSAGE(isFiniteNonNegative(tolerance),
                          "Planar Face modeling tolerance must be finite and non-negative.");

    const Foundation::RefPtr<const Geometry_Surface> surface(
        new Geometry_Plane(coordinateSystem));

    const Geometry_Plane& plane =
        static_cast<const Geometry_Plane&>(*surface);

    attachPlanarCurveRepresentations(surface,
                                     plane,
                                     wires,
                                     tolerance);

    return Topology_Face(surface, wires);
}

Topology_Face createPlanarFace(
    const Topology_Wire& wire,
    double tolerance)
{
    std::vector<Topology_Wire> wires;
    wires.push_back(wire);

    return createPlanarFace(wires, tolerance);
}

Topology_Face createPlanarFace(
    const MyMath::CoordinateSystem& coordinateSystem,
    const Topology_Wire& wire,
    double tolerance)
{
    std::vector<Topology_Wire> wires;
    wires.push_back(wire);

    return createPlanarFace(coordinateSystem,
                            wires,
                            tolerance);
}

/// 空间Face实例创建

Face makeFace(
    const Foundation::RefPtr<const Geometry_Surface>& surface,
    const std::vector<Topology_Wire>& wires)
{
    return Face(createFace(surface, wires));
}

Face makeFace(
    const Foundation::RefPtr<const Geometry_Surface>& surface,
    const std::vector<Topology_Wire>& wires,
    const MyMath::Matrix4& localToWorld)
{
    return Face(createFace(surface, wires),
                localToWorld);
}

Face makePlanarFace(
    const std::vector<Topology_Wire>& wires,
    double tolerance)
{
    return Face(createPlanarFace(wires,
                                 tolerance));
}

Face makePlanarFace(
    const std::vector<Topology_Wire>& wires,
    const MyMath::Matrix4& localToWorld,
    double tolerance)
{
    return Face(createPlanarFace(wires,
                                 tolerance),
                localToWorld);
}

Face makePlanarFace(
    const MyMath::CoordinateSystem& coordinateSystem,
    const std::vector<Topology_Wire>& wires,
    double tolerance)
{
    return Face(createPlanarFace(coordinateSystem,
                                 wires,
                                 tolerance));
}

Face makePlanarFace(
    const MyMath::CoordinateSystem& coordinateSystem,
    const std::vector<Topology_Wire>& wires,
    const MyMath::Matrix4& localToWorld,
    double tolerance)
{
    return Face(createPlanarFace(coordinateSystem,
                                 wires,
                                 tolerance),
                localToWorld);
}

}
}
