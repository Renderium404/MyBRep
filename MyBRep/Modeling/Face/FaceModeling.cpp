#include "FaceModeling.h"

#include <cmath>
#include <limits>
#include <vector>

#include "MyMath/MathUtils.h"
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
#include "MyBRep/Geometry/Surface/Geometry_PlaneSurface.h"
#include "MyBRep/Topology/Topology_Builder.h"

namespace MyBRep
{
namespace Modeling
{
namespace FaceModelingDetail
{

// 返回三维点在指定Plane参数空间中的(U,V)坐标。
MyMath::Vector2 pointToUV(const MyBRep::Geometry_PlaneSurface& plane,
                          const MyMath::Vector3& point,
                          double tolerance)
{
    const MyMath::Vector3 relative =point - plane.origin();
    const double normalDistance =MyMath::Vector3::dot(relative,plane.normal());

    MYBREP_ASSERT_MESSAGE(point.isFinite() &&
                          std::fabs(normalDistance) <= tolerance,
                          "Planar Face modeling requires every projected point to lie on the Geometry_PlaneSurface within tolerance.");

    return MyMath::Vector2(
        MyMath::Vector3::dot(relative, plane.uDir()),
        MyMath::Vector3::dot(relative, plane.vDir()));
}

// 返回三维向量在指定Plane U/V基中的二维分量。
MyMath::Vector2 vectorToUV(const MyBRep::Geometry_PlaneSurface& plane,
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
                    const MyBRep::Geometry_PlaneSurface& plane,
                    double tolerance,
                    double& firstParameter,
                    double& lastParameter)
{
    const MyBRep::Topology_Edge standardEdge =forwardEdge(edge);
    const MyBRep::Geometry_Curve& geometry =standardEdge.geometry();

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
        const double directionTolerance = (std::numeric_limits<double>::epsilon)() * 64.0;

        const MyMath::Vector2 centerUV =
            pointToUV(plane,
                      circle.center(),
                      tolerance);

        MYBREP_ASSERT_MESSAGE(
            std::fabs(MyMath::Vector3::dot(circle.xDir(),
                                           plane.normal())) <= directionTolerance &&
            std::fabs(MyMath::Vector3::dot(circle.yDir(),
                                           plane.normal())) <= directionTolerance,
            "Planar Circle Edge basis must lie in the Geometry_PlaneSurface.");

        const MyMath::Vector2 xDirection =
            vectorToUV(plane, circle.xDir());
        const MyMath::Vector2 yDirection =
            vectorToUV(plane, circle.yDir());

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
    const MyBRep::Geometry_PlaneSurface& plane,
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
    return createPlanarFace(MyMath::CoordinateSystem::identity(),
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
    MYBREP_ASSERT_MESSAGE(MyMath::isFinite(tolerance) && tolerance >= 0.0,
                          "Planar Face modeling tolerance must be finite and non-negative.");

    const Foundation::RefPtr<const Geometry_Surface> surface(
        new Geometry_PlaneSurface(coordinateSystem));

    const Geometry_PlaneSurface& plane =
        static_cast<const Geometry_PlaneSurface&>(*surface);

    FaceModelingDetail::attachPlanarCurveRepresentations(surface,
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

Face makeFace(const Foundation::RefPtr<const Geometry_Surface>& surface)
{
    return Face(createFace(surface));
}

Face makeFace(const Foundation::RefPtr<const Geometry_Surface>& surface,const MyMath::Matrix4& localToWorld)
{
    return Face(createFace(surface), localToWorld);
}

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
    return Face(createPlanarFace(coordinateSystem, wires, tolerance), localToWorld);
}

Face makePlanarFace(const Topology_Wire& wire,double tolerance)
{
    return Face(createPlanarFace(wire, tolerance));
}

Face makePlanarFace(const Topology_Wire& wire,const MyMath::Matrix4& localToWorld,double tolerance)
{
    return Face(createPlanarFace(wire, tolerance), localToWorld);
}

Face makePlanarFace(
    const MyMath::CoordinateSystem& coordinateSystem,
    const Topology_Wire& wire,
    double tolerance)
{
    return Face(createPlanarFace(coordinateSystem, wire, tolerance));
}

Face makePlanarFace(
    const MyMath::CoordinateSystem& coordinateSystem,
    const Topology_Wire& wire,
    const MyMath::Matrix4& localToWorld,
    double tolerance)
{
    return Face(createPlanarFace(coordinateSystem, wire, tolerance), localToWorld);
}

}
}
