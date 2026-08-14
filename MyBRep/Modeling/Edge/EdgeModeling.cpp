#include "EdgeModeling.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "MyBRep/Foundation/Diagnostic.h"
#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Curve/Geometry_Circle.h"
#include "MyBRep/Geometry/Curve/Geometry_Curve.h"
#include "MyBRep/Geometry/Curve/Geometry_Line.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 圆弧建模统一使用弧度制。
const double TwoPi = Pi * 2.0; // 单条圆弧Edge允许覆盖的最大完整周期。
const double CircleClosureToleranceScale = 64.0; // 覆盖完整圆端点三角函数舍入误差的双精度容差倍数。

// 判断标量是否为有限值。
bool isFiniteValue(double value)
{
    const double infinity = (std::numeric_limits<double>::infinity)();
    return value == value &&
           value != infinity &&
           value != -infinity;
}

// 判断扫掠是否在数值意义上表示完整圆。
bool isFullCircleSweep(double sweepAngle)
{
    return std::fabs(std::fabs(sweepAngle) - TwoPi) <=
           (std::numeric_limits<double>::epsilon)() *
               CircleClosureToleranceScale;
}

// 根据完整圆坐标尺度返回只用于拓扑端点与曲线端点一致性检查的数值容差。
double circleClosureTolerance(const MyMath::Vector3& center,
                              double radius)
{
    double scale = 1.0;
    scale = (std::max)(scale, std::fabs(center.x()));
    scale = (std::max)(scale, std::fabs(center.y()));
    scale = (std::max)(scale, std::fabs(center.z()));
    scale = (std::max)(scale, std::fabs(radius));

    return scale *
           (std::numeric_limits<double>::epsilon)() *
           CircleClosureToleranceScale;
}

// 使用完整圆几何及有向扫掠区间创建有限Topology_Edge。
MyBRep::Topology_Edge createCircleEdge(
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve>& geometry,
    double startAngle,
    double sweepAngle,
    double endpointTolerance)
{
    const double endAngle = startAngle + sweepAngle;
    const bool closed = isFullCircleSweep(sweepAngle);

    if (sweepAngle > 0.0)
    {
        const MyMath::Vector3 startPoint = geometry->pointAt(startAngle);
        const MyBRep::Topology_Vertex startVertex(startPoint);

        if (closed)
        {
            return MyBRep::Topology_Edge(startVertex,
                                         startVertex,
                                         geometry,
                                         startAngle,
                                         endAngle,
                                         endpointTolerance);
        }

        const MyBRep::Topology_Vertex endVertex(
            geometry->pointAt(endAngle));

        return MyBRep::Topology_Edge(startVertex,
                                     endVertex,
                                     geometry,
                                     startAngle,
                                     endAngle,
                                     endpointTolerance);
    }

    const MyMath::Vector3 startPoint = geometry->pointAt(startAngle);
    const MyBRep::Topology_Vertex traversalStartVertex(startPoint);

    if (closed)
    {
        const MyBRep::Topology_Edge forwardEdge(traversalStartVertex,
                                                traversalStartVertex,
                                                geometry,
                                                endAngle,
                                                startAngle,
                                                endpointTolerance);

        return forwardEdge.reversed();
    }

    const MyBRep::Topology_Vertex traversalEndVertex(
        geometry->pointAt(endAngle));

    const MyBRep::Topology_Edge forwardEdge(traversalEndVertex,
                                            traversalStartVertex,
                                            geometry,
                                            endAngle,
                                            startAngle,
                                            endpointTolerance);

    return forwardEdge.reversed();
}

}

namespace MyBRep
{
namespace Modeling
{

/// 局部Topology_Edge创建

Topology_Edge createLine(const MyMath::Vector3& startPoint,
                         const MyMath::Vector3& endPoint)
{
    MYBREP_ASSERT_MESSAGE(startPoint.isFinite() &&
                          endPoint.isFinite() &&
                          !startPoint.isEqualTo(endPoint, 0.0),
                          "Line modeling requires two different finite points.");

    const MyMath::Vector3 direction = endPoint - startPoint;
    const double length = direction.length();

    const Foundation::RefPtr<const Geometry_Curve> geometry(
        new Geometry_Line(startPoint, direction));

    const Topology_Vertex startVertex(startPoint);
    const Topology_Vertex endVertex(endPoint);

    return Topology_Edge(startVertex,
                         endVertex,
                         geometry,
                         0.0,
                         length,
                         0.0);
}

Topology_Edge createArc(const MyMath::Vector3& center,
                        double radius,
                        double startAngle,
                        double sweepAngle)
{
    MYBREP_ASSERT_MESSAGE(center.isFinite(),
                          "Arc modeling center must be finite.");
    MYBREP_ASSERT_MESSAGE(isFiniteValue(radius) && radius > 0.0,
                          "Arc modeling radius must be finite and positive.");
    MYBREP_ASSERT_MESSAGE(isFiniteValue(startAngle),
                          "Arc modeling start angle must be finite.");
    MYBREP_ASSERT_MESSAGE(isFiniteValue(sweepAngle) &&
                          sweepAngle != 0.0 &&
                          std::fabs(sweepAngle) <= TwoPi,
                          "Arc modeling sweep must be finite, non-zero and within one complete period.");

    const Foundation::RefPtr<const Geometry_Curve> geometry(
        new Geometry_Circle(center, radius));

    const double tolerance =
        isFullCircleSweep(sweepAngle)
            ? circleClosureTolerance(center, radius)
            : 0.0;

    return createCircleEdge(geometry,
                            startAngle,
                            sweepAngle,
                            tolerance);
}

Topology_Edge createArc(const MyMath::CoordinateSystem& coordinateSystem,
                        double radius,
                        double startAngle,
                        double sweepAngle)
{
    MYBREP_ASSERT_MESSAGE(coordinateSystem.isValid(),
                          "Arc modeling coordinate system must be valid.");
    MYBREP_ASSERT_MESSAGE(isFiniteValue(radius) && radius > 0.0,
                          "Arc modeling radius must be finite and positive.");
    MYBREP_ASSERT_MESSAGE(isFiniteValue(startAngle),
                          "Arc modeling start angle must be finite.");
    MYBREP_ASSERT_MESSAGE(isFiniteValue(sweepAngle) &&
                          sweepAngle != 0.0 &&
                          std::fabs(sweepAngle) <= TwoPi,
                          "Arc modeling sweep must be finite, non-zero and within one complete period.");

    const Foundation::RefPtr<const Geometry_Curve> geometry(
        new Geometry_Circle(coordinateSystem, radius));

    const double tolerance =
        isFullCircleSweep(sweepAngle)
            ? circleClosureTolerance(coordinateSystem.origin(), radius)
            : 0.0;

    return createCircleEdge(geometry,
                            startAngle,
                            sweepAngle,
                            tolerance);
}

/// 空间Edge实例创建

Edge makeLine(const MyMath::Vector3& startPoint,
              const MyMath::Vector3& endPoint)
{
    return Edge(createLine(startPoint, endPoint));
}

Edge makeLine(const MyMath::Vector3& startPoint,
              const MyMath::Vector3& endPoint,
              const MyMath::Matrix4& localToWorld)
{
    return Edge(createLine(startPoint, endPoint),
                localToWorld);
}

Edge makeArc(const MyMath::Vector3& center,
             double radius,
             double startAngle,
             double sweepAngle)
{
    return Edge(createArc(center,
                          radius,
                          startAngle,
                          sweepAngle));
}

Edge makeArc(const MyMath::Vector3& center,
             double radius,
             double startAngle,
             double sweepAngle,
             const MyMath::Matrix4& localToWorld)
{
    return Edge(createArc(center,
                          radius,
                          startAngle,
                          sweepAngle),
                localToWorld);
}

Edge makeArc(const MyMath::CoordinateSystem& coordinateSystem,
             double radius,
             double startAngle,
             double sweepAngle)
{
    return Edge(createArc(coordinateSystem,
                          radius,
                          startAngle,
                          sweepAngle));
}

Edge makeArc(const MyMath::CoordinateSystem& coordinateSystem,
             double radius,
             double startAngle,
             double sweepAngle,
             const MyMath::Matrix4& localToWorld)
{
    return Edge(createArc(coordinateSystem,
                          radius,
                          startAngle,
                          sweepAngle),
                localToWorld);
}

}
}
