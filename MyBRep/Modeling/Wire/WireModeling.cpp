#include "WireModeling.h"

#include <cmath>
#include <limits>

#include "MyBRep/Foundation/Diagnostic.h"
#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Curve/Geometry_Curve.h"
#include "MyBRep/Geometry/Curve/Geometry_Line.h"
#include "MyBRep/Modeling/Edge/EdgeModeling.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"

namespace
{

const double HalfScale = 0.5; // 完整矩形尺寸转换为半尺寸使用的固定比例。
const double Pi = 3.1415926535897932384626433832795; // 完整圆建模统一使用弧度制。
const double TwoPi = Pi * 2.0; // 单Edge闭合圆使用的完整扫掠角。

// 判断标量是否为有限值。
bool isFiniteValue(double value)
{
    const double infinity = (std::numeric_limits<double>::infinity)();
    return value == value &&
           value != infinity &&
           value != -infinity;
}

// 判断指定点是否为严格局部XY平面有限点。
bool isFinitePlanarPoint(const MyMath::Vector3& point)
{
    return point.isFinite() && point.z() == 0.0;
}

// 判断指定点序列是否满足开放折线或闭合多边形建模前置条件。
bool isValidPointSequence(const std::vector<MyMath::Vector3>& points,
                          bool closed)
{
    if (points.size() < (closed ? 3U : 2U))
    {
        return false;
    }

    for (std::size_t index = 0;
         index < points.size();
         ++index)
    {
        if (!isFinitePlanarPoint(points[index]))
        {
            return false;
        }

        if (index > 0 &&
            points[index].isEqualTo(points[index - 1], 0.0))
        {
            return false;
        }
    }

    return !closed ||
           !points.front().isEqualTo(points.back(), 0.0);
}

// 使用已经确定的共享拓扑顶点创建直线Topology_Edge。
MyBRep::Topology_Edge createLineEdge(
    const MyBRep::Topology_Vertex& startVertex,
    const MyBRep::Topology_Vertex& endVertex)
{
    const MyMath::Vector3 direction =
        endVertex.point() - startVertex.point();
    const double length = direction.length();

    MYBREP_ASSERT_MESSAGE(length > 0.0,
                          "Wire modeling line Edge requires different shared vertices.");

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> geometry(
        new MyBRep::Geometry_Line(startVertex.point(), direction));

    return MyBRep::Topology_Edge(startVertex,
                                 endVertex,
                                 geometry,
                                 0.0,
                                 length,
                                 0.0);
}

}

namespace MyBRep
{
namespace Modeling
{

/// 局部Topology_Wire创建

Topology_Wire createWire(const std::vector<Topology_Edge>& edges)
{
    return Topology_Wire(edges);
}

Topology_Wire createPolyline(const std::vector<MyMath::Vector3>& points)
{
    MYBREP_ASSERT_MESSAGE(isValidPointSequence(points, false),
                          "Polyline modeling requires at least two finite XY-plane points without repeated adjacent vertices.");

    std::vector<Topology_Vertex> vertices;
    vertices.reserve(points.size());

    for (std::size_t index = 0;
         index < points.size();
         ++index)
    {
        vertices.push_back(Topology_Vertex(points[index]));
    }

    std::vector<Topology_Edge> edges;
    edges.reserve(vertices.size() - 1);

    for (std::size_t index = 0;
         index + 1 < vertices.size();
         ++index)
    {
        edges.push_back(
            createLineEdge(vertices[index],
                           vertices[index + 1]));
    }

    return createWire(edges);
}

Topology_Wire createPolygon(const std::vector<MyMath::Vector3>& points)
{
    MYBREP_ASSERT_MESSAGE(isValidPointSequence(points, true),
                          "Polygon modeling requires at least three finite XY-plane vertices and must not repeat the first vertex at the end.");

    std::vector<Topology_Vertex> vertices;
    vertices.reserve(points.size());

    for (std::size_t index = 0;
         index < points.size();
         ++index)
    {
        vertices.push_back(Topology_Vertex(points[index]));
    }

    std::vector<Topology_Edge> edges;
    edges.reserve(vertices.size());

    for (std::size_t index = 0;
         index + 1 < vertices.size();
         ++index)
    {
        edges.push_back(
            createLineEdge(vertices[index],
                           vertices[index + 1]));
    }

    edges.push_back(
        createLineEdge(vertices.back(),
                       vertices.front()));

    return createWire(edges);
}

Topology_Wire createRectangle(double sizeX,
                              double sizeY)
{
    return createRectangle(MyMath::Vector3(0.0, 0.0, 0.0),
                           sizeX,
                           sizeY);
}

Topology_Wire createRectangle(const MyMath::Vector3& center,
                              double sizeX,
                              double sizeY)
{
    MYBREP_ASSERT_MESSAGE(isFinitePlanarPoint(center),
                          "Rectangle modeling center must be a finite XY-plane point.");
    MYBREP_ASSERT_MESSAGE(isFiniteValue(sizeX) &&
                          isFiniteValue(sizeY) &&
                          sizeX > 0.0 &&
                          sizeY > 0.0,
                          "Rectangle modeling sizes must be finite and positive.");

    const double halfX = sizeX * HalfScale;
    const double halfY = sizeY * HalfScale;

    std::vector<MyMath::Vector3> points;
    points.reserve(4); // 标准矩形固定由四个逆时针顶点定义。

    points.push_back(
        MyMath::Vector3(center.x() - halfX,
                        center.y() - halfY,
                        0.0));
    points.push_back(
        MyMath::Vector3(center.x() + halfX,
                        center.y() - halfY,
                        0.0));
    points.push_back(
        MyMath::Vector3(center.x() + halfX,
                        center.y() + halfY,
                        0.0));
    points.push_back(
        MyMath::Vector3(center.x() - halfX,
                        center.y() + halfY,
                        0.0));

    return createPolygon(points);
}

Topology_Wire createCircle(double radius)
{
    return createCircle(MyMath::Vector3(0.0, 0.0, 0.0),
                        radius);
}

Topology_Wire createCircle(const MyMath::Vector3& center,
                           double radius)
{
    MYBREP_ASSERT_MESSAGE(isFinitePlanarPoint(center),
                          "Circle modeling center must be a finite XY-plane point.");
    MYBREP_ASSERT_MESSAGE(isFiniteValue(radius) &&
                          radius > 0.0,
                          "Circle modeling radius must be finite and positive.");

    std::vector<Topology_Edge> edges;
    edges.reserve(1); // 完整圆固定由一条共享起终Vertex的2*pi逆时针Edge表示。
    edges.push_back(
        createArc(center,
                  radius,
                  0.0,
                  TwoPi));

    return createWire(edges);
}

/// 空间Wire实例创建

Wire makeWire(const std::vector<Topology_Edge>& edges)
{
    return Wire(createWire(edges));
}

Wire makeWire(const std::vector<Topology_Edge>& edges,
              const MyMath::Matrix4& localToWorld)
{
    return Wire(createWire(edges), localToWorld);
}

Wire makePolyline(const std::vector<MyMath::Vector3>& points)
{
    return Wire(createPolyline(points));
}

Wire makePolyline(const std::vector<MyMath::Vector3>& points,
                  const MyMath::Matrix4& localToWorld)
{
    return Wire(createPolyline(points), localToWorld);
}

Wire makePolygon(const std::vector<MyMath::Vector3>& points)
{
    return Wire(createPolygon(points));
}

Wire makePolygon(const std::vector<MyMath::Vector3>& points,
                 const MyMath::Matrix4& localToWorld)
{
    return Wire(createPolygon(points), localToWorld);
}

Wire makeRectangle(double sizeX,
                   double sizeY)
{
    return Wire(createRectangle(sizeX, sizeY));
}

Wire makeRectangle(double sizeX,
                   double sizeY,
                   const MyMath::Matrix4& localToWorld)
{
    return Wire(createRectangle(sizeX, sizeY),
                localToWorld);
}

Wire makeRectangle(const MyMath::Vector3& center,
                   double sizeX,
                   double sizeY)
{
    return Wire(createRectangle(center,
                                sizeX,
                                sizeY));
}

Wire makeRectangle(const MyMath::Vector3& center,
                   double sizeX,
                   double sizeY,
                   const MyMath::Matrix4& localToWorld)
{
    return Wire(createRectangle(center,
                                sizeX,
                                sizeY),
                localToWorld);
}

Wire makeCircle(double radius)
{
    return Wire(createCircle(radius));
}

Wire makeCircle(double radius,
                const MyMath::Matrix4& localToWorld)
{
    return Wire(createCircle(radius),
                localToWorld);
}

Wire makeCircle(const MyMath::Vector3& center,
                double radius)
{
    return Wire(createCircle(center, radius));
}

Wire makeCircle(const MyMath::Vector3& center,
                double radius,
                const MyMath::Matrix4& localToWorld)
{
    return Wire(createCircle(center, radius),
                localToWorld);
}

}
}
