#include "WireModeling.h"

#include "MyMath/MathUtils.h"
#include "MyBRep/Foundation/Diagnostic.h"
#include "MyBRep/Modeling/Edge/EdgeModeling.h"

namespace MyBRep
{
namespace Modeling
{

namespace WireModelingDetail
{

bool isFinitePlanarPoint(const MyMath::Vector3& point)
{
    return point.isFinite() && point.z() == 0.0;
}

bool isValidPointSequence(const std::vector<MyMath::Vector3>& points,bool closed)
{
    if (points.size() < (closed ? 3U : 2U)) return false;

    for (std::size_t index = 0; index < points.size(); ++index)
    {
        if (!points[index].isFinite()) return false;
        if (index > 0 && points[index].isEqualTo(points[index - 1], 0.0)) return false;
    }

    return !closed || !points.front().isEqualTo(points.back(), 0.0);
}

bool isValidVertexSequence(const std::vector<Topology_Vertex>& vertices,bool closed)
{
    if (vertices.size() < (closed ? 3U : 2U)) return false;

    for (std::size_t index = 0; index < vertices.size(); ++index)
    {
        if (!vertices[index].isValid() || !vertices[index].point().isFinite()) return false;
        if (index > 0 && vertices[index].point().isEqualTo(vertices[index - 1].point(), 0.0)) return false;
    }

    return !closed || !vertices.front().point().isEqualTo(vertices.back().point(), 0.0);
}

}

/// 局部Topology_Wire创建

Topology_Wire createWire(const std::vector<Topology_Edge>& edges)
{
    return Topology_Wire(edges);
}

Topology_Wire createPolyline(const std::vector<MyMath::Vector3>& points)
{
    MYBREP_ASSERT_MESSAGE(WireModelingDetail::isValidPointSequence(points, false),"Polyline modeling requires at least two finite three-dimensional points without repeated adjacent vertices.");

    std::vector<Topology_Vertex> vertices;
    vertices.reserve(points.size());

    for (std::size_t index = 0; index < points.size(); ++index)
    {
        vertices.push_back(Topology_Vertex(points[index]));
    }

    return createPolyline(vertices);
}

Topology_Wire createPolyline(const std::vector<Topology_Vertex>& vertices)
{
    MYBREP_ASSERT_MESSAGE(WireModelingDetail::isValidVertexSequence(vertices, false),"Polyline modeling requires at least two valid finite Topology_Vertex values without repeated adjacent positions.");

    std::vector<Topology_Edge> edges;
    edges.reserve(vertices.size() - 1);

    for (std::size_t index = 0; index + 1 < vertices.size(); ++index)
    {
        edges.push_back(createLine(vertices[index], vertices[index + 1]));
    }

    return createWire(edges);
}

Topology_Wire createPolygon(const std::vector<MyMath::Vector3>& points)
{
    MYBREP_ASSERT_MESSAGE(WireModelingDetail::isValidPointSequence(points, true),
                          "Polygon modeling requires at least three finite three-dimensional vertices and must not repeat the first vertex at the end.");

    std::vector<Topology_Vertex> vertices;
    vertices.reserve(points.size());

    for (std::size_t index = 0; index < points.size(); ++index)
    {
        vertices.push_back(Topology_Vertex(points[index]));
    }

    return createPolygon(vertices);
}

Topology_Wire createPolygon(const std::vector<Topology_Vertex>& vertices)
{
    MYBREP_ASSERT_MESSAGE(WireModelingDetail::isValidVertexSequence(vertices, true),
                          "Polygon modeling requires at least three valid finite Topology_Vertex values and must not repeat the first position at the end.");

    std::vector<Topology_Edge> edges;
    edges.reserve(vertices.size());

    for (std::size_t index = 0; index + 1 < vertices.size(); ++index)
    {
        edges.push_back(createLine(vertices[index], vertices[index + 1]));
    }

    edges.push_back(createLine(vertices.back(), vertices.front()));
    return createWire(edges);
}

Topology_Wire createRectangle(double sizeX,double sizeY)
{
    return createRectangle(MyMath::Vector3::zero(), sizeX, sizeY);
}

Topology_Wire createRectangle(const MyMath::Vector3& center,double sizeX,double sizeY)
{
    MYBREP_ASSERT_MESSAGE(WireModelingDetail::isFinitePlanarPoint(center),
                          "Rectangle modeling center must be a finite XY-plane point.");
    MYBREP_ASSERT_MESSAGE(MyMath::isFinite(sizeX) && MyMath::isFinite(sizeY) && sizeX > 0.0 && sizeY > 0.0,
                          "Rectangle modeling sizes must be finite and positive.");

    const double halfX = sizeX * 0.5;
    const double halfY = sizeY * 0.5;

    std::vector<MyMath::Vector3> points;
    points.reserve(4);
    points.push_back(MyMath::Vector3(center.x() - halfX, center.y() - halfY, 0.0));
    points.push_back(MyMath::Vector3(center.x() + halfX, center.y() - halfY, 0.0));
    points.push_back(MyMath::Vector3(center.x() + halfX, center.y() + halfY, 0.0));
    points.push_back(MyMath::Vector3(center.x() - halfX, center.y() + halfY, 0.0));

    return createPolygon(points);
}

Topology_Wire createCircle(double radius)
{
    return createCircle(MyMath::Vector3::zero(), radius);
}

Topology_Wire createCircle(const MyMath::Vector3& center,double radius)
{
    MYBREP_ASSERT_MESSAGE(WireModelingDetail::isFinitePlanarPoint(center),
                          "Circle modeling center must be a finite XY-plane point.");
    MYBREP_ASSERT_MESSAGE(MyMath::isFinite(radius) && radius > 0.0,
                          "Circle modeling radius must be finite and positive.");

    std::vector<Topology_Edge> edges;
    edges.reserve(1);
    edges.push_back(createArc(center, radius, 0.0, MyMath::TwoPi));
    return createWire(edges);
}

/// 空间Wire实例创建

Wire makeWire(const std::vector<Topology_Edge>& edges)
{
    return Wire(createWire(edges));
}

Wire makeWire(const std::vector<Topology_Edge>& edges,const MyMath::Matrix4& localToWorld)
{
    return Wire(createWire(edges), localToWorld);
}

Wire makePolyline(const std::vector<MyMath::Vector3>& points)
{
    return Wire(createPolyline(points));
}

Wire makePolyline(const std::vector<MyMath::Vector3>& points,const MyMath::Matrix4& localToWorld)
{
    return Wire(createPolyline(points), localToWorld);
}

Wire makePolyline(const std::vector<Topology_Vertex>& vertices)
{
    return Wire(createPolyline(vertices));
}

Wire makePolyline(const std::vector<Topology_Vertex>& vertices,const MyMath::Matrix4& localToWorld)
{
    return Wire(createPolyline(vertices), localToWorld);
}

Wire makePolygon(const std::vector<MyMath::Vector3>& points)
{
    return Wire(createPolygon(points));
}

Wire makePolygon(const std::vector<MyMath::Vector3>& points,const MyMath::Matrix4& localToWorld)
{
    return Wire(createPolygon(points), localToWorld);
}

Wire makePolygon(const std::vector<Topology_Vertex>& vertices)
{
    return Wire(createPolygon(vertices));
}

Wire makePolygon(const std::vector<Topology_Vertex>& vertices,const MyMath::Matrix4& localToWorld)
{
    return Wire(createPolygon(vertices), localToWorld);
}

Wire makeRectangle(double sizeX,double sizeY)
{
    return Wire(createRectangle(sizeX, sizeY));
}

Wire makeRectangle(double sizeX,double sizeY,const MyMath::Matrix4& localToWorld)
{
    return Wire(createRectangle(sizeX, sizeY), localToWorld);
}

Wire makeRectangle(const MyMath::Vector3& center,double sizeX,double sizeY)
{
    return Wire(createRectangle(center, sizeX, sizeY));
}

Wire makeRectangle(const MyMath::Vector3& center,double sizeX,double sizeY,const MyMath::Matrix4& localToWorld)
{
    return Wire(createRectangle(center, sizeX, sizeY), localToWorld);
}

Wire makeCircle(double radius)
{
    return Wire(createCircle(radius));
}

Wire makeCircle(double radius,const MyMath::Matrix4& localToWorld)
{
    return Wire(createCircle(radius), localToWorld);
}

Wire makeCircle(const MyMath::Vector3& center,double radius)
{
    return Wire(createCircle(center, radius));
}

Wire makeCircle(const MyMath::Vector3& center,double radius,const MyMath::Matrix4& localToWorld)
{
    return Wire(createCircle(center, radius), localToWorld);
}

}
}