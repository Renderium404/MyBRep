#ifndef MYBREP_SANDBOX_FREEFORMSURFACETESTFIXTURES_H
#define MYBREP_SANDBOX_FREEFORMSURFACETESTFIXTURES_H

#include <vector>

#include "MyMath/Vector2.h"
#include "MyMath/Vector3.h"

#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Curve/Geometry_Curve.h"
#include "MyBRep/Geometry/Curve/Geometry_Line.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Curve2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Line2D.h"
#include "MyBRep/Geometry/Surface/Geometry_BSplineSurface.h"
#include "MyBRep/Geometry/Surface/Geometry_BezierSurface.h"
#include "MyBRep/Geometry/Surface/Geometry_Surface.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Topology/Topology_Builder.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Face/Topology_Face.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

namespace FreeformSurfaceTestFixtures
{

const double TestTolerance = 1.0e-8; // 自由曲面专项测试统一使用的Topology与Curve-on-Surface连接容差。

inline void addLineCurveOnSurface(
    MyBRep::Topology_Edge& edge,
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>& surface,
    const MyMath::Vector2& firstUV,
    const MyMath::Vector2& lastUV)
{
    const MyMath::Vector2 direction = lastUV - firstUV;
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> curve(
        new MyBRep::Geometry_Line2D(firstUV, direction));

    MyBRep::Topology_Builder::addCurveOnSurface(
        edge,
        surface,
        curve,
        0.0,
        direction.length(),
        TestTolerance);
}

inline std::vector<MyMath::Vector3> createBezierControlPoints()
{
    const double coordinates[4] =
    {
        0.0,
        4.0 / 3.0,
        8.0 / 3.0,
        4.0
    };

    const double heights[4][4] =
    {
        {0.0, 0.0, 0.0, 0.0},
        {0.0, 1.4, 2.2, 0.0},
        {0.0, 2.0, 1.1, 0.0},
        {0.0, 0.0, 0.0, 0.0}
    };

    std::vector<MyMath::Vector3> points;
    points.reserve(16);

    for (std::size_t vIndex = 0; vIndex < 4; ++vIndex)
    {
        for (std::size_t uIndex = 0; uIndex < 4; ++uIndex)
        {
            points.push_back(
                MyMath::Vector3(
                    coordinates[uIndex],
                    coordinates[vIndex],
                    heights[vIndex][uIndex]));
        }
    }

    return points;
}

inline MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> createBezierSurface()
{
    return MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>(
        new MyBRep::Geometry_BezierSurface(
            4,
            4,
            createBezierControlPoints()));
}

inline std::vector<MyMath::Vector3> createBSplineControlPoints()
{
    // 二次开区间B-Spline使用Greville参数位置[0, 0.25, 0.75, 1]映射到[0,4]，
    // 因而四条边在三维空间中仍严格表示x=4u或y=4v的直线。
    const double coordinates[4] =
    {
        0.0,
        1.0,
        3.0,
        4.0
    };

    const double heights[4][4] =
    {
        {0.0, 0.0, 0.0, 0.0},
        {0.0, 1.2, 2.1, 0.0},
        {0.0, 2.3, 1.0, 0.0},
        {0.0, 0.0, 0.0, 0.0}
    };

    std::vector<MyMath::Vector3> points;
    points.reserve(16);

    for (std::size_t vIndex = 0; vIndex < 4; ++vIndex)
    {
        for (std::size_t uIndex = 0; uIndex < 4; ++uIndex)
        {
            points.push_back(
                MyMath::Vector3(
                    coordinates[uIndex],
                    coordinates[vIndex],
                    heights[vIndex][uIndex]));
        }
    }

    return points;
}

inline std::vector<double> createQuadraticTwoSpanKnots()
{
    std::vector<double> knots;
    knots.push_back(0.0);
    knots.push_back(0.0);
    knots.push_back(0.0);
    knots.push_back(0.5);
    knots.push_back(1.0);
    knots.push_back(1.0);
    knots.push_back(1.0);
    return knots;
}

inline MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> createBSplineSurface()
{
    return MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>(
        new MyBRep::Geometry_BSplineSurface(
            2,
            2,
            4,
            4,
            createBSplineControlPoints(),
            createQuadraticTwoSpanKnots(),
            createQuadraticTwoSpanKnots()));
}

inline MyBRep::Topology_Face createFullDomainFace(
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>& surface)
{
    const MyMath::Vector2 uv00(surface->uDomainStart(), surface->vDomainStart());
    const MyMath::Vector2 uv10(surface->uDomainEnd(), surface->vDomainStart());
    const MyMath::Vector2 uv11(surface->uDomainEnd(), surface->vDomainEnd());
    const MyMath::Vector2 uv01(surface->uDomainStart(), surface->vDomainEnd());

    const MyBRep::Topology_Vertex v00(surface->pointAt(uv00.x(), uv00.y()));
    const MyBRep::Topology_Vertex v10(surface->pointAt(uv10.x(), uv10.y()));
    const MyBRep::Topology_Vertex v11(surface->pointAt(uv11.x(), uv11.y()));
    const MyBRep::Topology_Vertex v01(surface->pointAt(uv01.x(), uv01.y()));

    const MyMath::Vector3 bottomDirection = v10.point() - v00.point();
    const MyMath::Vector3 rightDirection = v11.point() - v10.point();
    const MyMath::Vector3 topDirection = v11.point() - v01.point();
    const MyMath::Vector3 leftDirection = v01.point() - v00.point();

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> bottomGeometry(
        new MyBRep::Geometry_Line(v00.point(), bottomDirection));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> rightGeometry(
        new MyBRep::Geometry_Line(v10.point(), rightDirection));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> topGeometry(
        new MyBRep::Geometry_Line(v01.point(), topDirection));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> leftGeometry(
        new MyBRep::Geometry_Line(v00.point(), leftDirection));

    MyBRep::Topology_Edge bottom(v00, v10, bottomGeometry, 0.0, bottomDirection.length(), TestTolerance);
    MyBRep::Topology_Edge right(v10, v11, rightGeometry, 0.0, rightDirection.length(), TestTolerance);
    MyBRep::Topology_Edge top(v01, v11, topGeometry, 0.0, topDirection.length(), TestTolerance);
    MyBRep::Topology_Edge left(v00, v01, leftGeometry, 0.0, leftDirection.length(), TestTolerance);

    addLineCurveOnSurface(bottom, surface, uv00, uv10);
    addLineCurveOnSurface(right, surface, uv10, uv11);
    addLineCurveOnSurface(top, surface, uv01, uv11);
    addLineCurveOnSurface(left, surface, uv00, uv01);

    std::vector<MyBRep::Topology_Edge> edges;
    edges.push_back(bottom);
    edges.push_back(right);
    edges.push_back(top.reversed());
    edges.push_back(left.reversed());

    return MyBRep::Modeling::createFace(
        surface,
        std::vector<MyBRep::Topology_Wire>(
            1,
            MyBRep::Topology_Wire(edges)));
}

inline MyBRep::Topology_Face createBezierFace()
{
    return createFullDomainFace(createBezierSurface());
}

inline MyBRep::Topology_Face createBSplineFace()
{
    return createFullDomainFace(createBSplineSurface());
}

}

#endif // MYBREP_SANDBOX_FREEFORMSURFACETESTFIXTURES_H
