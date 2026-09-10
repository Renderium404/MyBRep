#ifndef MYBREP_SANDBOX_MIXEDFREEFORMSOLIDFIXTURE_H
#define MYBREP_SANDBOX_MIXEDFREEFORMSOLIDFIXTURE_H

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
#include "MyBRep/Geometry/Surface/Geometry_SurfaceOfExtrusion.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Topology/Topology_Builder.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Face/Topology_Face.h"
#include "MyBRep/Topology/Shell/Topology_Shell.h"
#include "MyBRep/Topology/Solid/Topology_Solid.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

namespace MixedFreeformSolidFixture
{

const double Size = 4.0;              // XY方向统一边长。
const double BottomZ = -2.0;          // B-Spline底面边界高度。
const double TopZ = 2.0;              // Bezier顶面边界高度。
const double Height = TopZ - BottomZ; // 四个Extrusion侧面的拉伸高度。
const double TestTolerance = 1.0e-8;  // Topology与P-Curve连接容差。

struct Fixture
{
    MyBRep::Topology_Face topFace;              // Bezier顶面。
    MyBRep::Topology_Face bottomFace;           // Reversed B-Spline底面。
    std::vector<MyBRep::Topology_Face> sides;   // 四个Extrusion侧面。
    std::vector<MyBRep::Topology_Edge> topEdges;
    std::vector<MyBRep::Topology_Edge> bottomEdges;
    std::vector<MyBRep::Topology_Edge> verticalEdges;
    MyBRep::Topology_Shell shell;
    MyBRep::Topology_Solid solid;
};

inline void addLineCurveOnSurface(MyBRep::Topology_Edge& edge,
                                  const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>& surface,
                                  const MyMath::Vector2& firstUV,
                                  const MyMath::Vector2& lastUV)
{
    const MyMath::Vector2 direction = lastUV - firstUV;
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> curve(new MyBRep::Geometry_Line2D(firstUV, direction));
    MyBRep::Topology_Builder::addCurveOnSurface(edge, surface, curve, 0.0, direction.length(), TestTolerance);
}

inline std::vector<MyMath::Vector3> createTopControlPoints()
{
    const double c[4] = {0.0, Size / 3.0, Size * 2.0 / 3.0, Size};
    const double h[4][4] = {{0,0,0,0},{0,0.9,1.7,0},{0,1.5,0.8,0},{0,0,0,0}};
    std::vector<MyMath::Vector3> points;
    points.reserve(16);
    for (std::size_t v = 0; v < 4; ++v)
        for (std::size_t u = 0; u < 4; ++u)
            points.push_back(MyMath::Vector3(c[u], c[v], TopZ + h[v][u]));
    return points;
}

inline std::vector<double> createKnots()
{
    const double values[7] = {0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0};
    return std::vector<double>(values, values + 7);
}

inline std::vector<MyMath::Vector3> createBottomControlPoints()
{
    const double c[4] = {0.0, 1.0, 3.0, Size};
    const double h[4][4] = {{0,0,0,0},{0,-0.7,-1.3,0},{0,-1.5,-0.6,0},{0,0,0,0}};
    std::vector<MyMath::Vector3> points;
    points.reserve(16);
    for (std::size_t v = 0; v < 4; ++v)
        for (std::size_t u = 0; u < 4; ++u)
            points.push_back(MyMath::Vector3(c[u], c[v], BottomZ + h[v][u]));
    return points;
}

inline MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> createTopSurface()
{
    return MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>(new MyBRep::Geometry_BezierSurface(4, 4, createTopControlPoints()));
}

inline MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> createBottomSurface()
{
    return MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>(
        new MyBRep::Geometry_BSplineSurface(2, 2, 4, 4, createBottomControlPoints(), createKnots(), createKnots()));
}

inline MyBRep::Topology_Edge createLineEdge(const MyBRep::Topology_Vertex& start,
                                            const MyBRep::Topology_Vertex& end,
                                            const MyMath::Vector3& unitDirection,
                                            double length)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> geometry(new MyBRep::Geometry_Line(start.point(), unitDirection));
    return MyBRep::Topology_Edge(start, end, geometry, 0.0, length, TestTolerance);
}

inline MyBRep::Topology_Face createSideFace(MyBRep::Topology_Edge& bottom,
                                            MyBRep::Topology_Edge& top,
                                            MyBRep::Topology_Edge& startVertical,
                                            MyBRep::Topology_Edge& endVertical,
                                            const MyMath::Vector3& origin,
                                            const MyMath::Vector3& direction)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> profile(new MyBRep::Geometry_Line(origin, direction));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface(
        new MyBRep::Geometry_SurfaceOfExtrusion(profile, MyMath::Vector3::unitZ()));

    addLineCurveOnSurface(bottom, surface, MyMath::Vector2(0.0, 0.0), MyMath::Vector2(Size, 0.0));
    addLineCurveOnSurface(endVertical, surface, MyMath::Vector2(Size, 0.0), MyMath::Vector2(Size, Height));
    addLineCurveOnSurface(top, surface, MyMath::Vector2(0.0, Height), MyMath::Vector2(Size, Height));
    addLineCurveOnSurface(startVertical, surface, MyMath::Vector2(0.0, 0.0), MyMath::Vector2(0.0, Height));

    std::vector<MyBRep::Topology_Edge> edges;
    edges.push_back(bottom);
    edges.push_back(endVertical);
    edges.push_back(top.reversed());
    edges.push_back(startVertical.reversed());
    return MyBRep::Modeling::createFace(surface, std::vector<MyBRep::Topology_Wire>(1, MyBRep::Topology_Wire(edges)));
}

inline Fixture create()
{
    Fixture result;
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> topSurface = createTopSurface();
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> bottomSurface = createBottomSurface();

    const MyBRep::Topology_Vertex b00(bottomSurface->pointAt(0.0, 0.0));
    const MyBRep::Topology_Vertex b10(bottomSurface->pointAt(1.0, 0.0));
    const MyBRep::Topology_Vertex b11(bottomSurface->pointAt(1.0, 1.0));
    const MyBRep::Topology_Vertex b01(bottomSurface->pointAt(0.0, 1.0));
    const MyBRep::Topology_Vertex t00(topSurface->pointAt(0.0, 0.0));
    const MyBRep::Topology_Vertex t10(topSurface->pointAt(1.0, 0.0));
    const MyBRep::Topology_Vertex t11(topSurface->pointAt(1.0, 1.0));
    const MyBRep::Topology_Vertex t01(topSurface->pointAt(0.0, 1.0));

    result.bottomEdges.push_back(createLineEdge(b00, b10, MyMath::Vector3::unitX(), Size));
    result.bottomEdges.push_back(createLineEdge(b10, b11, MyMath::Vector3::unitY(), Size));
    result.bottomEdges.push_back(createLineEdge(b11, b01, MyMath::Vector3::unitX() * -1.0, Size));
    result.bottomEdges.push_back(createLineEdge(b01, b00, MyMath::Vector3::unitY() * -1.0, Size));
    result.topEdges.push_back(createLineEdge(t00, t10, MyMath::Vector3::unitX(), Size));
    result.topEdges.push_back(createLineEdge(t10, t11, MyMath::Vector3::unitY(), Size));
    result.topEdges.push_back(createLineEdge(t11, t01, MyMath::Vector3::unitX() * -1.0, Size));
    result.topEdges.push_back(createLineEdge(t01, t00, MyMath::Vector3::unitY() * -1.0, Size));
    result.verticalEdges.push_back(createLineEdge(b00, t00, MyMath::Vector3::unitZ(), Height));
    result.verticalEdges.push_back(createLineEdge(b10, t10, MyMath::Vector3::unitZ(), Height));
    result.verticalEdges.push_back(createLineEdge(b11, t11, MyMath::Vector3::unitZ(), Height));
    result.verticalEdges.push_back(createLineEdge(b01, t01, MyMath::Vector3::unitZ(), Height));

    const MyMath::Vector2 uv[4] = {MyMath::Vector2(0,0), MyMath::Vector2(1,0), MyMath::Vector2(1,1), MyMath::Vector2(0,1)};
    for (std::size_t i = 0; i < 4; ++i)
    {
        const std::size_t j = (i + 1) % 4;
        addLineCurveOnSurface(result.topEdges[i], topSurface, uv[i], uv[j]);
        addLineCurveOnSurface(result.bottomEdges[i], bottomSurface, uv[i], uv[j]);
    }

    result.topFace = MyBRep::Modeling::createFace(topSurface, std::vector<MyBRep::Topology_Wire>(1, MyBRep::Topology_Wire(result.topEdges)));
    result.bottomFace = MyBRep::Modeling::createFace(
        bottomSurface, std::vector<MyBRep::Topology_Wire>(1, MyBRep::Topology_Wire(result.bottomEdges))).reversed();

    result.sides.push_back(createSideFace(result.bottomEdges[0], result.topEdges[0], result.verticalEdges[0], result.verticalEdges[1],
                                          MyMath::Vector3(0,0,BottomZ), MyMath::Vector3::unitX()));
    result.sides.push_back(createSideFace(result.bottomEdges[1], result.topEdges[1], result.verticalEdges[1], result.verticalEdges[2],
                                          MyMath::Vector3(Size,0,BottomZ), MyMath::Vector3::unitY()));
    result.sides.push_back(createSideFace(result.bottomEdges[2], result.topEdges[2], result.verticalEdges[2], result.verticalEdges[3],
                                          MyMath::Vector3(Size,Size,BottomZ), MyMath::Vector3::unitX() * -1.0));
    result.sides.push_back(createSideFace(result.bottomEdges[3], result.topEdges[3], result.verticalEdges[3], result.verticalEdges[0],
                                          MyMath::Vector3(0,Size,BottomZ), MyMath::Vector3::unitY() * -1.0));

    std::vector<MyBRep::Topology_Face> faces;
    faces.push_back(result.topFace);
    faces.push_back(result.bottomFace);
    faces.insert(faces.end(), result.sides.begin(), result.sides.end());
    result.shell = MyBRep::Topology_Shell(faces);
    result.solid = MyBRep::Topology_Solid(result.shell);
    return result;
}

}

#endif // MYBREP_SANDBOX_MIXEDFREEFORMSOLIDFIXTURE_H