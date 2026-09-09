#ifndef MYBREP_SANDBOX_GENERATEDSURFACESOLIDFIXTURES_H
#define MYBREP_SANDBOX_GENERATEDSURFACESOLIDFIXTURES_H

#include <cmath>
#include <vector>

#include "MyMath/CoordinateSystem.h"
#include "MyMath/Vector2.h"
#include "MyMath/Vector3.h"

#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Curve/Geometry_Circle.h"
#include "MyBRep/Geometry/Curve/Geometry_Curve.h"
#include "MyBRep/Geometry/Curve/Geometry_Line.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Curve2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Line2D.h"
#include "MyBRep/Geometry/Surface/Geometry_Surface.h"
#include "MyBRep/Geometry/Surface/Geometry_SurfaceOfExtrusion.h"
#include "MyBRep/Geometry/Surface/Geometry_SurfaceOfRevolution.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Topology/Topology_Builder.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Face/Topology_Face.h"
#include "MyBRep/Topology/Shell/Topology_Shell.h"
#include "MyBRep/Topology/Solid/Topology_Solid.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

namespace GeneratedSurfaceSolidFixtures
{

const double Pi = 3.1415926535897932384626433832795; // 生成曲面Solid测试统一使用的圆周率。
const double TwoPi = Pi * 2.0;                        // U/V完整周期。
const double TestTolerance = 1.0e-8;                 // Topology及Curve-on-Surface连接统一使用的几何容差。

struct ExtrusionSolidFixture
{
    MyBRep::Topology_Edge bottomEdge;
    MyBRep::Topology_Edge topEdge;
    MyBRep::Topology_Edge seamEdge;
    MyBRep::Topology_Face sideFace;
    MyBRep::Topology_Face bottomFace;
    MyBRep::Topology_Face topFace;
    MyBRep::Topology_Shell shell;
    MyBRep::Topology_Solid solid;
};

struct RevolutionSolidFixture
{
    MyBRep::Topology_Edge uSeamEdge;
    MyBRep::Topology_Edge vSeamEdge;
    MyBRep::Topology_Face face;
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

inline MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> createXYCircle(const MyMath::Vector3& center, double radius)
{
    return MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve>(
        new MyBRep::Geometry_Circle(center, radius, MyMath::Vector3::unitX(), MyMath::Vector3::unitY()));
}

inline ExtrusionSolidFixture createExtrusionSolid(double radius, double v0, double v1)
{
    ExtrusionSolidFixture result;

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> profile = createXYCircle(MyMath::Vector3::zero(), radius);
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface(
        new MyBRep::Geometry_SurfaceOfExtrusion(profile, MyMath::Vector3::unitZ()));

    const MyBRep::Topology_Vertex bottomVertex(surface->pointAt(0.0, v0));
    const MyBRep::Topology_Vertex topVertex(surface->pointAt(0.0, v1));

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> bottomGeometry(
        createXYCircle(MyMath::Vector3(0.0, 0.0, v0), radius));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> topGeometry(
        createXYCircle(MyMath::Vector3(0.0, 0.0, v1), radius));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> seamGeometry(
        new MyBRep::Geometry_Line(bottomVertex.point(), topVertex.point() - bottomVertex.point()));

    result.bottomEdge = MyBRep::Topology_Edge(bottomVertex, bottomVertex, bottomGeometry, 0.0, TwoPi, TestTolerance);
    result.topEdge = MyBRep::Topology_Edge(topVertex, topVertex, topGeometry, 0.0, TwoPi, TestTolerance);
    result.seamEdge = MyBRep::Topology_Edge(bottomVertex, topVertex, seamGeometry, 0.0, v1 - v0, TestTolerance);

    addLineCurveOnSurface(result.bottomEdge, surface, MyMath::Vector2(0.0, v0), MyMath::Vector2(TwoPi, v0));
    addLineCurveOnSurface(result.topEdge, surface, MyMath::Vector2(0.0, v1), MyMath::Vector2(TwoPi, v1));

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> firstSeamCurve(
        new MyBRep::Geometry_Line2D(MyMath::Vector2(0.0, v0), MyMath::Vector2(0.0, 1.0)));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> secondSeamCurve(
        new MyBRep::Geometry_Line2D(MyMath::Vector2(TwoPi, v0), MyMath::Vector2(0.0, 1.0)));

    MyBRep::Topology_Builder::addCurveOnClosedSurface(
        result.seamEdge, surface, firstSeamCurve, 0.0, v1 - v0, secondSeamCurve, 0.0, v1 - v0, TestTolerance);

    std::vector<MyBRep::Topology_Edge> sideEdges;
    sideEdges.push_back(result.bottomEdge.reversed());
    sideEdges.push_back(result.seamEdge);
    sideEdges.push_back(result.topEdge);
    sideEdges.push_back(result.seamEdge.reversed());

    result.sideFace = MyBRep::Modeling::createFace(
        surface, std::vector<MyBRep::Topology_Wire>(1, MyBRep::Topology_Wire(sideEdges)));

    std::vector<MyBRep::Topology_Edge> bottomEdges;
    bottomEdges.push_back(result.bottomEdge);
    std::vector<MyBRep::Topology_Edge> topEdges;
    topEdges.push_back(result.topEdge.reversed());

    const MyMath::CoordinateSystem bottomSystem = MyMath::CoordinateSystem::fromAxes(
        MyMath::Vector3(0.0, 0.0, v0), MyMath::Vector3::unitX(), MyMath::Vector3::unitY() * -1.0, MyMath::Vector3::unitZ() * -1.0);
    const MyMath::CoordinateSystem topSystem = MyMath::CoordinateSystem::fromAxes(
        MyMath::Vector3(0.0, 0.0, v1), MyMath::Vector3::unitX(), MyMath::Vector3::unitY(), MyMath::Vector3::unitZ());

    result.bottomFace = MyBRep::Modeling::createPlanarFace(bottomSystem, MyBRep::Topology_Wire(bottomEdges), TestTolerance);
    result.topFace = MyBRep::Modeling::createPlanarFace(topSystem, MyBRep::Topology_Wire(topEdges), TestTolerance);

    std::vector<MyBRep::Topology_Face> faces;
    faces.push_back(result.sideFace);
    faces.push_back(result.bottomFace);
    faces.push_back(result.topFace);

    result.shell = MyBRep::Topology_Shell(faces);
    result.solid = MyBRep::Topology_Solid(result.shell);
    return result;
}

inline MyMath::Vector3 torusRadialDirection(double u)
{
    return MyMath::Vector3(std::cos(u), std::sin(u), 0.0);
}

inline RevolutionSolidFixture createRevolutionTorusSolid(double majorRadius, double minorRadius)
{
    RevolutionSolidFixture result;

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> profile(
        new MyBRep::Geometry_Circle(
            MyMath::Vector3(majorRadius, 0.0, 0.0), minorRadius, MyMath::Vector3::unitX(), MyMath::Vector3::unitZ()));

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface(
        new MyBRep::Geometry_SurfaceOfRevolution(profile, MyMath::Vector3::zero(), MyMath::Vector3::unitZ()));

    const MyBRep::Topology_Vertex seamVertex(surface->pointAt(0.0, 0.0));
    const double outerRadius = majorRadius + minorRadius;

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> vSeamGeometry = createXYCircle(MyMath::Vector3::zero(), outerRadius);

    result.uSeamEdge = MyBRep::Topology_Edge(seamVertex, seamVertex, profile, 0.0, TwoPi, TestTolerance);
    result.vSeamEdge = MyBRep::Topology_Edge(seamVertex, seamVertex, vSeamGeometry, 0.0, TwoPi, TestTolerance);

    // Forward uSeam在矩形右边界U=2π使用；Reversed uSeam在左边界U=0使用。
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> firstUSeamCurve(
        new MyBRep::Geometry_Line2D(MyMath::Vector2(TwoPi, 0.0), MyMath::Vector2(0.0, 1.0)));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> secondUSeamCurve(
        new MyBRep::Geometry_Line2D(MyMath::Vector2(0.0, 0.0), MyMath::Vector2(0.0, 1.0)));

    MyBRep::Topology_Builder::addCurveOnClosedSurface(
        result.uSeamEdge, surface, firstUSeamCurve, 0.0, TwoPi, secondUSeamCurve, 0.0, TwoPi, TestTolerance);

    // Forward vSeam在矩形下边界V=0使用；Reversed vSeam在上边界V=2π使用。
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> firstVSeamCurve(
        new MyBRep::Geometry_Line2D(MyMath::Vector2(0.0, 0.0), MyMath::Vector2(1.0, 0.0)));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> secondVSeamCurve(
        new MyBRep::Geometry_Line2D(MyMath::Vector2(0.0, TwoPi), MyMath::Vector2(1.0, 0.0)));

    MyBRep::Topology_Builder::addCurveOnClosedSurface(
        result.vSeamEdge, surface, firstVSeamCurve, 0.0, TwoPi, secondVSeamCurve, 0.0, TwoPi, TestTolerance);

    std::vector<MyBRep::Topology_Edge> faceEdges;
    faceEdges.push_back(result.vSeamEdge);
    faceEdges.push_back(result.uSeamEdge);
    faceEdges.push_back(result.vSeamEdge.reversed());
    faceEdges.push_back(result.uSeamEdge.reversed());

    result.face = MyBRep::Modeling::createFace(
        surface, std::vector<MyBRep::Topology_Wire>(1, MyBRep::Topology_Wire(faceEdges)));

    result.shell = MyBRep::Topology_Shell(std::vector<MyBRep::Topology_Face>(1, result.face));
    result.solid = MyBRep::Topology_Solid(result.shell);
    return result;
}

}

#endif // MYBREP_SANDBOX_GENERATEDSURFACESOLIDFIXTURES_H
