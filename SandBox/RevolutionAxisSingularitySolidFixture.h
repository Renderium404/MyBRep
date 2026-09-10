#ifndef MYBREP_SANDBOX_REVOLUTIONAXISSINGULARITYSOLIDFIXTURE_H
#define MYBREP_SANDBOX_REVOLUTIONAXISSINGULARITYSOLIDFIXTURE_H

#include <vector>

#include "MyMath/Vector2.h"
#include "MyMath/Vector3.h"

#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Curve/Geometry_Circle.h"
#include "MyBRep/Geometry/Curve/Geometry_Curve.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Curve2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Line2D.h"
#include "MyBRep/Geometry/Surface/Geometry_Surface.h"
#include "MyBRep/Geometry/Surface/Geometry_SurfaceOfRevolution.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Topology/Topology_Builder.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Face/Topology_Face.h"
#include "MyBRep/Topology/Shell/Topology_Shell.h"
#include "MyBRep/Topology/Solid/Topology_Solid.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

namespace RevolutionAxisSingularitySolidFixture
{

using namespace MyBRep;

const double Pi = 3.1415926535897932384626433832795; // 轴奇点Solid测试统一使用的圆周率。
const double HalfPi = Pi * 0.5;                       // 南北pole对应的V参数。
const double TwoPi = Pi * 2.0;                        // Revolution完整U周期。
const double TestTolerance = 1.0e-8;                 // Topology及P-Curve连接容差。

struct Fixture
{
    Topology_Vertex southVertex; // 南极拓扑点。
    Topology_Vertex northVertex; // 北极拓扑点。
    Topology_Edge seamEdge;      // 唯一seam TEdge，连接南北pole。
    Topology_Face face;          // 完整双pole Revolution Face。
    Topology_Shell shell;        // 单Face闭合Shell。
    Topology_Solid solid;        // 最终Solid。
};

inline Foundation::RefPtr<const Geometry_Curve> createSphereProfile(double radius)
{
    return Foundation::RefPtr<const Geometry_Curve>(
        new Geometry_Circle(
            MyMath::Vector3::zero(),
            radius,
            MyMath::Vector3::unitX(),
            MyMath::Vector3::unitZ()));
}

inline Foundation::RefPtr<const Geometry_Surface> createSurface(double radius)
{
    return Foundation::RefPtr<const Geometry_Surface>(
        new Geometry_SurfaceOfRevolution(
            createSphereProfile(radius),
            MyMath::Vector3::zero(),
            MyMath::Vector3::unitZ()));
}

inline Foundation::RefPtr<const Geometry_Curve> createMeridian(double radius)
{
    return Foundation::RefPtr<const Geometry_Curve>(
        new Geometry_Circle(
            MyMath::Vector3::zero(),
            radius,
            MyMath::Vector3::unitX(),
            MyMath::Vector3::unitZ()));
}

inline Fixture create(double radius)
{
    Fixture result;

    const Foundation::RefPtr<const Geometry_Surface> surface = createSurface(radius);

    result.southVertex = Topology_Vertex(surface->pointAt(0.0, -HalfPi));
    result.northVertex = Topology_Vertex(surface->pointAt(0.0, HalfPi));
    result.seamEdge = Topology_Edge(
        result.southVertex,
        result.northVertex,
        createMeridian(radius),
        -HalfPi,
        HalfPi,
        TestTolerance);

    const Foundation::RefPtr<const Geometry_Curve2D> firstSeamCurve(
        new Geometry_Line2D(
            MyMath::Vector2(0.0, -HalfPi),
            MyMath::Vector2(0.0, 1.0)));
    const Foundation::RefPtr<const Geometry_Curve2D> secondSeamCurve(
        new Geometry_Line2D(
            MyMath::Vector2(TwoPi, -HalfPi),
            MyMath::Vector2(0.0, 1.0)));

    Topology_Builder::addCurveOnClosedSurface(
        result.seamEdge,
        surface,
        firstSeamCurve,
        0.0,
        Pi,
        secondSeamCurve,
        0.0,
        Pi,
        TestTolerance);

    std::vector<Topology_Edge> edges;
    edges.push_back(result.seamEdge);
    edges.push_back(result.seamEdge.reversed());

    result.face = Modeling::createFace(
        surface,
        std::vector<Topology_Wire>(1, Topology_Wire(edges)));

    result.shell = Topology_Shell(
        std::vector<Topology_Face>(1, result.face));
    result.solid = Topology_Solid(result.shell);

    return result;
}

}

#endif // MYBREP_SANDBOX_REVOLUTIONAXISSINGULARITYSOLIDFIXTURE_H
