#include <iostream>
#include <string>
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
#include "MyBRep/Geometry/Surface/Geometry_ConicalSurface.h"
#include "MyBRep/Geometry/Surface/Geometry_CylindricalSurface.h"
#include "MyBRep/Geometry/Surface/Geometry_SphericalSurface.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Modeling/Wire/WireModeling.h"
#include "MyBRep/Topology/Topology_Builder.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

#include "MyBRepOpenGL/Builder/BRepFaceBuilder.h"
#include "MyOpenGL/Resource/BufferGeometry.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // Surface分派测试统一使用的圆周率。
const double TestTolerance = 1.0e-8;                 // Topology和Curve-on-Surface连接统一使用的三维容差。

class TestContext
{
public:
    TestContext() : m_passed(0), m_failed(0)
    {
    }

    void expect(bool condition, const std::string& name)
    {
        if (condition)
        {
            ++m_passed;
            std::cout << "[PASS] " << name << std::endl;
        }
        else
        {
            ++m_failed;
            std::cout << "[FAIL] " << name << std::endl;
        }
    }

    int passed() const
    {
        return m_passed;
    }

    int failed() const
    {
        return m_failed;
    }

private:
    int m_passed;
    int m_failed;
};

void addLineCurveOnSurface(MyBRep::Topology_Edge& edge, const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>& surface, const MyMath::Vector2& firstUV, const MyMath::Vector2& lastUV)
{
    const MyMath::Vector2 direction = lastUV - firstUV;
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> curve(new MyBRep::Geometry_Line2D(firstUV, direction));
    MyBRep::Topology_Builder::addCurveOnSurface(edge, surface, curve, 0.0, direction.length(), TestTolerance);
}

MyBRep::Topology_Face createCylinderPatchFace(double radius, double u0, double u1, double v0, double v1)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface(new MyBRep::Geometry_CylindricalSurface(MyMath::CoordinateSystem::identity(), radius));
    const MyBRep::Geometry_CylindricalSurface& cylinder = static_cast<const MyBRep::Geometry_CylindricalSurface&>(*surface);

    const MyBRep::Topology_Vertex v00(cylinder.pointAt(u0, v0));
    const MyBRep::Topology_Vertex v10(cylinder.pointAt(u1, v0));
    const MyBRep::Topology_Vertex v11(cylinder.pointAt(u1, v1));
    const MyBRep::Topology_Vertex v01(cylinder.pointAt(u0, v1));

    const MyMath::Vector3 bottomCenter = cylinder.axisOrigin() + cylinder.axisDir() * v0;
    const MyMath::Vector3 topCenter = cylinder.axisOrigin() + cylinder.axisDir() * v1;

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> bottomGeometry(new MyBRep::Geometry_Circle(bottomCenter, radius, cylinder.xDir(), cylinder.yDir()));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> topGeometry(new MyBRep::Geometry_Circle(topCenter, radius, cylinder.xDir(), cylinder.yDir()));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> rightGeometry(new MyBRep::Geometry_Line(v10.point(), v11.point() - v10.point()));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> leftGeometry(new MyBRep::Geometry_Line(v00.point(), v01.point() - v00.point()));

    MyBRep::Topology_Edge bottom(v00, v10, bottomGeometry, u0, u1, TestTolerance);
    MyBRep::Topology_Edge right(v10, v11, rightGeometry, 0.0, (v11.point() - v10.point()).length(), TestTolerance);
    MyBRep::Topology_Edge top(v01, v11, topGeometry, u0, u1, TestTolerance);
    MyBRep::Topology_Edge left(v00, v01, leftGeometry, 0.0, (v01.point() - v00.point()).length(), TestTolerance);

    addLineCurveOnSurface(bottom, surface, MyMath::Vector2(u0, v0), MyMath::Vector2(u1, v0));
    addLineCurveOnSurface(right, surface, MyMath::Vector2(u1, v0), MyMath::Vector2(u1, v1));
    addLineCurveOnSurface(top, surface, MyMath::Vector2(u0, v1), MyMath::Vector2(u1, v1));
    addLineCurveOnSurface(left, surface, MyMath::Vector2(u0, v0), MyMath::Vector2(u0, v1));

    std::vector<MyBRep::Topology_Edge> edges;
    edges.push_back(bottom);
    edges.push_back(right);
    edges.push_back(top.reversed());
    edges.push_back(left.reversed());

    return MyBRep::Modeling::createFace(surface, std::vector<MyBRep::Topology_Wire>(1, MyBRep::Topology_Wire(edges)));
}

MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> createLatitudeCircle(const MyBRep::Geometry_SphericalSurface& sphere, double v)
{
    const double radius = sphere.radius() * std::cos(v);
    const MyMath::Vector3 center = sphere.center() + sphere.zDir() * (sphere.radius() * std::sin(v));
    return MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve>(new MyBRep::Geometry_Circle(center, radius, sphere.xDir(), sphere.yDir()));
}

MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> createMeridianCircle(const MyBRep::Geometry_SphericalSurface& sphere, double u)
{
    const MyMath::Vector3 radialDirection = sphere.xDir() * std::cos(u) + sphere.yDir() * std::sin(u);
    return MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve>(new MyBRep::Geometry_Circle(sphere.center(), sphere.radius(), radialDirection, sphere.zDir()));
}

MyBRep::Topology_Face createSpherePatchFace(double radius, double u0, double u1, double v0, double v1)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface(new MyBRep::Geometry_SphericalSurface(MyMath::CoordinateSystem::identity(), radius));
    const MyBRep::Geometry_SphericalSurface& sphere = static_cast<const MyBRep::Geometry_SphericalSurface&>(*surface);

    const MyBRep::Topology_Vertex v00(sphere.pointAt(u0, v0));
    const MyBRep::Topology_Vertex v10(sphere.pointAt(u1, v0));
    const MyBRep::Topology_Vertex v11(sphere.pointAt(u1, v1));
    const MyBRep::Topology_Vertex v01(sphere.pointAt(u0, v1));

    MyBRep::Topology_Edge bottom(v00, v10, createLatitudeCircle(sphere, v0), u0, u1, TestTolerance);
    MyBRep::Topology_Edge right(v10, v11, createMeridianCircle(sphere, u1), v0, v1, TestTolerance);
    MyBRep::Topology_Edge top(v01, v11, createLatitudeCircle(sphere, v1), u0, u1, TestTolerance);
    MyBRep::Topology_Edge left(v00, v01, createMeridianCircle(sphere, u0), v0, v1, TestTolerance);

    addLineCurveOnSurface(bottom, surface, MyMath::Vector2(u0, v0), MyMath::Vector2(u1, v0));
    addLineCurveOnSurface(right, surface, MyMath::Vector2(u1, v0), MyMath::Vector2(u1, v1));
    addLineCurveOnSurface(top, surface, MyMath::Vector2(u0, v1), MyMath::Vector2(u1, v1));
    addLineCurveOnSurface(left, surface, MyMath::Vector2(u0, v0), MyMath::Vector2(u0, v1));

    std::vector<MyBRep::Topology_Edge> edges;
    edges.push_back(bottom);
    edges.push_back(right);
    edges.push_back(top.reversed());
    edges.push_back(left.reversed());

    return MyBRep::Modeling::createFace(surface, std::vector<MyBRep::Topology_Wire>(1, MyBRep::Topology_Wire(edges)));
}

MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> createConeRingCurve(const MyBRep::Geometry_ConicalSurface& cone, double v)
{
    const double radius = v * cone.radialSlope();
    const MyMath::Vector3 center = cone.apex() + cone.axisDir() * v;
    return MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve>(new MyBRep::Geometry_Circle(center, radius, cone.xDir(), cone.yDir()));
}

MyBRep::Topology_Face createConePatchFace(double semiAngle, double u0, double u1, double v0, double v1)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface(new MyBRep::Geometry_ConicalSurface(MyMath::CoordinateSystem::identity(), semiAngle));
    const MyBRep::Geometry_ConicalSurface& cone = static_cast<const MyBRep::Geometry_ConicalSurface&>(*surface);

    const MyBRep::Topology_Vertex v00(cone.pointAt(u0, v0));
    const MyBRep::Topology_Vertex v10(cone.pointAt(u1, v0));
    const MyBRep::Topology_Vertex v11(cone.pointAt(u1, v1));
    const MyBRep::Topology_Vertex v01(cone.pointAt(u0, v1));

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> rightGeometry(new MyBRep::Geometry_Line(v10.point(), v11.point() - v10.point()));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> leftGeometry(new MyBRep::Geometry_Line(v00.point(), v01.point() - v00.point()));

    MyBRep::Topology_Edge bottom(v00, v10, createConeRingCurve(cone, v0), u0, u1, TestTolerance);
    MyBRep::Topology_Edge right(v10, v11, rightGeometry, 0.0, (v11.point() - v10.point()).length(), TestTolerance);
    MyBRep::Topology_Edge top(v01, v11, createConeRingCurve(cone, v1), u0, u1, TestTolerance);
    MyBRep::Topology_Edge left(v00, v01, leftGeometry, 0.0, (v01.point() - v00.point()).length(), TestTolerance);

    addLineCurveOnSurface(bottom, surface, MyMath::Vector2(u0, v0), MyMath::Vector2(u1, v0));
    addLineCurveOnSurface(right, surface, MyMath::Vector2(u1, v0), MyMath::Vector2(u1, v1));
    addLineCurveOnSurface(top, surface, MyMath::Vector2(u0, v1), MyMath::Vector2(u1, v1));
    addLineCurveOnSurface(left, surface, MyMath::Vector2(u0, v0), MyMath::Vector2(u0, v1));

    std::vector<MyBRep::Topology_Edge> edges;
    edges.push_back(bottom);
    edges.push_back(right);
    edges.push_back(top.reversed());
    edges.push_back(left.reversed());

    return MyBRep::Modeling::createFace(surface, std::vector<MyBRep::Topology_Wire>(1, MyBRep::Topology_Wire(edges)));
}

void testExistingPlanarCompatibility(TestContext& context)
{
    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createRectangle(10.0, 8.0), TestTolerance);

    MyBRep::Display::BRepFaceBuildOptions options;
    options.meshing.chordTolerance = 0.01; // 继续使用原有meshing字段，验证已有调用源码无需迁移。

    BufferGeometry* geometry = MyBRep::Display::BRepFaceBuilder::build(face, "PlanarFace", options);

    context.expect(options.isValid(), "BRepFaceBuildOptions remains valid with existing planar field");
    context.expect(geometry != 0, "Existing planar BRepFaceBuilder path still works");
    context.expect(geometry != 0 && geometry->vertexCount() == 4 && geometry->indexCount() == 6, "Existing planar rectangle Geometry unchanged");

    delete geometry;
}

void testCylinderBuilder(TestContext& context)
{
    const MyBRep::Topology_Face face = createCylinderPatchFace(5.0, 0.0, Pi * 0.5, -2.0, 2.0);

    MyBRep::Display::BRepFaceBuildOptions options;
    options.cylindricalMeshing.boundaryChordTolerance = 0.02;
    options.cylindricalMeshing.surfaceChordTolerance = 0.02;
    options.cylindricalMeshing.minimumBoundarySubdivisionDepth = 1;

    BufferGeometry* geometry = MyBRep::Display::BRepFaceBuilder::build(face, "CylinderFace", options);

    context.expect(geometry != 0, "BRepFaceBuilder creates Cylindrical Face Geometry");
    context.expect(geometry != 0 && geometry->renderType() == RenderType::Triangles, "Cylinder Face Geometry uses Triangles");
    context.expect(geometry != 0 && geometry->valuesPerVertex() == 6, "Cylinder Face Geometry uses Position Normal layout");
    context.expect(geometry != 0 && geometry->hasAttribute(GeometryAttribute::Position, 3), "Cylinder Face Geometry exposes Position");
    context.expect(geometry != 0 && geometry->hasAttribute(GeometryAttribute::Normal, 3), "Cylinder Face Geometry exposes Normal");
    context.expect(geometry != 0 && geometry->indexCount() > 6, "Cylinder Face Geometry preserves curvature subdivision");

    delete geometry;
}

void testCylinderToleranceForwarding(TestContext& context)
{
    const MyBRep::Topology_Face face = createCylinderPatchFace(6.0, 0.0, Pi, -2.0, 2.0);

    MyBRep::Display::BRepFaceBuildOptions loose;
    MyBRep::Display::BRepFaceBuildOptions tight;

    loose.cylindricalMeshing.boundaryChordTolerance = 0.05;
    tight.cylindricalMeshing.boundaryChordTolerance = 0.05;
    loose.cylindricalMeshing.surfaceChordTolerance = 0.20;
    tight.cylindricalMeshing.surfaceChordTolerance = 0.01;

    BufferGeometry* looseGeometry = MyBRep::Display::BRepFaceBuilder::build(face, "LooseCylinder", loose);
    BufferGeometry* tightGeometry = MyBRep::Display::BRepFaceBuilder::build(face, "TightCylinder", tight);

    context.expect(looseGeometry != 0 && tightGeometry != 0, "BRepFaceBuilder creates loose and tight Cylinder Geometry");
    context.expect(looseGeometry != 0 && tightGeometry != 0 && tightGeometry->indexCount() > looseGeometry->indexCount(), "BRepFaceBuilder forwards cylindrical surface tolerance");

    delete looseGeometry;
    delete tightGeometry;
}

void testSphereBuilder(TestContext& context)
{
    const MyBRep::Topology_Face face = createSpherePatchFace(5.0, 0.1, 1.5, -0.45, 0.55);

    MyBRep::Display::BRepFaceBuildOptions options;
    options.sphericalMeshing.boundaryChordTolerance = 0.02;
    options.sphericalMeshing.surfaceChordTolerance = 0.02;
    options.sphericalMeshing.minimumBoundarySubdivisionDepth = 1;

    BufferGeometry* geometry = MyBRep::Display::BRepFaceBuilder::build(face, "SphereFace", options);

    context.expect(geometry != 0, "BRepFaceBuilder creates Spherical Face Geometry");
    context.expect(geometry != 0 && geometry->renderType() == RenderType::Triangles, "Sphere Face Geometry uses Triangles");
    context.expect(geometry != 0 && geometry->valuesPerVertex() == 6, "Sphere Face Geometry uses Position Normal layout");
    context.expect(geometry != 0 && geometry->hasAttribute(GeometryAttribute::Position, 3), "Sphere Face Geometry exposes Position");
    context.expect(geometry != 0 && geometry->hasAttribute(GeometryAttribute::Normal, 3), "Sphere Face Geometry exposes Normal");
    context.expect(geometry != 0 && geometry->indexCount() > 6, "Sphere Face Geometry preserves curvature subdivision");

    delete geometry;
}

void testSphereToleranceForwarding(TestContext& context)
{
    const MyBRep::Topology_Face face = createSpherePatchFace(6.0, 0.0, Pi, -0.7, 0.7);

    MyBRep::Display::BRepFaceBuildOptions loose;
    MyBRep::Display::BRepFaceBuildOptions tight;

    loose.sphericalMeshing.boundaryChordTolerance = 0.05;
    tight.sphericalMeshing.boundaryChordTolerance = 0.05;
    loose.sphericalMeshing.surfaceChordTolerance = 0.20;
    tight.sphericalMeshing.surfaceChordTolerance = 0.01;

    BufferGeometry* looseGeometry = MyBRep::Display::BRepFaceBuilder::build(face, "LooseSphere", loose);
    BufferGeometry* tightGeometry = MyBRep::Display::BRepFaceBuilder::build(face, "TightSphere", tight);

    context.expect(looseGeometry != 0 && tightGeometry != 0, "BRepFaceBuilder creates loose and tight Sphere Geometry");
    context.expect(looseGeometry != 0 && tightGeometry != 0 && tightGeometry->indexCount() > looseGeometry->indexCount(), "BRepFaceBuilder forwards spherical surface tolerance");

    delete looseGeometry;
    delete tightGeometry;
}


void testConeBuilder(TestContext& context)
{
    const MyBRep::Topology_Face face = createConePatchFace(Pi / 6.0, 0.1, 1.5, 2.0, 6.0);

    MyBRep::Display::BRepFaceBuildOptions options;
    options.conicalMeshing.boundaryChordTolerance = 0.02;
    options.conicalMeshing.surfaceChordTolerance = 0.02;
    options.conicalMeshing.minimumBoundarySubdivisionDepth = 1;

    BufferGeometry* geometry = MyBRep::Display::BRepFaceBuilder::build(face, "ConeFace", options);

    context.expect(geometry != 0, "BRepFaceBuilder creates Conical Face Geometry");
    context.expect(geometry != 0 && geometry->renderType() == RenderType::Triangles, "Cone Face Geometry uses Triangles");
    context.expect(geometry != 0 && geometry->valuesPerVertex() == 6, "Cone Face Geometry uses Position Normal layout");
    context.expect(geometry != 0 && geometry->hasAttribute(GeometryAttribute::Position, 3), "Cone Face Geometry exposes Position");
    context.expect(geometry != 0 && geometry->hasAttribute(GeometryAttribute::Normal, 3), "Cone Face Geometry exposes Normal");
    context.expect(geometry != 0 && geometry->indexCount() > 6, "Cone Face Geometry preserves curvature subdivision");

    delete geometry;
}

void testConeToleranceForwarding(TestContext& context)
{
    const MyBRep::Topology_Face face = createConePatchFace(Pi / 6.0, 0.0, Pi, 2.0, 7.0);

    MyBRep::Display::BRepFaceBuildOptions loose;
    MyBRep::Display::BRepFaceBuildOptions tight;

    loose.conicalMeshing.boundaryChordTolerance = 0.05;
    tight.conicalMeshing.boundaryChordTolerance = 0.05;
    loose.conicalMeshing.surfaceChordTolerance = 0.20;
    tight.conicalMeshing.surfaceChordTolerance = 0.01;

    BufferGeometry* looseGeometry = MyBRep::Display::BRepFaceBuilder::build(face, "LooseCone", loose);
    BufferGeometry* tightGeometry = MyBRep::Display::BRepFaceBuilder::build(face, "TightCone", tight);

    context.expect(looseGeometry != 0 && tightGeometry != 0, "BRepFaceBuilder creates loose and tight Cone Geometry");
    context.expect(looseGeometry != 0 && tightGeometry != 0 && tightGeometry->indexCount() > looseGeometry->indexCount(), "BRepFaceBuilder forwards conical surface tolerance");

    delete looseGeometry;
    delete tightGeometry;
}

}

int main()
{
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep MyOpenGL Face Builder Surface Dispatch Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testExistingPlanarCompatibility(context);
    testCylinderBuilder(context);
    testCylinderToleranceForwarding(context);
    testSphereBuilder(context);
    testSphereToleranceForwarding(context);
    testConeBuilder(context);
    testConeToleranceForwarding(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}
