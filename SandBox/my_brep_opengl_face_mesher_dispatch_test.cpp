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

#include "MyBRep/Mesh/ConicalFaceMesher.h"
#include "MyBRep/Mesh/CylindricalFaceMesher.h"
#include "MyBRep/Mesh/FaceMesher.h"
#include "MyBRep/Mesh/PlanarFaceMesher.h"
#include "MyBRep/Mesh/SphericalFaceMesher.h"

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

bool sameMeshShape(const MyBRep::FaceMesh& first, const MyBRep::FaceMesh& second)
{
    return first.isValid() && second.isValid() && first.vertexCount() == second.vertexCount() && first.triangleCount() == second.triangleCount() && first.indices() == second.indices();
}

void testDefaultOptions(TestContext& context)
{
    const MyBRep::FaceMeshOptions options;
    context.expect(options.isValid(), "Default FaceMesher options valid");
}

void testPlanarDispatch(TestContext& context)
{
    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createRectangle(10.0, 8.0), TestTolerance);

    MyBRep::FaceMeshOptions options;
    options.planar.chordTolerance = 0.01;

    const MyBRep::FaceMesh direct = MyBRep::PlanarFaceMesher::mesh(face, options.planar);
    const MyBRep::FaceMesh dispatched = MyBRep::FaceMesher::mesh(face, options);

    context.expect(MyBRep::FaceMesher::canMesh(face), "FaceMesher recognizes Plane Face");
    context.expect(dispatched.isValid(), "FaceMesher dispatches Plane Face");
    context.expect(sameMeshShape(direct, dispatched), "Plane dispatch preserves PlanarFaceMesher result");
    context.expect(dispatched.vertexCount() == 4 && dispatched.triangleCount() == 2, "Plane dispatch keeps rectangle mesh");
}

void testCylindricalDispatch(TestContext& context)
{
    const MyBRep::Topology_Face face = createCylinderPatchFace(5.0, 0.0, Pi * 0.5, -2.0, 2.0);

    MyBRep::FaceMeshOptions options;
    options.cylindrical.boundaryChordTolerance = 0.02;
    options.cylindrical.surfaceChordTolerance = 0.02;
    options.cylindrical.minimumBoundarySubdivisionDepth = 1;

    const MyBRep::FaceMesh direct = MyBRep::CylindricalFaceMesher::mesh(face, options.cylindrical);
    const MyBRep::FaceMesh dispatched = MyBRep::FaceMesher::mesh(face, options);

    context.expect(MyBRep::FaceMesher::canMesh(face), "FaceMesher recognizes Cylindrical Face");
    context.expect(dispatched.isValid(), "FaceMesher dispatches Cylindrical Face");
    context.expect(sameMeshShape(direct, dispatched), "Cylinder dispatch preserves CylindricalFaceMesher result");
    context.expect(dispatched.triangleCount() > 2, "Cylinder dispatch preserves curvature subdivision");
}

void testCylindricalOptionsForwarding(TestContext& context)
{
    const MyBRep::Topology_Face face = createCylinderPatchFace(6.0, 0.0, Pi, -2.0, 2.0);

    MyBRep::FaceMeshOptions loose;
    MyBRep::FaceMeshOptions tight;

    loose.cylindrical.boundaryChordTolerance = 0.05;
    tight.cylindrical.boundaryChordTolerance = 0.05;
    loose.cylindrical.surfaceChordTolerance = 0.20;
    tight.cylindrical.surfaceChordTolerance = 0.01;

    const MyBRep::FaceMesh looseMesh = MyBRep::FaceMesher::mesh(face, loose);
    const MyBRep::FaceMesh tightMesh = MyBRep::FaceMesher::mesh(face, tight);

    context.expect(looseMesh.isValid() && tightMesh.isValid(), "FaceMesher forwards valid Cylinder options");
    context.expect(tightMesh.triangleCount() > looseMesh.triangleCount(), "FaceMesher forwards Cylinder surface tolerance");
}

void testSphericalDispatch(TestContext& context)
{
    const MyBRep::Topology_Face face = createSpherePatchFace(5.0, 0.1, 1.5, -0.45, 0.55);

    MyBRep::FaceMeshOptions options;
    options.spherical.boundaryChordTolerance = 0.02;
    options.spherical.surfaceChordTolerance = 0.02;
    options.spherical.minimumBoundarySubdivisionDepth = 1;

    const MyBRep::FaceMesh direct = MyBRep::SphericalFaceMesher::mesh(face, options.spherical);
    const MyBRep::FaceMesh dispatched = MyBRep::FaceMesher::mesh(face, options);

    context.expect(MyBRep::FaceMesher::canMesh(face), "FaceMesher recognizes Spherical Face");
    context.expect(dispatched.isValid(), "FaceMesher dispatches Spherical Face");
    context.expect(sameMeshShape(direct, dispatched), "Sphere dispatch preserves SphericalFaceMesher result");
    context.expect(dispatched.triangleCount() > 2, "Sphere dispatch preserves curvature subdivision");
}

void testSphericalOptionsForwarding(TestContext& context)
{
    const MyBRep::Topology_Face face = createSpherePatchFace(6.0, 0.0, Pi, -0.7, 0.7);

    MyBRep::FaceMeshOptions loose;
    MyBRep::FaceMeshOptions tight;

    loose.spherical.boundaryChordTolerance = 0.05;
    tight.spherical.boundaryChordTolerance = 0.05;
    loose.spherical.surfaceChordTolerance = 0.20;
    tight.spherical.surfaceChordTolerance = 0.01;

    const MyBRep::FaceMesh looseMesh = MyBRep::FaceMesher::mesh(face, loose);
    const MyBRep::FaceMesh tightMesh = MyBRep::FaceMesher::mesh(face, tight);

    context.expect(looseMesh.isValid() && tightMesh.isValid(), "FaceMesher forwards valid Sphere options");
    context.expect(tightMesh.triangleCount() > looseMesh.triangleCount(), "FaceMesher forwards Sphere surface tolerance");
}


void testConicalDispatch(TestContext& context)
{
    const MyBRep::Topology_Face face = createConePatchFace(Pi / 6.0, 0.1, 1.5, 2.0, 6.0);

    MyBRep::FaceMeshOptions options;
    options.conical.boundaryChordTolerance = 0.02;
    options.conical.surfaceChordTolerance = 0.02;
    options.conical.minimumBoundarySubdivisionDepth = 1;

    const MyBRep::FaceMesh direct = MyBRep::ConicalFaceMesher::mesh(face, options.conical);
    const MyBRep::FaceMesh dispatched = MyBRep::FaceMesher::mesh(face, options);

    context.expect(MyBRep::FaceMesher::canMesh(face), "FaceMesher recognizes Conical Face");
    context.expect(dispatched.isValid(), "FaceMesher dispatches Conical Face");
    context.expect(sameMeshShape(direct, dispatched), "Cone dispatch preserves ConicalFaceMesher result");
    context.expect(dispatched.triangleCount() > 2, "Cone dispatch preserves curvature subdivision");
}

void testConicalOptionsForwarding(TestContext& context)
{
    const MyBRep::Topology_Face face = createConePatchFace(Pi / 6.0, 0.0, Pi, 2.0, 7.0);

    MyBRep::FaceMeshOptions loose;
    MyBRep::FaceMeshOptions tight;

    loose.conical.boundaryChordTolerance = 0.05;
    tight.conical.boundaryChordTolerance = 0.05;
    loose.conical.surfaceChordTolerance = 0.20;
    tight.conical.surfaceChordTolerance = 0.01;

    const MyBRep::FaceMesh looseMesh = MyBRep::FaceMesher::mesh(face, loose);
    const MyBRep::FaceMesh tightMesh = MyBRep::FaceMesher::mesh(face, tight);

    context.expect(looseMesh.isValid() && tightMesh.isValid(), "FaceMesher forwards valid Cone options");
    context.expect(tightMesh.triangleCount() > looseMesh.triangleCount(), "FaceMesher forwards Cone surface tolerance");
}

void testUnsupportedSurface(TestContext& context)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface(new MyBRep::Geometry_ConicalSurface(MyMath::Vector3::zero(), Pi * 0.25));
    const MyBRep::Topology_Face face(surface);

    context.expect(!MyBRep::FaceMesher::canMesh(face), "Conical Face without trimming Wire cannot mesh");
    context.expect(MyBRep::FaceMesher::mesh(face).isEmpty(), "Conical Face without trimming Wire returns empty mesh");
}

}

int main()
{
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep Face Mesher Dispatch Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testDefaultOptions(context);
    testPlanarDispatch(context);
    testCylindricalDispatch(context);
    testCylindricalOptionsForwarding(context);
    testSphericalDispatch(context);
    testSphericalOptionsForwarding(context);
    testConicalDispatch(context);
    testConicalOptionsForwarding(context);
    testUnsupportedSurface(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}
