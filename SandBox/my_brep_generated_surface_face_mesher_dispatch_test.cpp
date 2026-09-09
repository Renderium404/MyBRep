#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "MyMath/Vector2.h"
#include "MyMath/Vector3.h"
#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Curve/Geometry_Circle.h"
#include "MyBRep/Geometry/Curve/Geometry_Curve.h"
#include "MyBRep/Geometry/Curve/Geometry_Line.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Curve2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Line2D.h"
#include "MyBRep/Geometry/Surface/Geometry_SurfaceOfExtrusion.h"
#include "MyBRep/Geometry/Surface/Geometry_SurfaceOfRevolution.h"
#include "MyBRep/Mesh/ExtrudedFaceMesher.h"
#include "MyBRep/Mesh/FaceMesher.h"
#include "MyBRep/Mesh/RevolvedFaceMesher.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Topology/Topology_Builder.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

namespace
{

const double TestTolerance = 1.0e-8; // 专项dispatcher测试使用的Topology/P-Curve连接容差。

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

void addLineCurveOnSurface(MyBRep::Topology_Edge& edge,
                           const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>& surface,
                           const MyMath::Vector2& firstUV,
                           const MyMath::Vector2& lastUV)
{
    const MyMath::Vector2 direction = lastUV - firstUV;
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> curve(new MyBRep::Geometry_Line2D(firstUV, direction));
    MyBRep::Topology_Builder::addCurveOnSurface(edge, surface, curve, 0.0, direction.length(), TestTolerance);
}

MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> createXYCircle(const MyMath::Vector3& center, double radius)
{
    return MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve>(
        new MyBRep::Geometry_Circle(center, radius, MyMath::Vector3::unitX(), MyMath::Vector3::unitY()));
}

MyBRep::Topology_Face createExtrusionPatchFace(double radius, double u0, double u1, double v0, double v1)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> profile = createXYCircle(MyMath::Vector3::zero(), radius);
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface(
        new MyBRep::Geometry_SurfaceOfExtrusion(profile, MyMath::Vector3::unitZ()));

    const MyBRep::Topology_Vertex v00(surface->pointAt(u0, v0));
    const MyBRep::Topology_Vertex v10(surface->pointAt(u1, v0));
    const MyBRep::Topology_Vertex v11(surface->pointAt(u1, v1));
    const MyBRep::Topology_Vertex v01(surface->pointAt(u0, v1));

    MyBRep::Topology_Edge bottom(v00, v10, createXYCircle(MyMath::Vector3(0.0, 0.0, v0), radius), u0, u1, TestTolerance);
    MyBRep::Topology_Edge top(v01, v11, createXYCircle(MyMath::Vector3(0.0, 0.0, v1), radius), u0, u1, TestTolerance);

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> rightGeometry(
        new MyBRep::Geometry_Line(v10.point(), v11.point() - v10.point()));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> leftGeometry(
        new MyBRep::Geometry_Line(v00.point(), v01.point() - v00.point()));

    MyBRep::Topology_Edge right(v10, v11, rightGeometry, 0.0, (v11.point() - v10.point()).length(), TestTolerance);
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

MyMath::Vector3 radialDirection(double u)
{
    return MyMath::Vector3(std::cos(u), std::sin(u), 0.0);
}

MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> createParallelCircle(double majorRadius, double minorRadius, double v)
{
    const double radius = majorRadius + minorRadius * std::cos(v);
    const double z = minorRadius * std::sin(v);
    return createXYCircle(MyMath::Vector3(0.0, 0.0, z), radius);
}

MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> createMeridianCircle(double majorRadius, double minorRadius, double u)
{
    const MyMath::Vector3 radial = radialDirection(u);
    return MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve>(
        new MyBRep::Geometry_Circle(radial * majorRadius, minorRadius, radial, MyMath::Vector3::unitZ()));
}

MyBRep::Topology_Face createRevolutionPatchFace(double majorRadius, double minorRadius, double u0, double u1, double v0, double v1)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> profile(
        new MyBRep::Geometry_Circle(MyMath::Vector3(majorRadius, 0.0, 0.0), minorRadius, MyMath::Vector3::unitX(), MyMath::Vector3::unitZ()));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface(
        new MyBRep::Geometry_SurfaceOfRevolution(profile, MyMath::Vector3::zero(), MyMath::Vector3::unitZ()));

    const MyBRep::Topology_Vertex v00(surface->pointAt(u0, v0));
    const MyBRep::Topology_Vertex v10(surface->pointAt(u1, v0));
    const MyBRep::Topology_Vertex v11(surface->pointAt(u1, v1));
    const MyBRep::Topology_Vertex v01(surface->pointAt(u0, v1));

    MyBRep::Topology_Edge bottom(v00, v10, createParallelCircle(majorRadius, minorRadius, v0), u0, u1, TestTolerance);
    MyBRep::Topology_Edge right(v10, v11, createMeridianCircle(majorRadius, minorRadius, u1), v0, v1, TestTolerance);
    MyBRep::Topology_Edge top(v01, v11, createParallelCircle(majorRadius, minorRadius, v1), u0, u1, TestTolerance);
    MyBRep::Topology_Edge left(v00, v01, createMeridianCircle(majorRadius, minorRadius, u0), v0, v1, TestTolerance);

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
    return first.isValid() &&
           second.isValid() &&
           first.vertexCount() == second.vertexCount() &&
           first.triangleCount() == second.triangleCount() &&
           first.indices() == second.indices();
}

void testExtrusionDispatch(TestContext& context)
{
    const MyBRep::Topology_Face face = createExtrusionPatchFace(3.0, 0.1, 1.8, -1.5, 2.0);

    MyBRep::FaceMeshOptions options;
    options.extruded.boundaryChordTolerance = 0.02;
    options.extruded.surfaceChordTolerance = 0.02;
    options.extruded.minimumBoundarySubdivisionDepth = 1;

    const MyBRep::FaceMesh direct = MyBRep::ExtrudedFaceMesher::mesh(face, options.extruded);
    const MyBRep::FaceMesh dispatched = MyBRep::FaceMesher::mesh(face, options);

    context.expect(MyBRep::FaceMesher::canMesh(face), "FaceMesher recognizes Extrusion Face");
    context.expect(dispatched.isValid(), "FaceMesher dispatches Extrusion Face");
    context.expect(sameMeshShape(direct, dispatched), "Extrusion dispatch preserves ExtrudedFaceMesher result");
    context.expect(dispatched.triangleCount() > 2, "Extrusion dispatch preserves curved-profile subdivision");
}

void testExtrusionToleranceForwarding(TestContext& context)
{
    const MyBRep::Topology_Face face = createExtrusionPatchFace(3.0, 0.0, 3.14159265358979323846, -2.0, 2.0);
    MyBRep::FaceMeshOptions loose;
    MyBRep::FaceMeshOptions tight;

    loose.extruded.surfaceChordTolerance = 0.20;
    tight.extruded.surfaceChordTolerance = 0.01;

    const MyBRep::FaceMesh looseMesh = MyBRep::FaceMesher::mesh(face, loose);
    const MyBRep::FaceMesh tightMesh = MyBRep::FaceMesher::mesh(face, tight);

    context.expect(looseMesh.isValid() && tightMesh.isValid(), "FaceMesher forwards valid Extrusion options");
    context.expect(tightMesh.triangleCount() > looseMesh.triangleCount(), "FaceMesher forwards Extrusion surface tolerance");
}

void testRevolutionDispatch(TestContext& context)
{
    const MyBRep::Topology_Face face = createRevolutionPatchFace(5.0, 1.5, 0.2, 1.6, -0.8, 0.9);

    MyBRep::FaceMeshOptions options;
    options.revolved.boundaryChordTolerance = 0.02;
    options.revolved.surfaceChordTolerance = 0.02;
    options.revolved.minimumBoundarySubdivisionDepth = 1;

    const MyBRep::FaceMesh direct = MyBRep::RevolvedFaceMesher::mesh(face, options.revolved);
    const MyBRep::FaceMesh dispatched = MyBRep::FaceMesher::mesh(face, options);

    context.expect(MyBRep::FaceMesher::canMesh(face), "FaceMesher recognizes Revolution Face");
    context.expect(dispatched.isValid(), "FaceMesher dispatches Revolution Face");
    context.expect(sameMeshShape(direct, dispatched), "Revolution dispatch preserves RevolvedFaceMesher result");
    context.expect(dispatched.triangleCount() > 2, "Revolution dispatch preserves doubly-curved subdivision");
}

void testRevolutionToleranceForwarding(TestContext& context)
{
    const MyBRep::Topology_Face face = createRevolutionPatchFace(5.0, 1.5, 0.0, 3.14159265358979323846, -1.2, 1.2);
    MyBRep::FaceMeshOptions loose;
    MyBRep::FaceMeshOptions tight;

    loose.revolved.surfaceChordTolerance = 0.20;
    tight.revolved.surfaceChordTolerance = 0.01;

    const MyBRep::FaceMesh looseMesh = MyBRep::FaceMesher::mesh(face, loose);
    const MyBRep::FaceMesh tightMesh = MyBRep::FaceMesher::mesh(face, tight);

    context.expect(looseMesh.isValid() && tightMesh.isValid(), "FaceMesher forwards valid Revolution options");
    context.expect(tightMesh.triangleCount() > looseMesh.triangleCount(), "FaceMesher forwards Revolution surface tolerance");
}

}

int main()
{
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep Generated Surface FaceMesher Dispatch Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    context.expect(MyBRep::FaceMeshOptions().isValid(), "Default six-surface FaceMeshOptions valid");
    testExtrusionDispatch(context);
    testExtrusionToleranceForwarding(context);
    testRevolutionDispatch(context);
    testRevolutionToleranceForwarding(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}
