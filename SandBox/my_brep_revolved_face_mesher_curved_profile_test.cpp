#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "MyMath/Vector2.h"
#include "MyMath/Vector3.h"

#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Curve/Geometry_Circle.h"
#include "MyBRep/Geometry/Curve/Geometry_Curve.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Curve2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Line2D.h"
#include "MyBRep/Geometry/Surface/Geometry_SurfaceOfRevolution.h"
#include "MyBRep/Mesh/RevolvedFaceMesher.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Topology/Topology_Builder.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

namespace
{
const double Pi = 3.1415926535897932384626433832795; // 曲线母线专项测试统一使用的圆周率。
const double TwoPi = Pi * 2.0;                        // 完整周期参数。
const double TestTolerance = 1.0e-8;                 // Topology与Curve-on-Surface连接统一使用的几何容差。
const double AreaTolerance = 0.45;                   // 曲面离散面积与解析面积比较允许的绝对误差。
const double NormalTolerance = 1.0e-6;               // FaceMesh法向比较容差。

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

double meshArea(const MyBRep::FaceMesh& mesh)
{
    double area = 0.0;

    for (std::size_t index = 0; index < mesh.indices().size(); index += 3)
    {
        const MyMath::Vector3& first = mesh.vertices()[mesh.indices()[index]].position;
        const MyMath::Vector3& second = mesh.vertices()[mesh.indices()[index + 1]].position;
        const MyMath::Vector3& third = mesh.vertices()[mesh.indices()[index + 2]].position;
        area += MyMath::Vector3::cross(second - first, third - first).length() * 0.5;
    }

    return area;
}

bool triangleWindingMatchesNormals(const MyBRep::FaceMesh& mesh)
{
    if (!mesh.isValid())
    {
        return false;
    }

    for (std::size_t index = 0; index < mesh.indices().size(); index += 3)
    {
        const MyBRep::FaceMeshVertex& first = mesh.vertices()[mesh.indices()[index]];
        const MyBRep::FaceMeshVertex& second = mesh.vertices()[mesh.indices()[index + 1]];
        const MyBRep::FaceMeshVertex& third = mesh.vertices()[mesh.indices()[index + 2]];
        const MyMath::Vector3 triangleNormal = MyMath::Vector3::cross(second.position - first.position, third.position - first.position);
        const MyMath::Vector3 averageNormal = first.normal + second.normal + third.normal;

        if (!triangleNormal.isVector(0.0) || MyMath::Vector3::dot(triangleNormal, averageNormal) <= 0.0)
        {
            return false;
        }
    }

    return true;
}

bool vertexNormalsMatchFace(const MyBRep::Topology_Face& face, const MyBRep::FaceMesh& mesh)
{
    if (!mesh.isValid())
    {
        return false;
    }

    for (std::size_t index = 0; index < mesh.vertices().size(); ++index)
    {
        const MyBRep::FaceMeshVertex& vertex = mesh.vertices()[index];
        const MyMath::Vector3 expected = face.normalAt(vertex.parameter.x(), vertex.parameter.y());

        if (!vertex.normal.isEqualTo(expected, NormalTolerance))
        {
            return false;
        }
    }

    return true;
}

double parameterSpan(const MyBRep::FaceMesh& mesh, int axis)
{
    if (mesh.vertices().empty())
    {
        return 0.0;
    }

    double minimum = axis == 0 ? mesh.vertices()[0].parameter.x() : mesh.vertices()[0].parameter.y();
    double maximum = minimum;

    for (std::size_t index = 1; index < mesh.vertices().size(); ++index)
    {
        const double value = axis == 0 ? mesh.vertices()[index].parameter.x() : mesh.vertices()[index].parameter.y();
        minimum = (std::min)(minimum, value);
        maximum = (std::max)(maximum, value);
    }

    return maximum - minimum;
}

MyBRep::RevolvedFaceMeshOptions testOptions()
{
    MyBRep::RevolvedFaceMeshOptions options;
    options.boundaryChordTolerance = 0.02;
    options.surfaceChordTolerance = 0.02;
    options.geometricTolerance = 1.0e-10;
    options.minimumBoundarySubdivisionDepth = 1;
    options.maximumBoundarySubdivisionDepth = 12;
    options.maximumSurfaceSubdivisionRounds = 12;
    return options;
}

MyMath::Vector3 radialDirection(double u)
{
    return MyMath::Vector3(std::cos(u), std::sin(u), 0.0);
}

MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> createTorusSurface(double majorRadius, double minorRadius)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> profile(
        new MyBRep::Geometry_Circle(MyMath::Vector3(majorRadius, 0.0, 0.0), minorRadius, MyMath::Vector3::unitX(), MyMath::Vector3::unitZ()));

    return MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>(
        new MyBRep::Geometry_SurfaceOfRevolution(profile, MyMath::Vector3::zero(), MyMath::Vector3::unitZ()));
}

MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> createParallelCircle(double majorRadius, double minorRadius, double v)
{
    const double radius = majorRadius + minorRadius * std::cos(v);
    const double z = minorRadius * std::sin(v);

    return MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve>(
        new MyBRep::Geometry_Circle(MyMath::Vector3(0.0, 0.0, z), radius, MyMath::Vector3::unitX(), MyMath::Vector3::unitY()));
}

MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> createMeridianCircle(double majorRadius, double minorRadius, double u)
{
    const MyMath::Vector3 radial = radialDirection(u);
    const MyMath::Vector3 center = radial * majorRadius;

    return MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve>(
        new MyBRep::Geometry_Circle(center, minorRadius, radial, MyMath::Vector3::unitZ()));
}

MyBRep::Topology_Wire createPatchWire(const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>& surface,
                                      double majorRadius,
                                      double minorRadius,
                                      double u0,
                                      double u1,
                                      double v0,
                                      double v1)
{
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
    return MyBRep::Topology_Wire(edges);
}

MyBRep::Topology_Face createPatchFace(double majorRadius, double minorRadius, double u0, double u1, double v0, double v1)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface = createTorusSurface(majorRadius, minorRadius);
    return MyBRep::Modeling::createFace(surface, std::vector<MyBRep::Topology_Wire>(1, createPatchWire(surface, majorRadius, minorRadius, u0, u1, v0, v1)));
}

MyBRep::Topology_Face createFaceWithHole(double majorRadius, double minorRadius)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface = createTorusSurface(majorRadius, minorRadius);
    std::vector<MyBRep::Topology_Wire> wires;
    wires.push_back(createPatchWire(surface, majorRadius, minorRadius, 0.0, 2.2, -1.0, 1.0));
    wires.push_back(createPatchWire(surface, majorRadius, minorRadius, 0.7, 1.4, -0.35, 0.35));
    return MyBRep::Modeling::createFace(surface, wires);
}

MyBRep::Topology_Face createFullVBandFace(double majorRadius, double minorRadius, double u0, double u1)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface = createTorusSurface(majorRadius, minorRadius);
    const MyBRep::Topology_Vertex leftVertex(surface->pointAt(u0, 0.0));
    const MyBRep::Topology_Vertex rightVertex(surface->pointAt(u1, 0.0));

    MyBRep::Topology_Edge seam(leftVertex, rightVertex, createParallelCircle(majorRadius, minorRadius, 0.0), u0, u1, TestTolerance);
    MyBRep::Topology_Edge right(rightVertex, rightVertex, createMeridianCircle(majorRadius, minorRadius, u1), 0.0, TwoPi, TestTolerance);
    MyBRep::Topology_Edge left(leftVertex, leftVertex, createMeridianCircle(majorRadius, minorRadius, u0), 0.0, TwoPi, TestTolerance);

    addLineCurveOnSurface(right, surface, MyMath::Vector2(u1, 0.0), MyMath::Vector2(u1, TwoPi));
    addLineCurveOnSurface(left, surface, MyMath::Vector2(u0, 0.0), MyMath::Vector2(u0, TwoPi));

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> firstSeamCurve(
        new MyBRep::Geometry_Line2D(MyMath::Vector2(u0, 0.0), MyMath::Vector2(1.0, 0.0)));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> secondSeamCurve(
        new MyBRep::Geometry_Line2D(MyMath::Vector2(u0, TwoPi), MyMath::Vector2(1.0, 0.0)));

    MyBRep::Topology_Builder::addCurveOnClosedSurface(
        seam, surface, firstSeamCurve, 0.0, u1 - u0, secondSeamCurve, 0.0, u1 - u0, TestTolerance);

    std::vector<MyBRep::Topology_Edge> edges;
    edges.push_back(seam);
    edges.push_back(right);
    edges.push_back(seam.reversed());
    edges.push_back(left.reversed());

    return MyBRep::Modeling::createFace(surface, std::vector<MyBRep::Topology_Wire>(1, MyBRep::Topology_Wire(edges)));
}

double torusPatchArea(double majorRadius, double minorRadius, double u0, double u1, double v0, double v1)
{
    const double vIntegral = majorRadius * (v1 - v0) + minorRadius * (std::sin(v1) - std::sin(v0));
    return minorRadius * (u1 - u0) * vIntegral;
}

void testDoublyCurvedPatch(TestContext& context)
{
    const double majorRadius = 5.0;
    const double minorRadius = 1.5;
    const double u0 = 0.2;
    const double u1 = 1.6;
    const double v0 = -0.8;
    const double v1 = 0.9;
    const MyBRep::Topology_Face face = createPatchFace(majorRadius, minorRadius, u0, u1, v0, v1);
    const MyBRep::FaceMesh mesh = MyBRep::RevolvedFaceMesher::mesh(face, testOptions());

    context.expect(MyBRep::RevolvedFaceMesher::canMesh(face), "Doubly-periodic torus patch Face can mesh");
    context.expect(mesh.isValid(), "Doubly-curved revolution mesh valid");
    context.expect(mesh.triangleCount() > 2, "Doubly-curved revolution receives U/V surface subdivision");
    context.expect(std::fabs(meshArea(mesh) - torusPatchArea(majorRadius, minorRadius, u0, u1, v0, v1)) <= AreaTolerance, "Doubly-curved revolution mesh area");
    context.expect(vertexNormalsMatchFace(face, mesh), "Doubly-curved revolution vertex normals match Face");
    context.expect(triangleWindingMatchesNormals(mesh), "Doubly-curved revolution winding matches normals");
}

void testHole(TestContext& context)
{
    const double majorRadius = 5.0;
    const double minorRadius = 1.5;
    const MyBRep::Topology_Face face = createFaceWithHole(majorRadius, minorRadius);
    const MyBRep::FaceMesh mesh = MyBRep::RevolvedFaceMesher::mesh(face, testOptions());
    const double outerArea = torusPatchArea(majorRadius, minorRadius, 0.0, 2.2, -1.0, 1.0);
    const double holeArea = torusPatchArea(majorRadius, minorRadius, 0.7, 1.4, -0.35, 0.35);

    context.expect(mesh.isValid(), "Doubly-periodic revolution Face with hole mesh valid");
    context.expect(std::fabs(meshArea(mesh) - (outerArea - holeArea)) <= AreaTolerance, "Doubly-periodic revolution Face with hole area");
    context.expect(triangleWindingMatchesNormals(mesh), "Doubly-periodic revolution Face with hole winding");
}

void testVSeamCrossing(TestContext& context)
{
    const MyBRep::Topology_Face face = createPatchFace(5.0, 1.5, 0.3, 1.4, 5.7, 6.8);
    const MyBRep::FaceMesh mesh = MyBRep::RevolvedFaceMesher::mesh(face, testOptions());
    bool hasUnwrappedV = false;

    for (std::size_t index = 0; index < mesh.vertices().size(); ++index)
    {
        hasUnwrappedV = hasUnwrappedV || mesh.vertices()[index].parameter.y() > TwoPi;
    }

    context.expect(mesh.isValid(), "Revolution V seam-crossing torus patch mesh valid");
    context.expect(hasUnwrappedV, "Revolution V seam-crossing patch preserves continuous unwrapped V");
    context.expect(triangleWindingMatchesNormals(mesh), "Revolution V seam-crossing patch winding");
}

void testFullVBand(TestContext& context)
{
    const double majorRadius = 5.0;
    const double minorRadius = 1.5;
    const double u0 = 0.3;
    const double u1 = 1.5;
    const MyBRep::Topology_Face face = createFullVBandFace(majorRadius, minorRadius, u0, u1);
    const MyBRep::FaceMesh mesh = MyBRep::RevolvedFaceMesher::mesh(face, testOptions());
    const double expectedArea = torusPatchArea(majorRadius, minorRadius, u0, u1, 0.0, TwoPi);

    context.expect(MyBRep::RevolvedFaceMesher::canMesh(face), "Full V-periodic revolution seam Face can mesh");
    context.expect(mesh.isValid(), "Full V-periodic revolution band mesh valid");
    context.expect(std::fabs(parameterSpan(mesh, 1) - TwoPi) <= 1.0e-8, "Full V-periodic revolution band keeps one complete V period");
    context.expect(std::fabs(meshArea(mesh) - expectedArea) <= AreaTolerance, "Full V-periodic revolution band area");
    context.expect(triangleWindingMatchesNormals(mesh), "Full V-periodic revolution band winding");
}

void testTolerance(TestContext& context)
{
    const MyBRep::Topology_Face face = createPatchFace(5.0, 1.5, 0.0, Pi, -1.2, 1.2);
    MyBRep::RevolvedFaceMeshOptions loose = testOptions();
    MyBRep::RevolvedFaceMeshOptions tight = testOptions();
    loose.surfaceChordTolerance = 0.20;
    tight.surfaceChordTolerance = 0.01;

    const MyBRep::FaceMesh looseMesh = MyBRep::RevolvedFaceMesher::mesh(face, loose);
    const MyBRep::FaceMesh tightMesh = MyBRep::RevolvedFaceMesher::mesh(face, tight);

    context.expect(looseMesh.isValid() && tightMesh.isValid(), "Loose and tight doubly-curved revolution meshes valid");
    context.expect(tightMesh.triangleCount() > looseMesh.triangleCount(), "Tighter torus surface tolerance increases triangle count");
}

}

int main()
{
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep Revolved Face Mesher Curved Profile Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testDoublyCurvedPatch(context);
    testHole(context);
    testVSeamCrossing(context);
    testFullVBand(context);
    testTolerance(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}
