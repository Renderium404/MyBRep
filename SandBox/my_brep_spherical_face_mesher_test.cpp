#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "MyMath/CoordinateSystem.h"
#include "MyMath/Vector2.h"
#include "MyMath/Vector3.h"

#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Curve/Geometry_Circle.h"
#include "MyBRep/Geometry/Curve/Geometry_Curve.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Curve2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Line2D.h"
#include "MyBRep/Geometry/Surface/Geometry_CylindricalSurface.h"
#include "MyBRep/Geometry/Surface/Geometry_SphericalSurface.h"
#include "MyBRep/Mesh/SphericalFaceMesher.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Topology/Topology_Builder.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 球面测试统一使用的圆周率。
const double HalfPi = Pi * 0.5;                       // 球面南北极对应的V参数绝对值。
const double TwoPi = Pi * 2.0;                        // 球面U参数完整周期。
const double TestTolerance = 1.0e-8;                 // Topology与Curve-on-Surface连接验证统一使用的三维容差。
const double AreaTolerance = 0.8;                    // 三角网格面积与解析球面片面积比较允许的绝对误差。
const double NormalTolerance = 1.0e-6;               // FaceMesh顶点法向与解析Face法向比较容差。

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

MyBRep::SphericalFaceMeshOptions testOptions()
{
    MyBRep::SphericalFaceMeshOptions options;
    options.boundaryChordTolerance = 0.02;
    options.surfaceChordTolerance = 0.02;
    options.geometricTolerance = 1.0e-10;
    options.minimumBoundarySubdivisionDepth = 1;
    options.maximumBoundarySubdivisionDepth = 12;
    options.maximumSurfaceSubdivisionRounds = 12;
    return options;
}

MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> createSphereSurface(const MyMath::CoordinateSystem& coordinateSystem, double radius)
{
    return MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>(new MyBRep::Geometry_SphericalSurface(coordinateSystem, radius));
}

void addLineCurveOnSurface(MyBRep::Topology_Edge& edge, const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>& surface, const MyMath::Vector2& firstUV, const MyMath::Vector2& lastUV)
{
    const MyMath::Vector2 direction = lastUV - firstUV;
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> curve(new MyBRep::Geometry_Line2D(firstUV, direction));
    MyBRep::Topology_Builder::addCurveOnSurface(edge, surface, curve, 0.0, direction.length(), TestTolerance);
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

MyBRep::Topology_Wire createSpherePatchWire(const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>& surface, double u0, double u1, double v0, double v1)
{
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
    return MyBRep::Topology_Wire(edges);
}

MyBRep::Topology_Face createSpherePatchFace(const MyMath::CoordinateSystem& coordinateSystem, double radius, double u0, double u1, double v0, double v1)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface = createSphereSurface(coordinateSystem, radius);
    return MyBRep::Modeling::createFace(surface, std::vector<MyBRep::Topology_Wire>(1, createSpherePatchWire(surface, u0, u1, v0, v1)));
}

MyBRep::Topology_Face createSphereFaceWithHole(const MyMath::CoordinateSystem& coordinateSystem, double radius)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface = createSphereSurface(coordinateSystem, radius);

    std::vector<MyBRep::Topology_Wire> wires;
    wires.push_back(createSpherePatchWire(surface, 0.0, 2.0, -0.6, 0.6));
    wires.push_back(createSpherePatchWire(surface, 0.7, 1.3, -0.25, 0.25));
    return MyBRep::Modeling::createFace(surface, wires);
}

MyBRep::Topology_Face createFullSphereBandFace(const MyMath::CoordinateSystem& coordinateSystem, double radius, double v0, double v1)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface = createSphereSurface(coordinateSystem, radius);
    const MyBRep::Geometry_SphericalSurface& sphere = static_cast<const MyBRep::Geometry_SphericalSurface&>(*surface);

    const MyBRep::Topology_Vertex bottomVertex(sphere.pointAt(0.0, v0));
    const MyBRep::Topology_Vertex topVertex(sphere.pointAt(0.0, v1));

    MyBRep::Topology_Edge bottom(bottomVertex, bottomVertex, createLatitudeCircle(sphere, v0), 0.0, TwoPi, TestTolerance);
    MyBRep::Topology_Edge top(topVertex, topVertex, createLatitudeCircle(sphere, v1), 0.0, TwoPi, TestTolerance);
    MyBRep::Topology_Edge seam(bottomVertex, topVertex, createMeridianCircle(sphere, 0.0), v0, v1, TestTolerance);

    addLineCurveOnSurface(bottom, surface, MyMath::Vector2(0.0, v0), MyMath::Vector2(TwoPi, v0));
    addLineCurveOnSurface(top, surface, MyMath::Vector2(0.0, v1), MyMath::Vector2(TwoPi, v1));

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> firstSeamCurve(new MyBRep::Geometry_Line2D(MyMath::Vector2(0.0, v0), MyMath::Vector2(0.0, 1.0)));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> secondSeamCurve(new MyBRep::Geometry_Line2D(MyMath::Vector2(TwoPi, v0), MyMath::Vector2(0.0, 1.0)));

    MyBRep::Topology_Builder::addCurveOnClosedSurface(seam, surface, firstSeamCurve, 0.0, v1 - v0, secondSeamCurve, 0.0, v1 - v0, TestTolerance);

    std::vector<MyBRep::Topology_Edge> edges;
    edges.push_back(bottom.reversed());
    edges.push_back(seam);
    edges.push_back(top);
    edges.push_back(seam.reversed());

    return MyBRep::Modeling::createFace(surface, std::vector<MyBRep::Topology_Wire>(1, MyBRep::Topology_Wire(edges)));
}

MyBRep::Topology_Face createNorthPoleTouchingFace(double radius)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface = createSphereSurface(MyMath::CoordinateSystem::identity(), radius);
    const MyBRep::Geometry_SphericalSurface& sphere = static_cast<const MyBRep::Geometry_SphericalSurface&>(*surface);

    const double u0 = 0.2;
    const double u1 = 1.1;
    const double v0 = 0.4;

    const MyBRep::Topology_Vertex leftVertex(sphere.pointAt(u0, v0));
    const MyBRep::Topology_Vertex rightVertex(sphere.pointAt(u1, v0));
    const MyBRep::Topology_Vertex poleVertex(sphere.pointAt(0.0, HalfPi));

    MyBRep::Topology_Edge bottom(leftVertex, rightVertex, createLatitudeCircle(sphere, v0), u0, u1, TestTolerance);
    MyBRep::Topology_Edge right(rightVertex, poleVertex, createMeridianCircle(sphere, u1), v0, HalfPi, TestTolerance);
    MyBRep::Topology_Edge left(leftVertex, poleVertex, createMeridianCircle(sphere, u0), v0, HalfPi, TestTolerance);

    addLineCurveOnSurface(bottom, surface, MyMath::Vector2(u0, v0), MyMath::Vector2(u1, v0));
    addLineCurveOnSurface(right, surface, MyMath::Vector2(u1, v0), MyMath::Vector2(u1, HalfPi));
    addLineCurveOnSurface(left, surface, MyMath::Vector2(u0, v0), MyMath::Vector2(u0, HalfPi));

    std::vector<MyBRep::Topology_Edge> edges;
    edges.push_back(bottom);
    edges.push_back(right);
    edges.push_back(left.reversed());

    return MyBRep::Modeling::createFace(surface, std::vector<MyBRep::Topology_Wire>(1, MyBRep::Topology_Wire(edges)));
}


MyBRep::Topology_Face createSouthPoleTouchingFace(double radius)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface = createSphereSurface(MyMath::CoordinateSystem::identity(), radius);
    const MyBRep::Geometry_SphericalSurface& sphere = static_cast<const MyBRep::Geometry_SphericalSurface&>(*surface);

    const double u0 = 0.3;
    const double u1 = 1.2;
    const double v1 = -0.35;

    const MyBRep::Topology_Vertex poleVertex(sphere.pointAt(0.0, -HalfPi));
    const MyBRep::Topology_Vertex leftVertex(sphere.pointAt(u0, v1));
    const MyBRep::Topology_Vertex rightVertex(sphere.pointAt(u1, v1));

    MyBRep::Topology_Edge left(poleVertex, leftVertex, createMeridianCircle(sphere, u0), -HalfPi, v1, TestTolerance);
    MyBRep::Topology_Edge top(leftVertex, rightVertex, createLatitudeCircle(sphere, v1), u0, u1, TestTolerance);
    MyBRep::Topology_Edge right(poleVertex, rightVertex, createMeridianCircle(sphere, u1), -HalfPi, v1, TestTolerance);

    addLineCurveOnSurface(left, surface, MyMath::Vector2(u0, -HalfPi), MyMath::Vector2(u0, v1));
    addLineCurveOnSurface(top, surface, MyMath::Vector2(u0, v1), MyMath::Vector2(u1, v1));
    addLineCurveOnSurface(right, surface, MyMath::Vector2(u1, -HalfPi), MyMath::Vector2(u1, v1));

    std::vector<MyBRep::Topology_Edge> edges;
    edges.push_back(left);
    edges.push_back(top);
    edges.push_back(right.reversed());

    return MyBRep::Modeling::createFace(surface, std::vector<MyBRep::Topology_Wire>(1, MyBRep::Topology_Wire(edges)));
}

MyBRep::Topology_Face createFullSphereFace(double radius)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface = createSphereSurface(MyMath::CoordinateSystem::identity(), radius);
    const MyBRep::Geometry_SphericalSurface& sphere = static_cast<const MyBRep::Geometry_SphericalSurface&>(*surface);

    const MyBRep::Topology_Vertex southVertex(sphere.pointAt(0.0, -HalfPi));
    const MyBRep::Topology_Vertex northVertex(sphere.pointAt(0.0, HalfPi));

    MyBRep::Topology_Edge seam(southVertex, northVertex, createMeridianCircle(sphere, 0.0), -HalfPi, HalfPi, TestTolerance);

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> firstSeamCurve(new MyBRep::Geometry_Line2D(MyMath::Vector2(0.0, -HalfPi), MyMath::Vector2(0.0, 1.0)));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> secondSeamCurve(new MyBRep::Geometry_Line2D(MyMath::Vector2(TwoPi, -HalfPi), MyMath::Vector2(0.0, 1.0)));

    MyBRep::Topology_Builder::addCurveOnClosedSurface(seam, surface, firstSeamCurve, 0.0, Pi, secondSeamCurve, 0.0, Pi, TestTolerance);

    std::vector<MyBRep::Topology_Edge> edges;
    edges.push_back(seam);
    edges.push_back(seam.reversed());

    return MyBRep::Modeling::createFace(surface, std::vector<MyBRep::Topology_Wire>(1, MyBRep::Topology_Wire(edges)));
}

std::size_t countVerticesAtPosition(const MyBRep::FaceMesh& mesh, const MyMath::Vector3& position, double tolerance)
{
    std::size_t count = 0;

    for (std::size_t index = 0; index < mesh.vertices().size(); ++index)
    {
        if (mesh.vertices()[index].position.isEqualTo(position, tolerance))
        {
            ++count;
        }
    }

    return count;
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
        const unsigned int firstIndex = mesh.indices()[index];
        const unsigned int secondIndex = mesh.indices()[index + 1];
        const unsigned int thirdIndex = mesh.indices()[index + 2];

        const MyMath::Vector3& first = mesh.vertices()[firstIndex].position;
        const MyMath::Vector3& second = mesh.vertices()[secondIndex].position;
        const MyMath::Vector3& third = mesh.vertices()[thirdIndex].position;
        const MyMath::Vector3 triangleNormal = MyMath::Vector3::cross(second - first, third - first);

        if (!triangleNormal.isVector(0.0))
        {
            return false;
        }

        const MyMath::Vector3 averageNormal = (mesh.vertices()[firstIndex].normal + mesh.vertices()[secondIndex].normal + mesh.vertices()[thirdIndex].normal).normalized(0.0);

        if (MyMath::Vector3::dot(triangleNormal, averageNormal) <= 0.0)
        {
            return false;
        }
    }

    return true;
}

bool vertexNormalsMatchFace(const MyBRep::Topology_Face& face, const MyBRep::FaceMesh& mesh)
{
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

void testDefaultOptions(TestContext& context)
{
    const MyBRep::SphericalFaceMeshOptions options;
    context.expect(options.isValid(), "Default spherical mesh options valid");
}

void testSpherePatch(TestContext& context)
{
    const double radius = 5.0;
    const double u0 = 0.0;
    const double u1 = Pi * 0.5;
    const double v0 = -0.4;
    const double v1 = 0.5;

    const MyBRep::Topology_Face face = createSpherePatchFace(MyMath::CoordinateSystem::identity(), radius, u0, u1, v0, v1);
    const MyBRep::FaceMesh mesh = MyBRep::SphericalFaceMesher::mesh(face, testOptions());
    const double expectedArea = radius * radius * (u1 - u0) * (std::sin(v1) - std::sin(v0));

    context.expect(MyBRep::SphericalFaceMesher::canMesh(face), "Spherical patch Face can mesh");
    context.expect(mesh.isValid(), "Spherical patch mesh valid");
    context.expect(mesh.triangleCount() > 2, "Spherical patch receives curvature subdivision");
    context.expect(std::fabs(meshArea(mesh) - expectedArea) <= AreaTolerance, "Spherical patch mesh area");
    context.expect(triangleWindingMatchesNormals(mesh), "Spherical patch triangle winding matches normals");
    context.expect(vertexNormalsMatchFace(face, mesh), "Spherical patch vertex normals match Face normals");
}

void testSphereHole(TestContext& context)
{
    const double radius = 4.0;
    const MyBRep::Topology_Face face = createSphereFaceWithHole(MyMath::CoordinateSystem::identity(), radius);
    const MyBRep::FaceMesh mesh = MyBRep::SphericalFaceMesher::mesh(face, testOptions());

    const double outerArea = radius * radius * 2.0 * (std::sin(0.6) - std::sin(-0.6));
    const double holeArea = radius * radius * 0.6 * (std::sin(0.25) - std::sin(-0.25));
    const double expectedArea = outerArea - holeArea;

    context.expect(mesh.isValid(), "Spherical Face with hole mesh valid");
    context.expect(std::fabs(meshArea(mesh) - expectedArea) <= AreaTolerance, "Spherical Face with hole area");
    context.expect(triangleWindingMatchesNormals(mesh), "Spherical Face with hole winding");
}

void testSeamCrossingPatch(TestContext& context)
{
    const MyBRep::Topology_Face face = createSpherePatchFace(MyMath::CoordinateSystem::identity(), 3.5, 5.7, 6.7, -0.45, 0.45);
    const MyBRep::FaceMesh mesh = MyBRep::SphericalFaceMesher::mesh(face, testOptions());

    bool containsBeyondTwoPi = false;

    for (std::size_t index = 0; index < mesh.vertices().size(); ++index)
    {
        if (mesh.vertices()[index].parameter.x() > TwoPi)
        {
            containsBeyondTwoPi = true;
            break;
        }
    }

    context.expect(mesh.isValid(), "Spherical seam-crossing patch mesh valid");
    context.expect(containsBeyondTwoPi, "Spherical seam-crossing patch preserves continuous unwrapped U");
    context.expect(triangleWindingMatchesNormals(mesh), "Spherical seam-crossing patch winding");
}

void testFullSphereBand(TestContext& context)
{
    const double radius = 3.0;
    const double v0 = -0.6;
    const double v1 = 0.6;

    const MyBRep::Topology_Face face = createFullSphereBandFace(MyMath::CoordinateSystem::identity(), radius, v0, v1);
    MyBRep::SphericalFaceMeshOptions options = testOptions();
    options.boundaryChordTolerance = 0.03;
    options.surfaceChordTolerance = 0.02;

    const MyBRep::FaceMesh mesh = MyBRep::SphericalFaceMesher::mesh(face, options);
    const double expectedArea = TwoPi * radius * radius * (std::sin(v1) - std::sin(v0));

    double minimumU = 0.0;
    double maximumU = 0.0;

    if (!mesh.vertices().empty())
    {
        minimumU = mesh.vertices()[0].parameter.x();
        maximumU = minimumU;

        for (std::size_t index = 1; index < mesh.vertices().size(); ++index)
        {
            minimumU = (std::min)(minimumU, mesh.vertices()[index].parameter.x());
            maximumU = (std::max)(maximumU, mesh.vertices()[index].parameter.x());
        }
    }

    context.expect(MyBRep::SphericalFaceMesher::canMesh(face), "Full spherical seam Face can mesh");
    context.expect(mesh.isValid(), "Full spherical band mesh valid");
    context.expect(maximumU - minimumU >= TwoPi - 1.0e-8, "Full spherical band keeps one complete U period");
    context.expect(std::fabs(meshArea(mesh) - expectedArea) <= AreaTolerance, "Full spherical band mesh area");
    context.expect(triangleWindingMatchesNormals(mesh), "Full spherical band winding");
}

void testSurfaceToleranceRefinement(TestContext& context)
{
    const MyBRep::Topology_Face face = createSpherePatchFace(MyMath::CoordinateSystem::identity(), 6.0, 0.0, Pi, -0.7, 0.7);

    MyBRep::SphericalFaceMeshOptions loose = testOptions();
    MyBRep::SphericalFaceMeshOptions tight = testOptions();
    loose.boundaryChordTolerance = 0.05;
    tight.boundaryChordTolerance = 0.05;
    loose.surfaceChordTolerance = 0.20;
    tight.surfaceChordTolerance = 0.01;

    const MyBRep::FaceMesh looseMesh = MyBRep::SphericalFaceMesher::mesh(face, loose);
    const MyBRep::FaceMesh tightMesh = MyBRep::SphericalFaceMesher::mesh(face, tight);

    context.expect(looseMesh.isValid() && tightMesh.isValid(), "Loose and tight spherical meshes valid");
    context.expect(tightMesh.triangleCount() > looseMesh.triangleCount(), "Tighter spherical surface tolerance increases triangle count");
}

void testReversedFace(TestContext& context)
{
    const MyBRep::Topology_Face forwardFace = createSpherePatchFace(MyMath::CoordinateSystem::identity(), 5.0, 0.0, Pi * 0.75, -0.5, 0.55);
    const MyBRep::Topology_Face reversedFace = forwardFace.reversed();

    const MyBRep::FaceMesh forwardMesh = MyBRep::SphericalFaceMesher::mesh(forwardFace, testOptions());
    const MyBRep::FaceMesh reversedMesh = MyBRep::SphericalFaceMesher::mesh(reversedFace, testOptions());

    context.expect(forwardMesh.isValid() && reversedMesh.isValid(), "Forward and Reversed spherical Face meshes valid");
    context.expect(std::fabs(meshArea(forwardMesh) - meshArea(reversedMesh)) <= 1.0e-8, "Reversed spherical Face preserves mesh area");

    bool normalsOpposite = false;

    if (!forwardMesh.vertices().empty() && !reversedMesh.vertices().empty())
    {
        const MyBRep::FaceMeshVertex& forwardVertex = forwardMesh.vertices()[0];

        for (std::size_t index = 0; index < reversedMesh.vertices().size(); ++index)
        {
            const MyBRep::FaceMeshVertex& reversedVertex = reversedMesh.vertices()[index];

            if (forwardVertex.position.isEqualTo(reversedVertex.position, NormalTolerance))
            {
                normalsOpposite = MyMath::Vector3::dot(forwardVertex.normal, reversedVertex.normal) < -0.999999;
                break;
            }
        }
    }

    context.expect(normalsOpposite, "Reversed spherical Face normals are opposite at the same surface point");
    context.expect(vertexNormalsMatchFace(reversedFace, reversedMesh), "Reversed spherical Face vertex normals match reversed Face");
    context.expect(triangleWindingMatchesNormals(reversedMesh), "Reversed spherical Face winding follows reversed normal");
}

void testOrientedSphere(TestContext& context)
{
    const MyMath::CoordinateSystem coordinateSystem = MyMath::CoordinateSystem::fromAxes(MyMath::Vector3(3.0, 4.0, 5.0), MyMath::Vector3::unitY(), MyMath::Vector3::unitZ(), MyMath::Vector3::unitX());
    const MyBRep::Topology_Face face = createSpherePatchFace(coordinateSystem, 4.0, 0.2, 1.4, -0.45, 0.55);
    const MyBRep::FaceMesh mesh = MyBRep::SphericalFaceMesher::mesh(face, testOptions());

    context.expect(mesh.isValid(), "Oriented spherical Face mesh valid");
    context.expect(vertexNormalsMatchFace(face, mesh), "Oriented spherical normals match Face");
    context.expect(triangleWindingMatchesNormals(mesh), "Oriented spherical winding matches normal");
}

void testNorthPoleSupport(TestContext& context)
{
    const double radius = 5.0;
    const double u0 = 0.2;
    const double u1 = 1.1;
    const double v0 = 0.4;
    const MyBRep::Topology_Face face = createNorthPoleTouchingFace(radius);
    const MyBRep::FaceMesh mesh = MyBRep::SphericalFaceMesher::mesh(face, testOptions());
    const MyMath::Vector3 northPole = MyMath::Vector3::unitZ() * radius;
    const double expectedArea = radius * radius * (u1 - u0) * (1.0 - std::sin(v0));

    context.expect(MyBRep::SphericalFaceMesher::canMesh(face), "North-pole touching Face can mesh");
    context.expect(mesh.isValid(), "North-pole touching Face mesh valid");
    context.expect(countVerticesAtPosition(mesh, northPole, NormalTolerance) == 1, "North-pole UV vertices collapse to one 3D vertex");
    context.expect(std::fabs(meshArea(mesh) - expectedArea) <= AreaTolerance, "North-pole touching Face mesh area");
    context.expect(vertexNormalsMatchFace(face, mesh), "North-pole touching Face normals match Face");
    context.expect(triangleWindingMatchesNormals(mesh), "North-pole touching Face winding");
}

void testSouthPoleSupport(TestContext& context)
{
    const double radius = 5.0;
    const double u0 = 0.3;
    const double u1 = 1.2;
    const double v1 = -0.35;
    const MyBRep::Topology_Face face = createSouthPoleTouchingFace(radius);
    const MyBRep::FaceMesh mesh = MyBRep::SphericalFaceMesher::mesh(face, testOptions());
    const MyMath::Vector3 southPole = MyMath::Vector3::unitZ() * -radius;
    const double expectedArea = radius * radius * (u1 - u0) * (std::sin(v1) + 1.0);

    context.expect(MyBRep::SphericalFaceMesher::canMesh(face), "South-pole touching Face can mesh");
    context.expect(mesh.isValid(), "South-pole touching Face mesh valid");
    context.expect(countVerticesAtPosition(mesh, southPole, NormalTolerance) == 1, "South-pole UV vertices collapse to one 3D vertex");
    context.expect(std::fabs(meshArea(mesh) - expectedArea) <= AreaTolerance, "South-pole touching Face mesh area");
    context.expect(triangleWindingMatchesNormals(mesh), "South-pole touching Face winding");
}

void testFullSphereSupport(TestContext& context)
{
    const double radius = 4.0;
    const MyBRep::Topology_Face face = createFullSphereFace(radius);

    MyBRep::SphericalFaceMeshOptions options = testOptions();
    options.boundaryChordTolerance = 0.05;
    options.surfaceChordTolerance = 0.02;

    const MyBRep::FaceMesh mesh = MyBRep::SphericalFaceMesher::mesh(face, options);
    const double expectedArea = 4.0 * Pi * radius * radius;
    const MyMath::Vector3 northPole = MyMath::Vector3::unitZ() * radius;
    const MyMath::Vector3 southPole = MyMath::Vector3::unitZ() * -radius;
    double minimumU = 0.0;
    double maximumU = 0.0;

    if (!mesh.vertices().empty())
    {
        minimumU = mesh.vertices()[0].parameter.x();
        maximumU = minimumU;

        for (std::size_t index = 1; index < mesh.vertices().size(); ++index)
        {
            minimumU = (std::min)(minimumU, mesh.vertices()[index].parameter.x());
            maximumU = (std::max)(maximumU, mesh.vertices()[index].parameter.x());
        }
    }

    context.expect(MyBRep::SphericalFaceMesher::canMesh(face), "Full spherical Face can mesh");
    context.expect(mesh.isValid(), "Full spherical Face mesh valid");
    context.expect(mesh.triangleCount() > 8, "Full spherical Face receives two-parameter curvature subdivision");
    context.expect(maximumU - minimumU >= TwoPi - 1.0e-8, "Full sphere preserves one complete unwrapped U period");
    context.expect(countVerticesAtPosition(mesh, northPole, NormalTolerance) == 1, "Full sphere keeps one north-pole vertex");
    context.expect(countVerticesAtPosition(mesh, southPole, NormalTolerance) == 1, "Full sphere keeps one south-pole vertex");
    context.expect(std::fabs(meshArea(mesh) - expectedArea) <= AreaTolerance, "Full spherical Face mesh area");
    context.expect(vertexNormalsMatchFace(face, mesh), "Full spherical Face normals match Face");
    context.expect(triangleWindingMatchesNormals(mesh), "Full spherical Face winding");
}

void testFullSphereReversed(TestContext& context)
{
    const MyBRep::Topology_Face forwardFace = createFullSphereFace(4.0);
    const MyBRep::Topology_Face reversedFace = forwardFace.reversed();
    const MyBRep::FaceMesh forwardMesh = MyBRep::SphericalFaceMesher::mesh(forwardFace, testOptions());
    const MyBRep::FaceMesh reversedMesh = MyBRep::SphericalFaceMesher::mesh(reversedFace, testOptions());

    context.expect(forwardMesh.isValid() && reversedMesh.isValid(), "Forward and Reversed full sphere meshes valid");
    context.expect(std::fabs(meshArea(forwardMesh) - meshArea(reversedMesh)) <= 1.0e-8, "Reversed full sphere preserves mesh area");
    context.expect(vertexNormalsMatchFace(reversedFace, reversedMesh), "Reversed full sphere normals match reversed Face");
    context.expect(triangleWindingMatchesNormals(reversedMesh), "Reversed full sphere winding follows reversed normal");
}

void testUnsupportedCylinder(TestContext& context)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface(new MyBRep::Geometry_CylindricalSurface(MyMath::Vector3::zero(), 5.0));
    const MyBRep::Topology_Face face(surface);

    context.expect(!MyBRep::SphericalFaceMesher::canMesh(face), "Cylinder Face cannot use spherical mesher");
    context.expect(MyBRep::SphericalFaceMesher::mesh(face).isEmpty(), "Cylinder Face returns empty spherical mesh");
}

}

int main()
{
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep Spherical Face Mesher Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testDefaultOptions(context);
    testSpherePatch(context);
    testSphereHole(context);
    testSeamCrossingPatch(context);
    testFullSphereBand(context);
    testSurfaceToleranceRefinement(context);
    testReversedFace(context);
    testOrientedSphere(context);
    testNorthPoleSupport(context);
    testSouthPoleSupport(context);
    testFullSphereSupport(context);
    testFullSphereReversed(context);
    testUnsupportedCylinder(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}
