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
#include "MyBRep/Mesh/FaceMesher.h"
#include "MyBRep/Mesh/RevolvedFaceMesher.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Topology/Topology_Builder.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // Revolution轴奇点专项测试统一使用的圆周率。
const double HalfPi = Pi * 0.5;                       // 球形旋转母线的南北轴点V参数。
const double TwoPi = Pi * 2.0;                        // Revolution完整U周期。
const double TestTolerance = 1.0e-8;                 // Topology与Curve-on-Surface连接容差。
const double AreaTolerance = 0.9;                    // 网格面积与解析球面积比较允许的绝对误差。
const double NormalTolerance = 1.0e-6;               // 极限法向比较容差。

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

MyBRep::RevolvedFaceMeshOptions testOptions()
{
    MyBRep::RevolvedFaceMeshOptions options;
    options.boundaryChordTolerance = 0.03;
    options.surfaceChordTolerance = 0.025;
    options.geometricTolerance = 1.0e-10;
    options.minimumBoundarySubdivisionDepth = 1;
    options.maximumBoundarySubdivisionDepth = 12;
    options.maximumSurfaceSubdivisionRounds = 12;
    return options;
}

void addLineCurveOnSurface(MyBRep::Topology_Edge& edge,
                           const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>& surface,
                           const MyMath::Vector2& firstUV,
                           const MyMath::Vector2& lastUV)
{
    const MyMath::Vector2 direction = lastUV - firstUV;
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> curve(
        new MyBRep::Geometry_Line2D(firstUV, direction));

    MyBRep::Topology_Builder::addCurveOnSurface(
        edge, surface, curve, 0.0, direction.length(), TestTolerance);
}

MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> createSphereProfile(double radius)
{
    return MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve>(
        new MyBRep::Geometry_Circle(
            MyMath::Vector3::zero(),
            radius,
            MyMath::Vector3::unitX(),
            MyMath::Vector3::unitZ()));
}

MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> createSphereLikeRevolutionSurface(double radius)
{
    return MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>(
        new MyBRep::Geometry_SurfaceOfRevolution(
            createSphereProfile(radius),
            MyMath::Vector3::zero(),
            MyMath::Vector3::unitZ()));
}

MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> createLatitudeCircle(double radius, double v)
{
    const double ringRadius = radius * std::cos(v);
    const double z = radius * std::sin(v);

    return MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve>(
        new MyBRep::Geometry_Circle(
            MyMath::Vector3(0.0, 0.0, z),
            ringRadius,
            MyMath::Vector3::unitX(),
            MyMath::Vector3::unitY()));
}

MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> createMeridianCircle(double radius, double u)
{
    const MyMath::Vector3 radial(std::cos(u), std::sin(u), 0.0);

    return MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve>(
        new MyBRep::Geometry_Circle(
            MyMath::Vector3::zero(),
            radius,
            radial,
            MyMath::Vector3::unitZ()));
}

MyBRep::Topology_Face createNorthPolePatch(double radius)
{
    const double u0 = 0.2;
    const double u1 = 1.2;
    const double v0 = 0.35;

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface =
        createSphereLikeRevolutionSurface(radius);

    const MyBRep::Topology_Vertex leftVertex(surface->pointAt(u0, v0));
    const MyBRep::Topology_Vertex rightVertex(surface->pointAt(u1, v0));
    const MyBRep::Topology_Vertex poleVertex(surface->pointAt(0.0, HalfPi));

    MyBRep::Topology_Edge bottom(
        leftVertex, rightVertex, createLatitudeCircle(radius, v0), u0, u1, TestTolerance);
    MyBRep::Topology_Edge right(
        rightVertex, poleVertex, createMeridianCircle(radius, u1), v0, HalfPi, TestTolerance);
    MyBRep::Topology_Edge left(
        leftVertex, poleVertex, createMeridianCircle(radius, u0), v0, HalfPi, TestTolerance);

    addLineCurveOnSurface(bottom, surface, MyMath::Vector2(u0, v0), MyMath::Vector2(u1, v0));
    addLineCurveOnSurface(right, surface, MyMath::Vector2(u1, v0), MyMath::Vector2(u1, HalfPi));
    addLineCurveOnSurface(left, surface, MyMath::Vector2(u0, v0), MyMath::Vector2(u0, HalfPi));

    std::vector<MyBRep::Topology_Edge> edges;
    edges.push_back(bottom);
    edges.push_back(right);
    edges.push_back(left.reversed());

    return MyBRep::Modeling::createFace(
        surface, std::vector<MyBRep::Topology_Wire>(1, MyBRep::Topology_Wire(edges)));
}

MyBRep::Topology_Face createSouthPolePatch(double radius)
{
    const double u0 = 0.3;
    const double u1 = 1.3;
    const double v1 = -0.30;

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface =
        createSphereLikeRevolutionSurface(radius);

    const MyBRep::Topology_Vertex poleVertex(surface->pointAt(0.0, -HalfPi));
    const MyBRep::Topology_Vertex leftVertex(surface->pointAt(u0, v1));
    const MyBRep::Topology_Vertex rightVertex(surface->pointAt(u1, v1));

    MyBRep::Topology_Edge left(
        poleVertex, leftVertex, createMeridianCircle(radius, u0), -HalfPi, v1, TestTolerance);
    MyBRep::Topology_Edge top(
        leftVertex, rightVertex, createLatitudeCircle(radius, v1), u0, u1, TestTolerance);
    MyBRep::Topology_Edge right(
        poleVertex, rightVertex, createMeridianCircle(radius, u1), -HalfPi, v1, TestTolerance);

    addLineCurveOnSurface(left, surface, MyMath::Vector2(u0, -HalfPi), MyMath::Vector2(u0, v1));
    addLineCurveOnSurface(top, surface, MyMath::Vector2(u0, v1), MyMath::Vector2(u1, v1));
    addLineCurveOnSurface(right, surface, MyMath::Vector2(u1, -HalfPi), MyMath::Vector2(u1, v1));

    std::vector<MyBRep::Topology_Edge> edges;
    edges.push_back(left);
    edges.push_back(top);
    edges.push_back(right.reversed());

    return MyBRep::Modeling::createFace(
        surface, std::vector<MyBRep::Topology_Wire>(1, MyBRep::Topology_Wire(edges)));
}

MyBRep::Topology_Face createFullSphereLikeRevolutionFace(double radius)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface =
        createSphereLikeRevolutionSurface(radius);

    const MyBRep::Topology_Vertex southVertex(surface->pointAt(0.0, -HalfPi));
    const MyBRep::Topology_Vertex northVertex(surface->pointAt(0.0, HalfPi));

    MyBRep::Topology_Edge seam(
        southVertex, northVertex, createMeridianCircle(radius, 0.0), -HalfPi, HalfPi, TestTolerance);

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> firstSeamCurve(
        new MyBRep::Geometry_Line2D(
            MyMath::Vector2(0.0, -HalfPi),
            MyMath::Vector2(0.0, 1.0)));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> secondSeamCurve(
        new MyBRep::Geometry_Line2D(
            MyMath::Vector2(TwoPi, -HalfPi),
            MyMath::Vector2(0.0, 1.0)));

    MyBRep::Topology_Builder::addCurveOnClosedSurface(
        seam,
        surface,
        firstSeamCurve,
        0.0,
        Pi,
        secondSeamCurve,
        0.0,
        Pi,
        TestTolerance);

    std::vector<MyBRep::Topology_Edge> edges;
    edges.push_back(seam);
    edges.push_back(seam.reversed());

    return MyBRep::Modeling::createFace(
        surface, std::vector<MyBRep::Topology_Wire>(1, MyBRep::Topology_Wire(edges)));
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

std::size_t countVerticesAtPosition(
    const MyBRep::FaceMesh& mesh,
    const MyMath::Vector3& position,
    double tolerance)
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

double parameterUSpan(const MyBRep::FaceMesh& mesh)
{
    if (mesh.vertices().empty())
    {
        return 0.0;
    }

    double minimum = mesh.vertices()[0].parameter.x();
    double maximum = minimum;

    for (std::size_t index = 1; index < mesh.vertices().size(); ++index)
    {
        minimum = (std::min)(minimum, mesh.vertices()[index].parameter.x());
        maximum = (std::max)(maximum, mesh.vertices()[index].parameter.x());
    }

    return maximum - minimum;
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
        const MyMath::Vector3 triangleNormal =
            MyMath::Vector3::cross(second - first, third - first);

        if (!triangleNormal.isVector(0.0))
        {
            return false;
        }

        const MyMath::Vector3 averageNormal =
            (mesh.vertices()[firstIndex].normal +
             mesh.vertices()[secondIndex].normal +
             mesh.vertices()[thirdIndex].normal).normalized(0.0);

        if (MyMath::Vector3::dot(triangleNormal, averageNormal) <= 0.0)
        {
            return false;
        }
    }

    return true;
}

bool sphereLikeNormalsMatch(const MyBRep::Topology_Face& face, const MyBRep::FaceMesh& mesh, double radius)
{
    const MyMath::Vector3 northPole(0.0, 0.0, radius);
    const MyMath::Vector3 southPole(0.0, 0.0, -radius);

    for (std::size_t index = 0; index < mesh.vertices().size(); ++index)
    {
        const MyBRep::FaceMeshVertex& vertex = mesh.vertices()[index];
        MyMath::Vector3 expected;

        if (vertex.position.isEqualTo(northPole, NormalTolerance))
        {
            expected = MyMath::Vector3::unitZ();
        }
        else if (vertex.position.isEqualTo(southPole, NormalTolerance))
        {
            expected = MyMath::Vector3::unitZ() * -1.0;
        }
        else
        {
            expected = face.normalAt(vertex.parameter.x(), vertex.parameter.y());
        }

        if (face.isReversed())
        {
            if (vertex.position.isEqualTo(northPole, NormalTolerance) ||
                vertex.position.isEqualTo(southPole, NormalTolerance))
            {
                expected *= -1.0;
            }
        }

        if (!vertex.normal.isEqualTo(expected, NormalTolerance))
        {
            return false;
        }
    }

    return true;
}

void testNorthPolePatch(TestContext& context)
{
    const double radius = 4.0;
    const double u0 = 0.2;
    const double u1 = 1.2;
    const double v0 = 0.35;
    const MyBRep::Topology_Face face = createNorthPolePatch(radius);
    const MyBRep::FaceMesh mesh = MyBRep::RevolvedFaceMesher::mesh(face, testOptions());
    const double expectedArea = radius * radius * (u1 - u0) * (1.0 - std::sin(v0));

    context.expect(MyBRep::RevolvedFaceMesher::canMesh(face), "North-axis touching Revolution Face can mesh");
    context.expect(mesh.isValid(), "North-axis touching Revolution mesh valid");
    context.expect(countVerticesAtPosition(mesh, MyMath::Vector3(0.0, 0.0, radius), NormalTolerance) >= 2,
                   "North-axis singularity keeps multiple UV limit vertices");
    context.expect(std::fabs(meshArea(mesh) - expectedArea) <= AreaTolerance, "North-axis touching Revolution mesh area");
    context.expect(sphereLikeNormalsMatch(face, mesh, radius), "North-axis touching Revolution uses analytic limiting normals");
    context.expect(triangleWindingMatchesNormals(mesh), "North-axis touching Revolution winding");
}

void testSouthPolePatch(TestContext& context)
{
    const double radius = 4.0;
    const double u0 = 0.3;
    const double u1 = 1.3;
    const double v1 = -0.30;
    const MyBRep::Topology_Face face = createSouthPolePatch(radius);
    const MyBRep::FaceMesh mesh = MyBRep::RevolvedFaceMesher::mesh(face, testOptions());
    const double expectedArea = radius * radius * (u1 - u0) * (std::sin(v1) + 1.0);

    context.expect(MyBRep::RevolvedFaceMesher::canMesh(face), "South-axis touching Revolution Face can mesh");
    context.expect(mesh.isValid(), "South-axis touching Revolution mesh valid");
    context.expect(countVerticesAtPosition(mesh, MyMath::Vector3(0.0, 0.0, -radius), NormalTolerance) >= 2,
                   "South-axis singularity keeps multiple UV limit vertices");
    context.expect(std::fabs(meshArea(mesh) - expectedArea) <= AreaTolerance, "South-axis touching Revolution mesh area");
    context.expect(sphereLikeNormalsMatch(face, mesh, radius), "South-axis touching Revolution uses analytic limiting normals");
    context.expect(triangleWindingMatchesNormals(mesh), "South-axis touching Revolution winding");
}

void testFullSphereLikeRevolution(TestContext& context)
{
    const double radius = 4.0;
    const MyBRep::Topology_Face face = createFullSphereLikeRevolutionFace(radius);

    MyBRep::RevolvedFaceMeshOptions options = testOptions();
    options.boundaryChordTolerance = 0.05;
    options.surfaceChordTolerance = 0.04;

    const MyBRep::FaceMesh mesh = MyBRep::RevolvedFaceMesher::mesh(face, options);
    const double expectedArea = 4.0 * Pi * radius * radius;

    context.expect(MyBRep::RevolvedFaceMesher::canMesh(face), "Full two-pole Revolution Face can mesh");
    context.expect(mesh.isValid(), "Full two-pole Revolution mesh valid");
    context.expect(mesh.triangleCount() > 8, "Full two-pole Revolution receives curvature subdivision");
    context.expect(parameterUSpan(mesh) >= TwoPi - 1.0e-8, "Full two-pole Revolution preserves one complete U period");
    context.expect(countVerticesAtPosition(mesh, MyMath::Vector3(0.0, 0.0, radius), NormalTolerance) >= 2,
                   "Full Revolution keeps north-pole seam-limit vertices");
    context.expect(countVerticesAtPosition(mesh, MyMath::Vector3(0.0, 0.0, -radius), NormalTolerance) >= 2,
                   "Full Revolution keeps south-pole seam-limit vertices");
    context.expect(std::fabs(meshArea(mesh) - expectedArea) <= AreaTolerance, "Full two-pole Revolution mesh area");
    context.expect(sphereLikeNormalsMatch(face, mesh, radius), "Full two-pole Revolution limiting normals");
    context.expect(triangleWindingMatchesNormals(mesh), "Full two-pole Revolution winding");

    MyBRep::FaceMeshOptions dispatchOptions;
    dispatchOptions.revolved = options;
    const MyBRep::FaceMesh dispatched = MyBRep::FaceMesher::mesh(face, dispatchOptions);
    context.expect(dispatched.isValid(), "FaceMesher dispatch accepts full two-pole Revolution Face");
}

void testReversedFullSphereLikeRevolution(TestContext& context)
{
    const double radius = 4.0;
    const MyBRep::Topology_Face forwardFace = createFullSphereLikeRevolutionFace(radius);
    const MyBRep::Topology_Face reversedFace = forwardFace.reversed();
    const MyBRep::FaceMesh forwardMesh = MyBRep::RevolvedFaceMesher::mesh(forwardFace, testOptions());
    const MyBRep::FaceMesh reversedMesh = MyBRep::RevolvedFaceMesher::mesh(reversedFace, testOptions());

    context.expect(forwardMesh.isValid() && reversedMesh.isValid(), "Forward and Reversed two-pole Revolution meshes valid");
    context.expect(std::fabs(meshArea(forwardMesh) - meshArea(reversedMesh)) <= 1.0e-8,
                   "Reversed two-pole Revolution preserves area");
    context.expect(sphereLikeNormalsMatch(reversedFace, reversedMesh, radius),
                   "Reversed two-pole Revolution limiting normals follow Face orientation");
    context.expect(triangleWindingMatchesNormals(reversedMesh),
                   "Reversed two-pole Revolution winding follows reversed normal");
}

}

int main()
{
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep Revolved Face Mesher Axis Singularity v2 Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    context.expect(testOptions().isValid(), "Default v2 Revolution mesh options valid");
    testNorthPolePatch(context);
    testSouthPolePatch(context);
    testFullSphereLikeRevolution(context);
    testReversedFullSphereLikeRevolution(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}