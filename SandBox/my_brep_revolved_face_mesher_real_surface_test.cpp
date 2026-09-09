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
#include "MyBRep/Geometry/Curve/Geometry_Line.h"
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

const double Pi = 3.1415926535897932384626433832795; // 专项测试统一使用的圆周率。
const double TwoPi = Pi * 2.0;                        // Revolution完整旋转周期。
const double TestTolerance = 1.0e-8;                 // Topology与Curve-on-Surface连接统一使用的几何容差。
const double AreaTolerance = 0.35;                   // 离散面积与解析面积比较允许的绝对误差。
const double NormalTolerance = 1.0e-6;               // FaceMesh法向与解析Face法向比较容差。

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

        if (!triangleNormal.isVector(0.0))
        {
            return false;
        }

        const MyMath::Vector3 averageNormal = first.normal + second.normal + third.normal;

        if (MyMath::Vector3::dot(triangleNormal, averageNormal) <= 0.0)
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

bool normalsOppositeAtSharedPositions(const MyBRep::FaceMesh& forwardMesh, const MyBRep::FaceMesh& reversedMesh)
{
    if (!forwardMesh.isValid() || !reversedMesh.isValid())
    {
        return false;
    }

    std::size_t sharedVertexCount = 0;

    for (std::size_t forwardIndex = 0; forwardIndex < forwardMesh.vertices().size(); ++forwardIndex)
    {
        const MyBRep::FaceMeshVertex& forwardVertex = forwardMesh.vertices()[forwardIndex];

        for (std::size_t reversedIndex = 0; reversedIndex < reversedMesh.vertices().size(); ++reversedIndex)
        {
            const MyBRep::FaceMeshVertex& reversedVertex = reversedMesh.vertices()[reversedIndex];

            if (!forwardVertex.position.isEqualTo(reversedVertex.position, NormalTolerance))
            {
                continue;
            }

            if (!forwardVertex.normal.isEqualTo(reversedVertex.normal * -1.0, NormalTolerance))
            {
                return false;
            }

            ++sharedVertexCount;
            break;
        }
    }

    // 正反向三角化允许选择不同内部对角线和细分点，但四个原始边界角点必须共同存在。
    return sharedVertexCount >= 4;
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

MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> createRevolutionSurface(double radius)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> profile(
        new MyBRep::Geometry_Line(MyMath::Vector3(radius, 0.0, 0.0), MyMath::Vector3::unitZ()));

    return MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>(
        new MyBRep::Geometry_SurfaceOfRevolution(profile, MyMath::Vector3::zero(), MyMath::Vector3::unitZ()));
}

MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> createRingCurve(double radius, double v)
{
    return MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve>(
        new MyBRep::Geometry_Circle(MyMath::Vector3(0.0, 0.0, v), radius, MyMath::Vector3::unitX(), MyMath::Vector3::unitY()));
}

MyBRep::Topology_Wire createRevolutionPatchWire(const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>& surface,
                                                double radius,
                                                double u0,
                                                double u1,
                                                double v0,
                                                double v1)
{
    const MyBRep::Topology_Vertex v00(surface->pointAt(u0, v0));
    const MyBRep::Topology_Vertex v10(surface->pointAt(u1, v0));
    const MyBRep::Topology_Vertex v11(surface->pointAt(u1, v1));
    const MyBRep::Topology_Vertex v01(surface->pointAt(u0, v1));

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> rightGeometry(
        new MyBRep::Geometry_Line(v10.point(), v11.point() - v10.point()));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> leftGeometry(
        new MyBRep::Geometry_Line(v00.point(), v01.point() - v00.point()));

    MyBRep::Topology_Edge bottom(v00, v10, createRingCurve(radius, v0), u0, u1, TestTolerance);
    MyBRep::Topology_Edge right(v10, v11, rightGeometry, 0.0, (v11.point() - v10.point()).length(), TestTolerance);
    MyBRep::Topology_Edge top(v01, v11, createRingCurve(radius, v1), u0, u1, TestTolerance);
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
    return MyBRep::Topology_Wire(edges);
}

MyBRep::Topology_Face createRevolutionPatchFace(double radius, double u0, double u1, double v0, double v1)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface = createRevolutionSurface(radius);
    return MyBRep::Modeling::createFace(surface, std::vector<MyBRep::Topology_Wire>(1, createRevolutionPatchWire(surface, radius, u0, u1, v0, v1)));
}

MyBRep::Topology_Face createRevolutionFaceWithHole(double radius)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface = createRevolutionSurface(radius);
    std::vector<MyBRep::Topology_Wire> wires;
    wires.push_back(createRevolutionPatchWire(surface, radius, 0.0, 2.4, -2.0, 2.0));
    wires.push_back(createRevolutionPatchWire(surface, radius, 0.8, 1.5, -0.7, 0.7));
    return MyBRep::Modeling::createFace(surface, wires);
}

MyBRep::Topology_Face createFullRevolutionBandFace(double radius, double v0, double v1)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface = createRevolutionSurface(radius);

    const MyBRep::Topology_Vertex bottomVertex(surface->pointAt(0.0, v0));
    const MyBRep::Topology_Vertex topVertex(surface->pointAt(0.0, v1));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> seamGeometry(
        new MyBRep::Geometry_Line(bottomVertex.point(), topVertex.point() - bottomVertex.point()));

    MyBRep::Topology_Edge bottom(bottomVertex, bottomVertex, createRingCurve(radius, v0), 0.0, TwoPi, TestTolerance);
    MyBRep::Topology_Edge top(topVertex, topVertex, createRingCurve(radius, v1), 0.0, TwoPi, TestTolerance);
    MyBRep::Topology_Edge seam(bottomVertex, topVertex, seamGeometry, 0.0, (topVertex.point() - bottomVertex.point()).length(), TestTolerance);

    addLineCurveOnSurface(bottom, surface, MyMath::Vector2(0.0, v0), MyMath::Vector2(TwoPi, v0));
    addLineCurveOnSurface(top, surface, MyMath::Vector2(0.0, v1), MyMath::Vector2(TwoPi, v1));

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> firstSeamCurve(
        new MyBRep::Geometry_Line2D(MyMath::Vector2(0.0, v0), MyMath::Vector2(0.0, 1.0)));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> secondSeamCurve(
        new MyBRep::Geometry_Line2D(MyMath::Vector2(TwoPi, v0), MyMath::Vector2(0.0, 1.0)));

    MyBRep::Topology_Builder::addCurveOnClosedSurface(
        seam, surface, firstSeamCurve, 0.0, v1 - v0, secondSeamCurve, 0.0, v1 - v0, TestTolerance);

    std::vector<MyBRep::Topology_Edge> edges;
    edges.push_back(bottom.reversed());
    edges.push_back(seam);
    edges.push_back(top);
    edges.push_back(seam.reversed());

    return MyBRep::Modeling::createFace(surface, std::vector<MyBRep::Topology_Wire>(1, MyBRep::Topology_Wire(edges)));
}

double revolutionArea(double radius, double u0, double u1, double v0, double v1)
{
    return radius * (u1 - u0) * (v1 - v0);
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

void testPatch(TestContext& context)
{
    const double radius = 3.0;
    const double u0 = 0.2;
    const double u1 = 1.7;
    const double v0 = -1.5;
    const double v1 = 2.0;
    const MyBRep::Topology_Face face = createRevolutionPatchFace(radius, u0, u1, v0, v1);
    const MyBRep::FaceMesh mesh = MyBRep::RevolvedFaceMesher::mesh(face, testOptions());

    context.expect(MyBRep::RevolvedFaceMesher::canMesh(face), "Revolution patch Face can mesh");
    context.expect(mesh.isValid(), "Revolution patch mesh valid");
    context.expect(mesh.triangleCount() > 2, "Revolution patch receives curvature subdivision");
    context.expect(std::fabs(meshArea(mesh) - revolutionArea(radius, u0, u1, v0, v1)) <= AreaTolerance, "Revolution patch mesh area");
    context.expect(vertexNormalsMatchFace(face, mesh), "Revolution patch vertex normals match Face");
    context.expect(triangleWindingMatchesNormals(mesh), "Revolution patch winding matches normals");
}

void testHole(TestContext& context)
{
    const double radius = 3.0;
    const MyBRep::Topology_Face face = createRevolutionFaceWithHole(radius);
    const MyBRep::FaceMesh mesh = MyBRep::RevolvedFaceMesher::mesh(face, testOptions());
    const double expectedArea = revolutionArea(radius, 0.0, 2.4, -2.0, 2.0) - revolutionArea(radius, 0.8, 1.5, -0.7, 0.7);

    context.expect(mesh.isValid(), "Revolution Face with hole mesh valid");
    context.expect(std::fabs(meshArea(mesh) - expectedArea) <= AreaTolerance, "Revolution Face with hole area");
    context.expect(triangleWindingMatchesNormals(mesh), "Revolution Face with hole winding");
}

void testSeamCrossing(TestContext& context)
{
    const MyBRep::Topology_Face face = createRevolutionPatchFace(3.0, 5.6, 6.9, -1.0, 1.5);
    const MyBRep::FaceMesh mesh = MyBRep::RevolvedFaceMesher::mesh(face, testOptions());

    bool hasUnwrappedU = false;

    for (std::size_t index = 0; index < mesh.vertices().size(); ++index)
    {
        hasUnwrappedU = hasUnwrappedU || mesh.vertices()[index].parameter.x() > TwoPi;
    }

    context.expect(mesh.isValid(), "Revolution seam-crossing patch mesh valid");
    context.expect(hasUnwrappedU, "Revolution seam-crossing patch preserves continuous unwrapped U");
    context.expect(triangleWindingMatchesNormals(mesh), "Revolution seam-crossing patch winding");
}

void testFullBand(TestContext& context)
{
    const double radius = 3.0;
    const double v0 = -1.5;
    const double v1 = 2.0;
    const MyBRep::Topology_Face face = createFullRevolutionBandFace(radius, v0, v1);
    const MyBRep::FaceMesh mesh = MyBRep::RevolvedFaceMesher::mesh(face, testOptions());

    context.expect(MyBRep::RevolvedFaceMesher::canMesh(face), "Full revolution seam Face can mesh");
    context.expect(mesh.isValid(), "Full revolution band mesh valid");
    context.expect(std::fabs(parameterUSpan(mesh) - TwoPi) <= 1.0e-8, "Full revolution band keeps one complete U period");
    context.expect(std::fabs(meshArea(mesh) - revolutionArea(radius, 0.0, TwoPi, v0, v1)) <= AreaTolerance, "Full revolution band mesh area");
    context.expect(triangleWindingMatchesNormals(mesh), "Full revolution band winding");
}

void testTolerance(TestContext& context)
{
    const MyBRep::Topology_Face face = createRevolutionPatchFace(3.0, 0.0, Pi, -2.0, 2.0);

    MyBRep::RevolvedFaceMeshOptions loose = testOptions();
    MyBRep::RevolvedFaceMeshOptions tight = testOptions();
    loose.surfaceChordTolerance = 0.20;
    tight.surfaceChordTolerance = 0.01;

    const MyBRep::FaceMesh looseMesh = MyBRep::RevolvedFaceMesher::mesh(face, loose);
    const MyBRep::FaceMesh tightMesh = MyBRep::RevolvedFaceMesher::mesh(face, tight);

    context.expect(looseMesh.isValid() && tightMesh.isValid(), "Loose and tight revolution meshes valid");
    context.expect(tightMesh.triangleCount() > looseMesh.triangleCount(), "Tighter revolution surface tolerance increases triangle count");
}

void testReversed(TestContext& context)
{
    const MyBRep::Topology_Face forwardFace = createRevolutionPatchFace(3.0, 0.2, 1.7, -1.5, 2.0);
    const MyBRep::Topology_Face reversedFace = forwardFace.reversed();
    const MyBRep::FaceMesh forwardMesh = MyBRep::RevolvedFaceMesher::mesh(forwardFace, testOptions());
    const MyBRep::FaceMesh reversedMesh = MyBRep::RevolvedFaceMesher::mesh(reversedFace, testOptions());

    context.expect(forwardMesh.isValid() && reversedMesh.isValid(), "Forward and Reversed revolution meshes valid");
    context.expect(std::fabs(meshArea(forwardMesh) - meshArea(reversedMesh)) <= 1.0e-8, "Reversed revolution preserves area");
    context.expect(normalsOppositeAtSharedPositions(forwardMesh, reversedMesh), "Reversed revolution shared-vertex normals are opposite");
    context.expect(vertexNormalsMatchFace(reversedFace, reversedMesh), "Reversed revolution normals match reversed Face");
    context.expect(triangleWindingMatchesNormals(reversedMesh), "Reversed revolution winding follows reversed normal");
}

}

int main()
{
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep Revolved Face Mesher Real Surface Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testPatch(context);
    testHole(context);
    testSeamCrossing(context);
    testFullBand(context);
    testTolerance(context);
    testReversed(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}
