#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "MyMath/Vector2.h"
#include "MyMath/Vector3.h"

#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Curve/Geometry_Curve.h"
#include "MyBRep/Geometry/Curve/Geometry_Line.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Curve2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Line2D.h"
#include "MyBRep/Geometry/Surface/Geometry_SurfaceOfExtrusion.h"
#include "MyBRep/Mesh/ExtrudedFaceMesher.h"
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

bool normalsOppositeAtSamePosition(const MyBRep::FaceMesh& forwardMesh, const MyBRep::FaceMesh& reversedMesh)
{
    if (!forwardMesh.isValid() || !reversedMesh.isValid())
    {
        return false;
    }

    for (std::size_t forwardIndex = 0; forwardIndex < forwardMesh.vertices().size(); ++forwardIndex)
    {
        const MyBRep::FaceMeshVertex& forwardVertex = forwardMesh.vertices()[forwardIndex];
        bool matched = false;

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

            matched = true;
            break;
        }

        if (!matched)
        {
            return false;
        }
    }

    return true;
}

MyBRep::ExtrudedFaceMeshOptions testOptions()
{
    MyBRep::ExtrudedFaceMeshOptions options;
    options.boundaryChordTolerance = 0.02;
    options.surfaceChordTolerance = 0.02;
    options.geometricTolerance = 1.0e-10;
    options.minimumBoundarySubdivisionDepth = 1;
    options.maximumBoundarySubdivisionDepth = 12;
    options.maximumSurfaceSubdivisionRounds = 12;
    return options;
}

MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> createExtrusionSurface()
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> profile(
        new MyBRep::Geometry_Line(MyMath::Vector3(0.0, 0.0, 0.0), MyMath::Vector3::unitX()));

    return MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>(
        new MyBRep::Geometry_SurfaceOfExtrusion(profile, MyMath::Vector3::unitZ()));
}

MyBRep::Topology_Wire createExtrusionPatchWire(const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>& surface,
                                               double u0,
                                               double u1,
                                               double v0,
                                               double v1)
{
    const MyBRep::Topology_Vertex v00(surface->pointAt(u0, v0));
    const MyBRep::Topology_Vertex v10(surface->pointAt(u1, v0));
    const MyBRep::Topology_Vertex v11(surface->pointAt(u1, v1));
    const MyBRep::Topology_Vertex v01(surface->pointAt(u0, v1));

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> bottomGeometry(
        new MyBRep::Geometry_Line(v00.point(), v10.point() - v00.point()));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> rightGeometry(
        new MyBRep::Geometry_Line(v10.point(), v11.point() - v10.point()));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> topGeometry(
        new MyBRep::Geometry_Line(v01.point(), v11.point() - v01.point()));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> leftGeometry(
        new MyBRep::Geometry_Line(v00.point(), v01.point() - v00.point()));

    MyBRep::Topology_Edge bottom(v00, v10, bottomGeometry, 0.0, (v10.point() - v00.point()).length(), TestTolerance);
    MyBRep::Topology_Edge right(v10, v11, rightGeometry, 0.0, (v11.point() - v10.point()).length(), TestTolerance);
    MyBRep::Topology_Edge top(v01, v11, topGeometry, 0.0, (v11.point() - v01.point()).length(), TestTolerance);
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

MyBRep::Topology_Face createExtrusionPatchFace(double u0, double u1, double v0, double v1)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface = createExtrusionSurface();
    return MyBRep::Modeling::createFace(surface, std::vector<MyBRep::Topology_Wire>(1, createExtrusionPatchWire(surface, u0, u1, v0, v1)));
}

MyBRep::Topology_Face createExtrusionFaceWithHole()
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface = createExtrusionSurface();
    std::vector<MyBRep::Topology_Wire> wires;
    wires.push_back(createExtrusionPatchWire(surface, -3.0, 3.0, -2.0, 2.0));
    wires.push_back(createExtrusionPatchWire(surface, -1.0, 1.0, -0.75, 0.75));
    return MyBRep::Modeling::createFace(surface, wires);
}

void testPatch(TestContext& context)
{
    const MyBRep::Topology_Face face = createExtrusionPatchFace(-2.0, 2.0, -1.5, 1.5);
    const MyBRep::FaceMesh mesh = MyBRep::ExtrudedFaceMesher::mesh(face, testOptions());
    const double expectedArea = 4.0 * 3.0;

    context.expect(MyBRep::ExtrudedFaceMesher::canMesh(face), "Extrusion patch Face can mesh");
    context.expect(mesh.isValid(), "Extrusion patch mesh valid");
    context.expect(mesh.triangleCount() == 2, "Linear-profile extrusion stays at two triangles");
    context.expect(std::fabs(meshArea(mesh) - expectedArea) <= AreaTolerance, "Extrusion patch mesh area");
    context.expect(vertexNormalsMatchFace(face, mesh), "Extrusion patch vertex normals match Face");
    context.expect(triangleWindingMatchesNormals(mesh), "Extrusion patch winding matches normals");
}

void testHole(TestContext& context)
{
    const MyBRep::Topology_Face face = createExtrusionFaceWithHole();
    const MyBRep::FaceMesh mesh = MyBRep::ExtrudedFaceMesher::mesh(face, testOptions());
    const double expectedArea = 6.0 * 4.0 - 2.0 * 1.5;

    context.expect(mesh.isValid(), "Extrusion Face with hole mesh valid");
    context.expect(std::fabs(meshArea(mesh) - expectedArea) <= AreaTolerance, "Extrusion Face with hole area");
    context.expect(mesh.triangleCount() > 2, "Extrusion Face with hole keeps even-odd triangulation");
    context.expect(triangleWindingMatchesNormals(mesh), "Extrusion Face with hole winding");
}

void testReversed(TestContext& context)
{
    const MyBRep::Topology_Face forwardFace = createExtrusionPatchFace(-2.0, 2.0, -1.5, 1.5);
    const MyBRep::Topology_Face reversedFace = forwardFace.reversed();
    const MyBRep::FaceMesh forwardMesh = MyBRep::ExtrudedFaceMesher::mesh(forwardFace, testOptions());
    const MyBRep::FaceMesh reversedMesh = MyBRep::ExtrudedFaceMesher::mesh(reversedFace, testOptions());

    context.expect(forwardMesh.isValid() && reversedMesh.isValid(), "Forward and Reversed extrusion meshes valid");
    context.expect(std::fabs(meshArea(forwardMesh) - meshArea(reversedMesh)) <= 1.0e-8, "Reversed extrusion preserves area");
    context.expect(normalsOppositeAtSamePosition(forwardMesh, reversedMesh), "Reversed extrusion normals are opposite");
    context.expect(vertexNormalsMatchFace(reversedFace, reversedMesh), "Reversed extrusion normals match reversed Face");
    context.expect(triangleWindingMatchesNormals(reversedMesh), "Reversed extrusion winding follows reversed normal");
}

}

int main()
{
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep Extruded Face Mesher Real Surface Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testPatch(context);
    testHole(context);
    testReversed(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}
