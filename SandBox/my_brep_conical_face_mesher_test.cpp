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
#include "MyBRep/Geometry/Curve/Geometry_Line.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Curve2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Line2D.h"
#include "MyBRep/Geometry/Surface/Geometry_ConicalSurface.h"
#include "MyBRep/Geometry/Surface/Geometry_CylindricalSurface.h"
#include "MyBRep/Mesh/ConicalFaceMesher.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Topology/Topology_Builder.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 圆锥测试统一使用的圆周率。
const double TwoPi = Pi * 2.0;                        // 完整圆锥U参数周期。
const double TestTolerance = 1.0e-8;                 // Topology与Curve-on-Surface连接验证统一使用的三维容差。
const double AreaTolerance = 0.8;                    // 三角网格面积与解析圆锥面片面积比较允许的绝对误差。
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

MyBRep::ConicalFaceMeshOptions testOptions()
{
    MyBRep::ConicalFaceMeshOptions options;
    options.boundaryChordTolerance = 0.02;
    options.surfaceChordTolerance = 0.02;
    options.geometricTolerance = 1.0e-10;
    options.minimumBoundarySubdivisionDepth = 1;
    options.maximumBoundarySubdivisionDepth = 12;
    options.maximumSurfaceSubdivisionRounds = 12;
    return options;
}

MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> createConeSurface(const MyMath::CoordinateSystem& coordinateSystem, double semiAngle)
{
    return MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>(new MyBRep::Geometry_ConicalSurface(coordinateSystem, semiAngle));
}

void addLineCurveOnSurface(MyBRep::Topology_Edge& edge, const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>& surface, const MyMath::Vector2& firstUV, const MyMath::Vector2& lastUV)
{
    const MyMath::Vector2 direction = lastUV - firstUV;
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> curve(new MyBRep::Geometry_Line2D(firstUV, direction));
    MyBRep::Topology_Builder::addCurveOnSurface(edge, surface, curve, 0.0, direction.length(), TestTolerance);
}

MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> createConeRingCurve(const MyBRep::Geometry_ConicalSurface& cone, double v)
{
    const double radius = v * cone.radialSlope();
    const MyMath::Vector3 center = cone.apex() + cone.axisDir() * v;
    return MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve>(new MyBRep::Geometry_Circle(center, radius, cone.xDir(), cone.yDir()));
}

MyBRep::Topology_Wire createConePatchWire(const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>& surface, double u0, double u1, double v0, double v1)
{
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
    return MyBRep::Topology_Wire(edges);
}

MyBRep::Topology_Face createConePatchFace(const MyMath::CoordinateSystem& coordinateSystem, double semiAngle, double u0, double u1, double v0, double v1)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface = createConeSurface(coordinateSystem, semiAngle);
    return MyBRep::Modeling::createFace(surface, std::vector<MyBRep::Topology_Wire>(1, createConePatchWire(surface, u0, u1, v0, v1)));
}

MyBRep::Topology_Face createConeFaceWithHole(const MyMath::CoordinateSystem& coordinateSystem, double semiAngle)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface = createConeSurface(coordinateSystem, semiAngle);

    std::vector<MyBRep::Topology_Wire> wires;
    wires.push_back(createConePatchWire(surface, 0.0, 2.0, 2.0, 6.0));
    wires.push_back(createConePatchWire(surface, 0.7, 1.3, 3.0, 5.0));
    return MyBRep::Modeling::createFace(surface, wires);
}

MyBRep::Topology_Face createFullConeBandFace(const MyMath::CoordinateSystem& coordinateSystem, double semiAngle, double v0, double v1)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface = createConeSurface(coordinateSystem, semiAngle);
    const MyBRep::Geometry_ConicalSurface& cone = static_cast<const MyBRep::Geometry_ConicalSurface&>(*surface);

    const MyBRep::Topology_Vertex bottomVertex(cone.pointAt(0.0, v0));
    const MyBRep::Topology_Vertex topVertex(cone.pointAt(0.0, v1));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> seamGeometry(new MyBRep::Geometry_Line(bottomVertex.point(), topVertex.point() - bottomVertex.point()));

    MyBRep::Topology_Edge bottom(bottomVertex, bottomVertex, createConeRingCurve(cone, v0), 0.0, TwoPi, TestTolerance);
    MyBRep::Topology_Edge top(topVertex, topVertex, createConeRingCurve(cone, v1), 0.0, TwoPi, TestTolerance);
    MyBRep::Topology_Edge seam(bottomVertex, topVertex, seamGeometry, 0.0, (topVertex.point() - bottomVertex.point()).length(), TestTolerance);

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

MyBRep::Topology_Face createApexTouchingConeFace(const MyMath::CoordinateSystem& coordinateSystem, double semiAngle)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface = createConeSurface(coordinateSystem, semiAngle);
    const MyBRep::Geometry_ConicalSurface& cone = static_cast<const MyBRep::Geometry_ConicalSurface&>(*surface);
    const double u0 = 0.0;
    const double u1 = Pi * 0.5;
    const double v1 = 4.0;

    const MyBRep::Topology_Vertex apexVertex(cone.pointAt(u0, 0.0));
    const MyBRep::Topology_Vertex firstTopVertex(cone.pointAt(u0, v1));
    const MyBRep::Topology_Vertex secondTopVertex(cone.pointAt(u1, v1));

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> firstGeneratorGeometry(new MyBRep::Geometry_Line(apexVertex.point(), firstTopVertex.point() - apexVertex.point()));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> secondGeneratorGeometry(new MyBRep::Geometry_Line(apexVertex.point(), secondTopVertex.point() - apexVertex.point()));

    MyBRep::Topology_Edge firstGenerator(apexVertex, firstTopVertex, firstGeneratorGeometry, 0.0, (firstTopVertex.point() - apexVertex.point()).length(), TestTolerance);
    MyBRep::Topology_Edge top(firstTopVertex, secondTopVertex, createConeRingCurve(cone, v1), u0, u1, TestTolerance);
    MyBRep::Topology_Edge secondGenerator(apexVertex, secondTopVertex, secondGeneratorGeometry, 0.0, (secondTopVertex.point() - apexVertex.point()).length(), TestTolerance);

    addLineCurveOnSurface(firstGenerator, surface, MyMath::Vector2(u0, 0.0), MyMath::Vector2(u0, v1));
    addLineCurveOnSurface(top, surface, MyMath::Vector2(u0, v1), MyMath::Vector2(u1, v1));
    addLineCurveOnSurface(secondGenerator, surface, MyMath::Vector2(u1, 0.0), MyMath::Vector2(u1, v1));

    std::vector<MyBRep::Topology_Edge> edges;
    edges.push_back(firstGenerator);
    edges.push_back(top);
    edges.push_back(secondGenerator.reversed());
    return MyBRep::Modeling::createFace(surface, std::vector<MyBRep::Topology_Wire>(1, MyBRep::Topology_Wire(edges)));
}

MyBRep::Topology_Face createFullApexConeFace(const MyMath::CoordinateSystem& coordinateSystem, double semiAngle, double v1)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface = createConeSurface(coordinateSystem, semiAngle);
    const MyBRep::Geometry_ConicalSurface& cone = static_cast<const MyBRep::Geometry_ConicalSurface&>(*surface);

    const MyBRep::Topology_Vertex apexVertex(cone.pointAt(0.0, 0.0));
    const MyBRep::Topology_Vertex topVertex(cone.pointAt(0.0, v1));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> seamGeometry(new MyBRep::Geometry_Line(apexVertex.point(), topVertex.point() - apexVertex.point()));

    MyBRep::Topology_Edge seam(apexVertex, topVertex, seamGeometry, 0.0, (topVertex.point() - apexVertex.point()).length(), TestTolerance);
    MyBRep::Topology_Edge top(topVertex, topVertex, createConeRingCurve(cone, v1), 0.0, TwoPi, TestTolerance);

    addLineCurveOnSurface(top, surface, MyMath::Vector2(0.0, v1), MyMath::Vector2(TwoPi, v1));

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> firstSeamCurve(new MyBRep::Geometry_Line2D(MyMath::Vector2(0.0, 0.0), MyMath::Vector2(0.0, 1.0)));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> secondSeamCurve(new MyBRep::Geometry_Line2D(MyMath::Vector2(TwoPi, 0.0), MyMath::Vector2(0.0, 1.0)));
    MyBRep::Topology_Builder::addCurveOnClosedSurface(seam, surface, firstSeamCurve, 0.0, v1, secondSeamCurve, 0.0, v1, TestTolerance);

    std::vector<MyBRep::Topology_Edge> edges;
    edges.push_back(seam);
    edges.push_back(top);
    edges.push_back(seam.reversed());
    return MyBRep::Modeling::createFace(surface, std::vector<MyBRep::Topology_Wire>(1, MyBRep::Topology_Wire(edges)));
}

double conePatchArea(const MyBRep::Geometry_ConicalSurface& cone, double u0, double u1, double v0, double v1)
{
    const double slope = cone.radialSlope();
    const double metricScale = slope * std::sqrt(1.0 + slope * slope);
    return (u1 - u0) * 0.5 * (v1 * v1 - v0 * v0) * metricScale;
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

MyMath::Vector3 coneLimitNormalForTest(const MyBRep::Topology_Face& face, const MyBRep::Geometry_ConicalSurface& cone, double u)
{
    const MyMath::Vector3 radialDir = cone.xDir() * std::cos(u) + cone.yDir() * std::sin(u);
    const MyMath::Vector3 normal = (radialDir - cone.axisDir() * cone.radialSlope()).normalized(0.0);
    return face.isForward() ? normal : normal * -1.0;
}

bool vertexNormalsMatchConeIncludingApex(const MyBRep::Topology_Face& face, const MyBRep::FaceMesh& mesh)
{
    const MyBRep::Geometry_ConicalSurface& cone = static_cast<const MyBRep::Geometry_ConicalSurface&>(face.geometry());

    for (std::size_t index = 0; index < mesh.vertices().size(); ++index)
    {
        const MyBRep::FaceMeshVertex& vertex = mesh.vertices()[index];
        const bool apex = vertex.parameter.y() <= cone.vDomainStart() + TestTolerance;
        const MyMath::Vector3 expected = apex ? coneLimitNormalForTest(face, cone, vertex.parameter.x()) : face.normalAt(vertex.parameter.x(), vertex.parameter.y());

        if (!vertex.normal.isEqualTo(expected, NormalTolerance))
        {
            return false;
        }
    }

    return true;
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

bool normalsOppositeAtSamePosition(const MyBRep::FaceMesh& forwardMesh, const MyBRep::FaceMesh& reversedMesh)
{
    if (forwardMesh.vertices().empty() || reversedMesh.vertices().empty())
    {
        return false;
    }

    const MyBRep::FaceMeshVertex& reference = forwardMesh.vertices()[0];

    for (std::size_t index = 0; index < reversedMesh.vertices().size(); ++index)
    {
        const MyBRep::FaceMeshVertex& candidate = reversedMesh.vertices()[index];

        if (!reference.position.isEqualTo(candidate.position, NormalTolerance))
        {
            continue;
        }

        return MyMath::Vector3::dot(reference.normal, candidate.normal) < -0.999999;
    }

    return false;
}

void testDefaultOptions(TestContext& context)
{
    const MyBRep::ConicalFaceMeshOptions options;
    context.expect(options.isValid(), "Default conical mesh options valid");
}

void testConePatch(TestContext& context)
{
    const double semiAngle = Pi / 6.0;
    const double u0 = 0.2;
    const double u1 = 1.7;
    const double v0 = 2.0;
    const double v1 = 6.0;

    const MyBRep::Topology_Face face = createConePatchFace(MyMath::CoordinateSystem::identity(), semiAngle, u0, u1, v0, v1);
    const MyBRep::FaceMesh mesh = MyBRep::ConicalFaceMesher::mesh(face, testOptions());
    const MyBRep::Geometry_ConicalSurface& cone = static_cast<const MyBRep::Geometry_ConicalSurface&>(face.geometry());
    const double expectedArea = conePatchArea(cone, u0, u1, v0, v1);

    context.expect(MyBRep::ConicalFaceMesher::canMesh(face), "Conical patch Face can mesh");
    context.expect(mesh.isValid(), "Conical patch mesh valid");
    context.expect(mesh.triangleCount() > 2, "Conical patch receives curvature subdivision");
    context.expect(std::fabs(meshArea(mesh) - expectedArea) <= AreaTolerance, "Conical patch mesh area");
    context.expect(triangleWindingMatchesNormals(mesh), "Conical patch triangle winding matches normals");
    context.expect(vertexNormalsMatchFace(face, mesh), "Conical patch vertex normals match Face normals");
}

void testConeHole(TestContext& context)
{
    const double semiAngle = 0.55;
    const MyBRep::Topology_Face face = createConeFaceWithHole(MyMath::CoordinateSystem::identity(), semiAngle);
    const MyBRep::FaceMesh mesh = MyBRep::ConicalFaceMesher::mesh(face, testOptions());
    const MyBRep::Geometry_ConicalSurface& cone = static_cast<const MyBRep::Geometry_ConicalSurface&>(face.geometry());

    const double outerArea = conePatchArea(cone, 0.0, 2.0, 2.0, 6.0);
    const double holeArea = conePatchArea(cone, 0.7, 1.3, 3.0, 5.0);

    context.expect(mesh.isValid(), "Conical Face with hole mesh valid");
    context.expect(std::fabs(meshArea(mesh) - (outerArea - holeArea)) <= AreaTolerance, "Conical Face with hole area");
    context.expect(triangleWindingMatchesNormals(mesh), "Conical Face with hole winding");
}

void testSeamCrossingPatch(TestContext& context)
{
    const MyBRep::Topology_Face face = createConePatchFace(MyMath::CoordinateSystem::identity(), Pi / 7.0, 5.7, 6.7, 2.0, 6.0);
    const MyBRep::FaceMesh mesh = MyBRep::ConicalFaceMesher::mesh(face, testOptions());

    bool containsBeyondTwoPi = false;

    for (std::size_t index = 0; index < mesh.vertices().size(); ++index)
    {
        if (mesh.vertices()[index].parameter.x() > TwoPi)
        {
            containsBeyondTwoPi = true;
            break;
        }
    }

    context.expect(mesh.isValid(), "Conical seam-crossing patch mesh valid");
    context.expect(containsBeyondTwoPi, "Conical seam-crossing patch preserves continuous unwrapped U");
    context.expect(triangleWindingMatchesNormals(mesh), "Conical seam-crossing patch winding");
}

void testFullConeBand(TestContext& context)
{
    const double semiAngle = Pi / 6.0;
    const double v0 = 2.0;
    const double v1 = 6.0;
    const MyBRep::Topology_Face face = createFullConeBandFace(MyMath::CoordinateSystem::identity(), semiAngle, v0, v1);

    MyBRep::ConicalFaceMeshOptions options = testOptions();
    options.boundaryChordTolerance = 0.05;

    const MyBRep::FaceMesh mesh = MyBRep::ConicalFaceMesher::mesh(face, options);
    const MyBRep::Geometry_ConicalSurface& cone = static_cast<const MyBRep::Geometry_ConicalSurface&>(face.geometry());
    const double expectedArea = conePatchArea(cone, 0.0, TwoPi, v0, v1);

    double minimumU = 0.0;
    double maximumU = 0.0;

    if (!mesh.vertices().empty())
    {
        minimumU = mesh.vertices()[0].parameter.x();
        maximumU = mesh.vertices()[0].parameter.x();

        for (std::size_t index = 1; index < mesh.vertices().size(); ++index)
        {
            minimumU = (std::min)(minimumU, mesh.vertices()[index].parameter.x());
            maximumU = (std::max)(maximumU, mesh.vertices()[index].parameter.x());
        }
    }

    context.expect(MyBRep::ConicalFaceMesher::canMesh(face), "Full conical seam Face can mesh");
    context.expect(mesh.isValid(), "Full conical band mesh valid");
    context.expect(maximumU - minimumU >= TwoPi - 1.0e-8, "Full conical band keeps one complete U period");
    context.expect(std::fabs(meshArea(mesh) - expectedArea) <= AreaTolerance, "Full conical band mesh area");
    context.expect(triangleWindingMatchesNormals(mesh), "Full conical band winding");
}

void testSurfaceToleranceRefinement(TestContext& context)
{
    const MyBRep::Topology_Face face = createConePatchFace(MyMath::CoordinateSystem::identity(), Pi / 6.0, 0.0, Pi, 2.0, 7.0);

    MyBRep::ConicalFaceMeshOptions loose = testOptions();
    MyBRep::ConicalFaceMeshOptions tight = testOptions();

    loose.boundaryChordTolerance = 0.05;
    tight.boundaryChordTolerance = 0.05;
    loose.surfaceChordTolerance = 0.20;
    tight.surfaceChordTolerance = 0.01;

    const MyBRep::FaceMesh looseMesh = MyBRep::ConicalFaceMesher::mesh(face, loose);
    const MyBRep::FaceMesh tightMesh = MyBRep::ConicalFaceMesher::mesh(face, tight);

    context.expect(looseMesh.isValid() && tightMesh.isValid(), "Loose and tight conical meshes valid");
    context.expect(tightMesh.triangleCount() > looseMesh.triangleCount(), "Tighter conical surface tolerance increases triangle count");
}

void testReversedFace(TestContext& context)
{
    const MyBRep::Topology_Face forwardFace = createConePatchFace(MyMath::CoordinateSystem::identity(), Pi / 6.0, 0.0, Pi * 0.75, 2.0, 6.0);
    const MyBRep::Topology_Face reversedFace = forwardFace.reversed();

    const MyBRep::FaceMesh forwardMesh = MyBRep::ConicalFaceMesher::mesh(forwardFace, testOptions());
    const MyBRep::FaceMesh reversedMesh = MyBRep::ConicalFaceMesher::mesh(reversedFace, testOptions());

    context.expect(forwardMesh.isValid() && reversedMesh.isValid(), "Forward and Reversed conical Face meshes valid");
    context.expect(std::fabs(meshArea(forwardMesh) - meshArea(reversedMesh)) <= 1.0e-8, "Reversed conical Face preserves mesh area");
    context.expect(normalsOppositeAtSamePosition(forwardMesh, reversedMesh), "Reversed conical Face normals are opposite at the same surface point");
    context.expect(vertexNormalsMatchFace(reversedFace, reversedMesh), "Reversed conical Face vertex normals match reversed Face");
    context.expect(triangleWindingMatchesNormals(reversedMesh), "Reversed conical Face winding follows reversed normal");
}

void testOrientedCone(TestContext& context)
{
    const MyMath::CoordinateSystem coordinateSystem = MyMath::CoordinateSystem::fromAxes(MyMath::Vector3(3.0, 4.0, 5.0), MyMath::Vector3::unitY(), MyMath::Vector3::unitZ(), MyMath::Vector3::unitX());
    const MyBRep::Topology_Face face = createConePatchFace(coordinateSystem, Pi / 5.0, 0.2, 1.4, 2.0, 6.0);
    const MyBRep::FaceMesh mesh = MyBRep::ConicalFaceMesher::mesh(face, testOptions());

    context.expect(mesh.isValid(), "Oriented conical Face mesh valid");
    context.expect(vertexNormalsMatchFace(face, mesh), "Oriented conical normals match Face");
    context.expect(triangleWindingMatchesNormals(mesh), "Oriented conical winding matches normal");
}

void testApexTouchingPatch(TestContext& context)
{
    const double semiAngle = Pi / 6.0;
    const MyBRep::Topology_Face face = createApexTouchingConeFace(MyMath::CoordinateSystem::identity(), semiAngle);
    const MyBRep::Geometry_ConicalSurface& cone = static_cast<const MyBRep::Geometry_ConicalSurface&>(face.geometry());
    const MyBRep::FaceMesh mesh = MyBRep::ConicalFaceMesher::mesh(face, testOptions());
    const double expectedArea = conePatchArea(cone, 0.0, Pi * 0.5, 0.0, 4.0);

    context.expect(MyBRep::ConicalFaceMesher::canMesh(face), "Apex-touching conical Face can mesh");
    context.expect(mesh.isValid(), "Apex-touching conical Face mesh valid");
    context.expect(countVerticesAtPosition(mesh, cone.apex(), NormalTolerance) >= 2, "Apex-touching patch keeps U-dependent apex normal vertices");
    context.expect(std::fabs(meshArea(mesh) - expectedArea) <= AreaTolerance, "Apex-touching conical Face mesh area");
    context.expect(vertexNormalsMatchConeIncludingApex(face, mesh), "Apex-touching conical Face normals use analytic limiting normals");
    context.expect(triangleWindingMatchesNormals(mesh), "Apex-touching conical Face winding");
}

void testFullApexCone(TestContext& context)
{
    const double semiAngle = Pi / 6.0;
    const double v1 = 6.0;
    const MyBRep::Topology_Face face = createFullApexConeFace(MyMath::CoordinateSystem::identity(), semiAngle, v1);
    const MyBRep::Geometry_ConicalSurface& cone = static_cast<const MyBRep::Geometry_ConicalSurface&>(face.geometry());

    MyBRep::ConicalFaceMeshOptions options = testOptions();
    options.boundaryChordTolerance = 0.04;

    const MyBRep::FaceMesh mesh = MyBRep::ConicalFaceMesher::mesh(face, options);
    const double expectedArea = conePatchArea(cone, 0.0, TwoPi, 0.0, v1);

    context.expect(MyBRep::ConicalFaceMesher::canMesh(face), "Full apex conical Face can mesh");
    context.expect(mesh.isValid(), "Full apex conical Face mesh valid");
    context.expect(mesh.triangleCount() > 8, "Full apex cone receives curvature subdivision");
    context.expect(std::fabs(parameterUSpan(mesh) - TwoPi) <= 1.0e-8, "Full apex cone preserves one complete unwrapped U period");
    context.expect(countVerticesAtPosition(mesh, cone.apex(), NormalTolerance) >= 2, "Full apex cone keeps seam-limit apex vertices");
    context.expect(std::fabs(meshArea(mesh) - expectedArea) <= AreaTolerance, "Full apex conical Face mesh area");
    context.expect(vertexNormalsMatchConeIncludingApex(face, mesh), "Full apex cone normals include valid apex limiting normals");
    context.expect(triangleWindingMatchesNormals(mesh), "Full apex cone winding");
}

void testReversedFullApexCone(TestContext& context)
{
    const MyBRep::Topology_Face forwardFace = createFullApexConeFace(MyMath::CoordinateSystem::identity(), Pi / 6.0, 6.0);
    const MyBRep::Topology_Face reversedFace = forwardFace.reversed();
    const MyBRep::FaceMesh forwardMesh = MyBRep::ConicalFaceMesher::mesh(forwardFace, testOptions());
    const MyBRep::FaceMesh reversedMesh = MyBRep::ConicalFaceMesher::mesh(reversedFace, testOptions());

    context.expect(forwardMesh.isValid() && reversedMesh.isValid(), "Forward and Reversed full apex cone meshes valid");
    context.expect(std::fabs(meshArea(forwardMesh) - meshArea(reversedMesh)) <= 1.0e-8, "Reversed full apex cone preserves mesh area");
    context.expect(vertexNormalsMatchConeIncludingApex(reversedFace, reversedMesh), "Reversed full apex cone normals match limiting Face orientation");
    context.expect(triangleWindingMatchesNormals(reversedMesh), "Reversed full apex cone winding follows reversed normal");
}

void testUnsupportedCylinder(TestContext& context)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface(new MyBRep::Geometry_CylindricalSurface(MyMath::CoordinateSystem::identity(), 4.0));
    const MyBRep::Topology_Face face(surface);

    context.expect(!MyBRep::ConicalFaceMesher::canMesh(face), "Cylinder Face cannot use conical mesher");
    context.expect(MyBRep::ConicalFaceMesher::mesh(face).isEmpty(), "Cylinder Face returns empty conical mesh");
}

}

int main()
{
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep Conical Face Mesher Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testDefaultOptions(context);
    testConePatch(context);
    testConeHole(context);
    testSeamCrossingPatch(context);
    testFullConeBand(context);
    testSurfaceToleranceRefinement(context);
    testReversedFace(context);
    testOrientedCone(context);
    testApexTouchingPatch(context);
    testFullApexCone(context);
    testReversedFullApexCone(context);
    testUnsupportedCylinder(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}
