#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "MyMath/CoordinateSystem.h"
#include "MyMath/Vector2.h"
#include "MyMath/Vector3.h"

#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Curve/Geometry_Line.h"
#include "MyBRep/Mesh/PlanarFaceMesher.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Modeling/Wire/WireModeling.h"
#include "MyBRep/Tool/Query/FaceClassifier2D.h"

namespace
{

const double TestTolerance = 1.0e-8;     // Face构造与面积比较使用的测试几何容差。
const double MeshChordTolerance = 0.01;   // 圆边界测试使用1e-2弦误差，兼顾网格规模与面积精度。
const double AreaTolerance = 1.0e-6;      // 直线多边形理论面积比较允许的数值误差。
const double CircleAreaTolerance = 0.2;   // 圆经折线离散后允许0.2平方单位面积误差。
const double Pi = 3.14159265358979323846; // 圆面积理论值计算使用的圆周率常量。

class TestContext
{
public:
    TestContext()
        : m_passed(0)
        , m_failed(0)
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

double triangleArea(const MyMath::Vector3& first, const MyMath::Vector3& second, const MyMath::Vector3& third)
{
    return MyMath::Vector3::cross(second - first, third - first).length() * 0.5;
}

double meshArea(const MyBRep::FaceMesh& mesh)
{
    double area = 0.0;
    const std::vector<MyBRep::FaceMeshVertex>& vertices = mesh.vertices();
    const std::vector<unsigned int>& indices = mesh.indices();

    for (std::size_t index = 0; index < indices.size(); index += 3)
    {
        area += triangleArea(vertices[indices[index]].position, vertices[indices[index + 1]].position, vertices[indices[index + 2]].position);
    }

    return area;
}

bool allTriangleCentersInside(const MyBRep::Topology_Face& face, const MyBRep::FaceMesh& mesh)
{
    const std::vector<MyBRep::FaceMeshVertex>& vertices = mesh.vertices();
    const std::vector<unsigned int>& indices = mesh.indices();

    for (std::size_t index = 0; index < indices.size(); index += 3)
    {
        const MyMath::Vector2 center =
            (vertices[indices[index]].parameter + vertices[indices[index + 1]].parameter + vertices[indices[index + 2]].parameter) / 3.0;

        if (MyBRep::classifyFaceUV(face, center, TestTolerance) != MyBRep::FaceUVClassification::Inside)
        {
            return false;
        }
    }

    return true;
}

bool triangleWindingMatchesNormals(const MyBRep::FaceMesh& mesh)
{
    const std::vector<MyBRep::FaceMeshVertex>& vertices = mesh.vertices();
    const std::vector<unsigned int>& indices = mesh.indices();

    for (std::size_t index = 0; index < indices.size(); index += 3)
    {
        const MyBRep::FaceMeshVertex& first = vertices[indices[index]];
        const MyBRep::FaceMeshVertex& second = vertices[indices[index + 1]];
        const MyBRep::FaceMeshVertex& third = vertices[indices[index + 2]];
        const MyMath::Vector3 triangleNormal = MyMath::Vector3::cross(second.position - first.position, third.position - first.position);

        if (MyMath::Vector3::dot(triangleNormal, first.normal) <= 0.0)
        {
            return false;
        }
    }

    return true;
}

MyBRep::PlanarFaceMeshOptions testOptions()
{
    MyBRep::PlanarFaceMeshOptions options;
    options.chordTolerance = MeshChordTolerance;
    options.geometricTolerance = TestTolerance;
    return options;
}

void testRectangle(TestContext& context)
{
    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createRectangle(10.0, 8.0), TestTolerance);
    const MyBRep::FaceMesh mesh = MyBRep::PlanarFaceMesher::mesh(face, testOptions());

    context.expect(MyBRep::PlanarFaceMesher::canMesh(face), "Rectangle Face can mesh");
    context.expect(mesh.isValid(), "Rectangle mesh valid");
    context.expect(mesh.triangleCount() == 2, "Rectangle triangulates to two triangles");
    context.expect(std::fabs(meshArea(mesh) - 80.0) <= AreaTolerance, "Rectangle mesh area");
    context.expect(triangleWindingMatchesNormals(mesh), "Rectangle triangle winding matches Face normal");
}

void testRectangleHole(TestContext& context)
{
    std::vector<MyBRep::Topology_Wire> wires;
    wires.push_back(MyBRep::Modeling::createRectangle(10.0, 8.0));
    wires.push_back(MyBRep::Modeling::createRectangle(4.0, 2.0));

    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(wires, TestTolerance);
    const MyBRep::FaceMesh mesh = MyBRep::PlanarFaceMesher::mesh(face, testOptions());

    context.expect(mesh.isValid(), "Rectangle with hole mesh valid");
    context.expect(std::fabs(meshArea(mesh) - 72.0) <= AreaTolerance, "Rectangle with hole area");
    context.expect(allTriangleCentersInside(face, mesh), "Rectangle with hole triangles remain inside Face");
}

void testMultipleHoles(TestContext& context)
{
    std::vector<MyBRep::Topology_Wire> wires;
    wires.push_back(MyBRep::Modeling::createRectangle(20.0, 12.0));
    wires.push_back(MyBRep::Modeling::createRectangle(MyMath::Vector3(-5.0, 0.0, 0.0), 4.0, 4.0));
    wires.push_back(MyBRep::Modeling::createRectangle(MyMath::Vector3(5.0, 0.0, 0.0), 4.0, 4.0));

    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(wires, TestTolerance);
    const MyBRep::FaceMesh mesh = MyBRep::PlanarFaceMesher::mesh(face, testOptions());

    context.expect(mesh.isValid(), "Multiple-hole Face mesh valid");
    context.expect(std::fabs(meshArea(mesh) - 208.0) <= AreaTolerance, "Multiple-hole Face area");
    context.expect(allTriangleCentersInside(face, mesh), "Multiple-hole triangles remain inside Face");
}

void testThreeNestedWires(TestContext& context)
{
    std::vector<MyBRep::Topology_Wire> wires;
    wires.push_back(MyBRep::Modeling::createRectangle(12.0, 10.0));
    wires.push_back(MyBRep::Modeling::createRectangle(8.0, 6.0));
    wires.push_back(MyBRep::Modeling::createRectangle(2.0, 2.0));

    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(wires, TestTolerance);
    const MyBRep::FaceMesh mesh = MyBRep::PlanarFaceMesher::mesh(face, testOptions());

    context.expect(mesh.isValid(), "Three-nested-Wire Face mesh valid");
    context.expect(std::fabs(meshArea(mesh) - 76.0) <= AreaTolerance, "Three-nested-Wire even-odd area");
    context.expect(allTriangleCentersInside(face, mesh), "Three-nested-Wire triangles remain inside Face");
}

void testCircleFace(TestContext& context)
{
    const double radius = 5.0; // 圆Face测试半径固定为5个模型单位，便于直接计算理论面积。
    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createCircle(radius), TestTolerance);
    const MyBRep::FaceMesh mesh = MyBRep::PlanarFaceMesher::mesh(face, testOptions());

    context.expect(mesh.isValid(), "Circle Face mesh valid");
    context.expect(std::fabs(meshArea(mesh) - Pi * radius * radius) <= CircleAreaTolerance, "Circle Face mesh area within chord approximation tolerance");
    context.expect(allTriangleCentersInside(face, mesh), "Circle triangles remain inside Face");
}

void testReversedFace(TestContext& context)
{
    const MyBRep::Topology_Face forwardFace = MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createRectangle(10.0, 8.0), TestTolerance);
    const MyBRep::Topology_Face reversedFace = forwardFace.reversed();

    const MyBRep::FaceMesh forwardMesh = MyBRep::PlanarFaceMesher::mesh(forwardFace, testOptions());
    const MyBRep::FaceMesh reversedMesh = MyBRep::PlanarFaceMesher::mesh(reversedFace, testOptions());

    context.expect(forwardMesh.isValid() && reversedMesh.isValid(), "Forward and Reversed Face mesh valid");
    context.expect(std::fabs(meshArea(forwardMesh) - meshArea(reversedMesh)) <= AreaTolerance, "Reversed Face preserves area");
    context.expect(triangleWindingMatchesNormals(reversedMesh), "Reversed Face triangle winding follows reversed normal");

    const bool normalsOpposite =
        !forwardMesh.vertices().empty() &&
        !reversedMesh.vertices().empty() &&
        MyMath::Vector3::dot(forwardMesh.vertices()[0].normal, reversedMesh.vertices()[0].normal) < -0.999999;

    context.expect(normalsOpposite, "Reversed Face normals are opposite");
}

MyBRep::Topology_Wire createSpatialRectangle(const MyMath::Vector3& center,
                                                  const MyMath::Vector3& uDirection,
                                                  const MyMath::Vector3& vDirection,
                                                  double sizeU,
                                                  double sizeV)
{
    const double halfU = sizeU * 0.5; // 矩形沿U方向的半尺寸。
    const double halfV = sizeV * 0.5; // 矩形沿V方向的半尺寸。

    std::vector<MyBRep::Topology_Vertex> vertices;
    vertices.push_back(MyBRep::Topology_Vertex(center - uDirection * halfU - vDirection * halfV));
    vertices.push_back(MyBRep::Topology_Vertex(center + uDirection * halfU - vDirection * halfV));
    vertices.push_back(MyBRep::Topology_Vertex(center + uDirection * halfU + vDirection * halfV));
    vertices.push_back(MyBRep::Topology_Vertex(center - uDirection * halfU + vDirection * halfV));

    std::vector<MyBRep::Topology_Edge> edges;
    edges.reserve(4);

    for (std::size_t index = 0; index < vertices.size(); ++index)
    {
        const std::size_t nextIndex = (index + 1) % vertices.size();
        const MyMath::Vector3 direction = vertices[nextIndex].point() - vertices[index].point();
        const double length = direction.length();

        const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> geometry(
            new MyBRep::Geometry_Line(vertices[index].point(), direction));

        edges.push_back(MyBRep::Topology_Edge(vertices[index],
                                              vertices[nextIndex],
                                              geometry,
                                              0.0,
                                              length,
                                              TestTolerance));
    }

    return MyBRep::Topology_Wire(edges);
}

void testOrientedPlane(TestContext& context)
{
    const MyMath::Vector3 origin(3.0, 4.0, 5.0); // 空间Plane原点，刻意离开世界原点验证平移后的Surface参数映射。
    const MyMath::Vector3 uDirection = MyMath::Vector3::unitY(); // U轴沿世界Y方向，使测试Plane成为x=3的YZ平面。
    const MyMath::Vector3 vDirection = MyMath::Vector3::unitZ(); // V轴沿世界Z方向，与U轴正交。
    const MyMath::Vector3 normal = MyMath::Vector3::cross(uDirection, vDirection); // U叉V得到世界+X方向Face标准法向。

    const MyMath::CoordinateSystem coordinateSystem =
        MyMath::CoordinateSystem::fromAxes(origin, uDirection, vDirection, normal);

    const MyBRep::Topology_Wire wire = createSpatialRectangle(origin, uDirection, vDirection, 6.0, 4.0);
    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(coordinateSystem, wire, TestTolerance);
    const MyBRep::FaceMesh mesh = MyBRep::PlanarFaceMesher::mesh(face, testOptions());

    context.expect(mesh.isValid(), "Oriented Plane Face mesh valid");
    context.expect(mesh.isValid() && std::fabs(meshArea(mesh) - 24.0) <= AreaTolerance, "Oriented Plane Face area");
    context.expect(mesh.isValid() && triangleWindingMatchesNormals(mesh), "Oriented Plane triangle winding matches normal");
}

void testUntrimmedPlaneRejected(TestContext& context)
{
    const MyBRep::Topology_Face trimmed = MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createRectangle(10.0, 8.0), TestTolerance);
    const MyBRep::Topology_Face untrimmed(trimmed.geometryResource());

    context.expect(!MyBRep::PlanarFaceMesher::canMesh(untrimmed), "Untrimmed infinite Plane cannot mesh");
    context.expect(MyBRep::PlanarFaceMesher::mesh(untrimmed, testOptions()).isEmpty(), "Untrimmed infinite Plane returns empty mesh");
}

}

int main()
{
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep Planar Face Mesher Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testRectangle(context);
    testRectangleHole(context);
    testMultipleHoles(context);
    testThreeNestedWires(context);
    testCircleFace(context);
    testReversedFace(context);
    testOrientedPlane(context);
    testUntrimmedPlaneRejected(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}
