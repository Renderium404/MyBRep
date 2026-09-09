#include <iostream>
#include <string>
#include <vector>

#include "MyMath/CoordinateSystem.h"
#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"

#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Curve/Geometry_Curve.h"
#include "MyBRep/Geometry/Curve/Geometry_Line.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Modeling/Shell/ShellModeling.h"
#include "MyBRep/Modeling/Solid/SolidModeling.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

#include "MyBRepOpenGL/Builder/BRepSolidBuilder.h"
#include "MyOpenGL/Resource/BufferGeometry.h"

namespace
{

const double TestTolerance = 1.0e-8; // 六面体共享Edge连接和Planar Face构造统一使用的几何容差。
const double ValueTolerance = 1.0e-5; // double转换为GLfloat后的显示数据比较容差。

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

MyBRep::Topology_Edge createLineEdge(const MyBRep::Topology_Vertex& startVertex, const MyBRep::Topology_Vertex& endVertex)
{
    const MyMath::Vector3 direction = endVertex.point() - startVertex.point();
    const double length = direction.length();

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> geometry(
        new MyBRep::Geometry_Line(startVertex.point(), direction));

    return MyBRep::Topology_Edge(startVertex, endVertex, geometry, 0.0, length, TestTolerance);
}

MyBRep::Topology_Face createBoxFace(const MyMath::CoordinateSystem& coordinateSystem,
                                    const MyBRep::Topology_Edge& first,
                                    const MyBRep::Topology_Edge& second,
                                    const MyBRep::Topology_Edge& third,
                                    const MyBRep::Topology_Edge& fourth)
{
    std::vector<MyBRep::Topology_Edge> edges;
    edges.push_back(first);
    edges.push_back(second);
    edges.push_back(third);
    edges.push_back(fourth);

    return MyBRep::Modeling::createPlanarFace(
        coordinateSystem,
        MyBRep::Topology_Wire(edges),
        TestTolerance);
}

MyBRep::Topology_Shell createBoxShell(const MyMath::Vector3& center, double sizeX, double sizeY, double sizeZ)
{
    const double halfX = sizeX * 0.5; // 六面体X方向半尺寸。
    const double halfY = sizeY * 0.5; // 六面体Y方向半尺寸。
    const double halfZ = sizeZ * 0.5; // 六面体Z方向半尺寸。

    const double x0 = center.x() - halfX;
    const double x1 = center.x() + halfX;
    const double y0 = center.y() - halfY;
    const double y1 = center.y() + halfY;
    const double z0 = center.z() - halfZ;
    const double z1 = center.z() + halfZ;

    // 八个共享拓扑顶点。
    const MyBRep::Topology_Vertex v000(MyMath::Vector3(x0, y0, z0));
    const MyBRep::Topology_Vertex v100(MyMath::Vector3(x1, y0, z0));
    const MyBRep::Topology_Vertex v110(MyMath::Vector3(x1, y1, z0));
    const MyBRep::Topology_Vertex v010(MyMath::Vector3(x0, y1, z0));
    const MyBRep::Topology_Vertex v001(MyMath::Vector3(x0, y0, z1));
    const MyBRep::Topology_Vertex v101(MyMath::Vector3(x1, y0, z1));
    const MyBRep::Topology_Vertex v111(MyMath::Vector3(x1, y1, z1));
    const MyBRep::Topology_Vertex v011(MyMath::Vector3(x0, y1, z1));

    // 十二条共享Topology_TEdge；相邻Face通过reversed()以相反方向使用同一Edge身份。
    const MyBRep::Topology_Edge e0 = createLineEdge(v000, v100);
    const MyBRep::Topology_Edge e1 = createLineEdge(v100, v110);
    const MyBRep::Topology_Edge e2 = createLineEdge(v110, v010);
    const MyBRep::Topology_Edge e3 = createLineEdge(v010, v000);

    const MyBRep::Topology_Edge e4 = createLineEdge(v001, v101);
    const MyBRep::Topology_Edge e5 = createLineEdge(v101, v111);
    const MyBRep::Topology_Edge e6 = createLineEdge(v111, v011);
    const MyBRep::Topology_Edge e7 = createLineEdge(v011, v001);

    const MyBRep::Topology_Edge e8 = createLineEdge(v000, v001);
    const MyBRep::Topology_Edge e9 = createLineEdge(v100, v101);
    const MyBRep::Topology_Edge e10 = createLineEdge(v110, v111);
    const MyBRep::Topology_Edge e11 = createLineEdge(v010, v011);

    const MyMath::CoordinateSystem bottomSystem =
        MyMath::CoordinateSystem::fromAxes(
            MyMath::Vector3(center.x(), center.y(), z0),
            MyMath::Vector3::unitX(),
            MyMath::Vector3(0.0, -1.0, 0.0),
            MyMath::Vector3(0.0, 0.0, -1.0));

    const MyMath::CoordinateSystem topSystem =
        MyMath::CoordinateSystem::fromAxes(
            MyMath::Vector3(center.x(), center.y(), z1),
            MyMath::Vector3::unitX(),
            MyMath::Vector3::unitY(),
            MyMath::Vector3::unitZ());

    const MyMath::CoordinateSystem frontSystem =
        MyMath::CoordinateSystem::fromAxes(
            MyMath::Vector3(center.x(), y0, center.z()),
            MyMath::Vector3::unitX(),
            MyMath::Vector3::unitZ(),
            MyMath::Vector3(0.0, -1.0, 0.0));

    const MyMath::CoordinateSystem backSystem =
        MyMath::CoordinateSystem::fromAxes(
            MyMath::Vector3(center.x(), y1, center.z()),
            MyMath::Vector3::unitZ(),
            MyMath::Vector3::unitX(),
            MyMath::Vector3::unitY());

    const MyMath::CoordinateSystem leftSystem =
        MyMath::CoordinateSystem::fromAxes(
            MyMath::Vector3(x0, center.y(), center.z()),
            MyMath::Vector3::unitZ(),
            MyMath::Vector3::unitY(),
            MyMath::Vector3(-1.0, 0.0, 0.0));

    const MyMath::CoordinateSystem rightSystem =
        MyMath::CoordinateSystem::fromAxes(
            MyMath::Vector3(x1, center.y(), center.z()),
            MyMath::Vector3::unitY(),
            MyMath::Vector3::unitZ(),
            MyMath::Vector3::unitX());

    std::vector<MyBRep::Topology_Face> faces;

    // Bottom，外法向-Z。
    faces.push_back(createBoxFace(bottomSystem, e3.reversed(), e2.reversed(), e1.reversed(), e0.reversed()));
    // Top，外法向+Z。
    faces.push_back(createBoxFace(topSystem, e4, e5, e6, e7));
    // Front，外法向-Y。
    faces.push_back(createBoxFace(frontSystem, e0, e9, e4.reversed(), e8.reversed()));
    // Back，外法向+Y。
    faces.push_back(createBoxFace(backSystem, e11, e6.reversed(), e10.reversed(), e2));
    // Left，外法向-X。
    faces.push_back(createBoxFace(leftSystem, e8, e7.reversed(), e11.reversed(), e3));
    // Right，外法向+X。
    faces.push_back(createBoxFace(rightSystem, e1, e10, e5.reversed(), e9.reversed()));

    return MyBRep::Modeling::createShell(faces);
}

MyBRep::Topology_Solid createBoxSolid(const MyMath::Vector3& center, double sizeX, double sizeY, double sizeZ)
{
    return MyBRep::Modeling::createSolid(createBoxShell(center, sizeX, sizeY, sizeZ));
}

MyMath::Vector3 geometryPosition(const BufferGeometry& geometry, unsigned int vertexIndex)
{
    const std::size_t offset = static_cast<std::size_t>(vertexIndex) * 6;
    const std::vector<GLfloat>& data = geometry.vertexData();

    return MyMath::Vector3(data[offset], data[offset + 1], data[offset + 2]);
}

MyMath::Vector3 geometryNormal(const BufferGeometry& geometry, unsigned int vertexIndex)
{
    const std::size_t offset = static_cast<std::size_t>(vertexIndex) * 6 + 3;
    const std::vector<GLfloat>& data = geometry.vertexData();

    return MyMath::Vector3(data[offset], data[offset + 1], data[offset + 2]);
}

bool allIndicesValid(const BufferGeometry& geometry)
{
    for (std::size_t index = 0; index < geometry.indexData().size(); ++index)
    {
        if (geometry.indexData()[index] >= static_cast<GLuint>(geometry.vertexCount()))
        {
            return false;
        }
    }

    return true;
}

void testDefaultOptions(TestContext& context)
{
    const MyBRep::Display::BRepSolidBuildOptions options;
    context.expect(options.isValid(), "Default Solid build options valid");
}

void testBoxTopology(TestContext& context)
{
    const MyBRep::Topology_Shell shell = createBoxShell(MyMath::Vector3::zero(), 4.0, 3.0, 2.0);
    const MyBRep::Topology_Solid solid = MyBRep::Modeling::createSolid(shell);

    context.expect(shell.isValid(), "Box Shell valid");
    context.expect(shell.isClosed(), "Box Shell closed");
    context.expect(shell.faceCount() == 6, "Box Shell has six Faces");
    context.expect(solid.isValid(), "Box Solid valid");
    context.expect(solid.shellCount() == 1, "Box Solid has one Shell");
}

void testBoxSurface(TestContext& context)
{
    const MyBRep::Topology_Solid solid = createBoxSolid(MyMath::Vector3::zero(), 4.0, 3.0, 2.0);
    BufferGeometry* geometry = MyBRep::Display::BRepSolidBuilder::buildSurface(solid, "BoxSurface");

    context.expect(geometry != 0, "Box Solid surface Geometry created");
    context.expect(geometry != 0 && geometry->renderType() == RenderType::Triangles, "Box Solid surface uses Triangles");
    context.expect(geometry != 0 && geometry->valuesPerVertex() == 6, "Box Solid surface uses Position Normal layout");
    context.expect(geometry != 0 && geometry->hasAttribute(GeometryAttribute::Position, 3), "Box Solid surface exposes Position");
    context.expect(geometry != 0 && geometry->hasAttribute(GeometryAttribute::Normal, 3), "Box Solid surface exposes Normal");
    context.expect(geometry != 0 && geometry->vertexCount() == 24, "Box Solid keeps four Face-local vertices per Face");
    context.expect(geometry != 0 && geometry->indexCount() == 36, "Box Solid contains twelve triangles");
    context.expect(geometry != 0 && allIndicesValid(*geometry), "Box Solid merged indices remain valid");

    delete geometry;
}

void testBoxBoundary(TestContext& context)
{
    const MyBRep::Topology_Solid solid = createBoxSolid(MyMath::Vector3::zero(), 4.0, 3.0, 2.0);
    BufferGeometry* geometry = MyBRep::Display::BRepSolidBuilder::buildBoundary(solid, "BoxBoundary");

    context.expect(geometry != 0, "Box Solid boundary Geometry created");
    context.expect(geometry != 0 && geometry->renderType() == RenderType::Lines, "Box Solid boundary uses Lines");

    // 六个矩形Face共有24个Edge Use，但实际只共享12条Topology_TEdge，因此唯一边界为12条Line，对应24个Index。
    context.expect(geometry != 0 && geometry->indexCount() == 24, "Box Solid emits twelve shared Topology_Edges once");

    delete geometry;
}

void testReversedSolid(TestContext& context)
{
    const MyBRep::Topology_Solid forwardSolid = createBoxSolid(MyMath::Vector3::zero(), 4.0, 3.0, 2.0);
    const MyBRep::Topology_Solid reversedSolid = forwardSolid.reversed();

    BufferGeometry* forwardSurface = MyBRep::Display::BRepSolidBuilder::buildSurface(forwardSolid, "ForwardBoxSurface");
    BufferGeometry* reversedSurface = MyBRep::Display::BRepSolidBuilder::buildSurface(reversedSolid, "ReversedBoxSurface");
    BufferGeometry* forwardBoundary = MyBRep::Display::BRepSolidBuilder::buildBoundary(forwardSolid, "ForwardBoxBoundary");
    BufferGeometry* reversedBoundary = MyBRep::Display::BRepSolidBuilder::buildBoundary(reversedSolid, "ReversedBoxBoundary");

    context.expect(forwardSurface != 0 && reversedSurface != 0, "Forward and Reversed Solid surfaces created");
    context.expect(forwardSurface != 0 && reversedSurface != 0 &&
                   forwardSurface->vertexCount() == reversedSurface->vertexCount() &&
                   forwardSurface->indexCount() == reversedSurface->indexCount(),
                   "Reversed Solid preserves surface mesh size");

    bool normalsOpposite = false;

    if (forwardSurface != 0 && reversedSurface != 0 &&
        forwardSurface->vertexCount() > 0 && reversedSurface->vertexCount() > 0)
    {
        normalsOpposite =
            MyMath::Vector3::dot(geometryNormal(*forwardSurface, 0), geometryNormal(*reversedSurface, 0)) < -0.999999;
    }

    context.expect(normalsOpposite, "Reversed Solid reverses Face normals");
    context.expect(forwardBoundary != 0 && reversedBoundary != 0 &&
                   forwardBoundary->indexCount() == reversedBoundary->indexCount(),
                   "Reversed Solid preserves unique boundary count");

    delete forwardSurface;
    delete reversedSurface;
    delete forwardBoundary;
    delete reversedBoundary;
}

void testGeneralAffinePlacement(TestContext& context)
{
    const MyBRep::Topology_Solid solid = createBoxSolid(MyMath::Vector3::zero(), 4.0, 3.0, 2.0);

    MyMath::Matrix4 transform = MyMath::Matrix4::identity();
    transform(0, 1) = 0.35; // XY剪切验证完整Solid的一般仿射位置烘焙。
    transform(2, 0) = 0.20; // Z随X变化，验证不同Face法向的逆转置处理仍由Face Builder保持。
    transform(0, 3) = 7.0;  // X方向平移7个模型单位。
    transform(1, 3) = -3.0; // Y方向平移-3个模型单位。

    BufferGeometry* localSurface = MyBRep::Display::BRepSolidBuilder::buildSurface(solid, "LocalBoxSurface");
    BufferGeometry* transformedSurface = MyBRep::Display::BRepSolidBuilder::buildSurface(solid, transform, "TransformedBoxSurface");
    BufferGeometry* transformedBoundary = MyBRep::Display::BRepSolidBuilder::buildBoundary(solid, transform, "TransformedBoxBoundary");

    const bool created =
        localSurface != 0 &&
        transformedSurface != 0 &&
        transformedBoundary != 0 &&
        localSurface->vertexCount() == transformedSurface->vertexCount();

    context.expect(created, "General affine Solid Surface and Boundary created");

    bool positionsCorrect = created;

    if (created)
    {
        for (int index = 0; index < localSurface->vertexCount(); ++index)
        {
            const MyMath::Vector3 localPosition = geometryPosition(*localSurface, static_cast<unsigned int>(index));
            const MyMath::Vector3 expectedPosition = transform.transformPoint(localPosition);
            const MyMath::Vector3 actualPosition = geometryPosition(*transformedSurface, static_cast<unsigned int>(index));

            if (!actualPosition.isEqualTo(expectedPosition, ValueTolerance))
            {
                positionsCorrect = false;
                break;
            }
        }
    }

    context.expect(positionsCorrect, "General affine placement is baked into all Solid surface vertices");
    context.expect(transformedBoundary != 0 && transformedBoundary->indexCount() == 24,
                   "General affine Solid boundary preserves twelve unique Edges");

    delete localSurface;
    delete transformedSurface;
    delete transformedBoundary;
}

void testMultipleShells(TestContext& context)
{
    std::vector<MyBRep::Topology_Shell> shells;
    shells.push_back(createBoxShell(MyMath::Vector3(-5.0, 0.0, 0.0), 2.0, 2.0, 2.0));
    shells.push_back(createBoxShell(MyMath::Vector3(5.0, 0.0, 0.0), 2.0, 2.0, 2.0));

    const MyBRep::Topology_Solid solid = MyBRep::Modeling::createSolid(shells);

    context.expect(solid.isValid(), "Multiple-Shell Solid valid");
    context.expect(solid.shellCount() == 2, "Multiple-Shell Solid shell count");

    BufferGeometry* surface = MyBRep::Display::BRepSolidBuilder::buildSurface(solid, "MultipleShellSurface");
    BufferGeometry* boundary = MyBRep::Display::BRepSolidBuilder::buildBoundary(solid, "MultipleShellBoundary");

    context.expect(surface != 0 && surface->vertexCount() == 48, "Multiple-Shell Solid merges both Shell surface vertices");
    context.expect(surface != 0 && surface->indexCount() == 72, "Multiple-Shell Solid merges twenty-four triangles");
    context.expect(boundary != 0 && boundary->indexCount() == 48, "Multiple-Shell Solid emits twenty-four unique Edges");

    delete surface;
    delete boundary;
}

void testInvalidSolidRejected(TestContext& context)
{
    const MyBRep::Topology_Solid solid;

    BufferGeometry* surface = MyBRep::Display::BRepSolidBuilder::buildSurface(solid, "InvalidSolidSurface");
    BufferGeometry* boundary = MyBRep::Display::BRepSolidBuilder::buildBoundary(solid, "InvalidSolidBoundary");

    context.expect(surface == 0, "Invalid Solid surface rejected");
    context.expect(boundary == 0, "Invalid Solid boundary rejected");

    delete surface;
    delete boundary;
}

}

int main()
{
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep MyOpenGL Solid Builder Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testDefaultOptions(context);
    testBoxTopology(context);
    testBoxSurface(context);
    testBoxBoundary(context);
    testReversedSolid(context);
    testGeneralAffinePlacement(context);
    testMultipleShells(context);
    testInvalidSolidRejected(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}
