#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"

#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Curve/Geometry_Curve.h"
#include "MyBRep/Geometry/Curve/Geometry_Line.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Modeling/Shell/ShellModeling.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

#include "MyBRepOpenGL/Builder/BRepShellBuilder.h"
#include "MyOpenGL/Resource/BufferGeometry.h"

namespace
{

const double TestTolerance = 1.0e-8; // 测试拓扑连接和Planar Face构造统一使用的几何容差。
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

MyBRep::Topology_Shell createTwoFaceShell()
{
    const MyBRep::Topology_Vertex leftBottom(MyMath::Vector3(-2.0, -1.0, 0.0));
    const MyBRep::Topology_Vertex middleBottom(MyMath::Vector3(0.0, -1.0, 0.0));
    const MyBRep::Topology_Vertex middleTop(MyMath::Vector3(0.0, 1.0, 0.0));
    const MyBRep::Topology_Vertex leftTop(MyMath::Vector3(-2.0, 1.0, 0.0));
    const MyBRep::Topology_Vertex rightBottom(MyMath::Vector3(2.0, -1.0, 0.0));
    const MyBRep::Topology_Vertex rightTop(MyMath::Vector3(2.0, 1.0, 0.0));

    const MyBRep::Topology_Edge leftBottomEdge = createLineEdge(leftBottom, middleBottom);
    const MyBRep::Topology_Edge sharedEdge = createLineEdge(middleBottom, middleTop);
    const MyBRep::Topology_Edge leftTopEdge = createLineEdge(middleTop, leftTop);
    const MyBRep::Topology_Edge leftSideEdge = createLineEdge(leftTop, leftBottom);

    const MyBRep::Topology_Edge rightBottomEdge = createLineEdge(middleBottom, rightBottom);
    const MyBRep::Topology_Edge rightSideEdge = createLineEdge(rightBottom, rightTop);
    const MyBRep::Topology_Edge rightTopEdge = createLineEdge(rightTop, middleTop);

    std::vector<MyBRep::Topology_Edge> leftEdges;
    leftEdges.push_back(leftBottomEdge);
    leftEdges.push_back(sharedEdge);
    leftEdges.push_back(leftTopEdge);
    leftEdges.push_back(leftSideEdge);

    std::vector<MyBRep::Topology_Edge> rightEdges;
    rightEdges.push_back(rightBottomEdge);
    rightEdges.push_back(rightSideEdge);
    rightEdges.push_back(rightTopEdge);
    rightEdges.push_back(sharedEdge.reversed());

    const MyBRep::Topology_Wire leftWire(leftEdges);
    const MyBRep::Topology_Wire rightWire(rightEdges);

    const MyBRep::Topology_Face leftFace = MyBRep::Modeling::createPlanarFace(leftWire, TestTolerance);
    const MyBRep::Topology_Face rightFace = MyBRep::Modeling::createPlanarFace(rightWire, TestTolerance);

    std::vector<MyBRep::Topology_Face> faces;
    faces.push_back(leftFace);
    faces.push_back(rightFace);

    return MyBRep::Modeling::createShell(faces);
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
    const MyBRep::Display::BRepShellBuildOptions options;
    context.expect(options.isValid(), "Default Shell build options valid");
}

void testTwoFaceShellSetup(TestContext& context)
{
    const MyBRep::Topology_Shell shell = createTwoFaceShell();

    context.expect(shell.isValid(), "Two-Face Shell valid");
    context.expect(shell.faceCount() == 2, "Two-Face Shell face count");
    context.expect(!shell.isClosed(), "Two-Face Shell remains open");
}

void testSurfaceMerge(TestContext& context)
{
    const MyBRep::Topology_Shell shell = createTwoFaceShell();
    BufferGeometry* geometry = MyBRep::Display::BRepShellBuilder::buildSurface(shell, "TwoFaceShellSurface");

    context.expect(geometry != 0, "Shell surface Geometry created");
    context.expect(geometry != 0 && geometry->renderType() == RenderType::Triangles, "Shell surface uses Triangles render type");
    context.expect(geometry != 0 && geometry->valuesPerVertex() == 6, "Shell surface uses Position Normal layout");
    context.expect(geometry != 0 && geometry->hasAttribute(GeometryAttribute::Position, 3), "Shell surface exposes Position attribute");
    context.expect(geometry != 0 && geometry->hasAttribute(GeometryAttribute::Normal, 3), "Shell surface exposes Normal attribute");
    context.expect(geometry != 0 && geometry->vertexCount() == 8, "Shell surface keeps Face-local vertices");
    context.expect(geometry != 0 && geometry->indexCount() == 12, "Shell surface merges four triangles");
    context.expect(geometry != 0 && allIndicesValid(*geometry), "Shell surface merged indices remain valid");

    bool secondFaceOffsetApplied = false;

    if (geometry != 0)
    {
        for (std::size_t index = 0; index < geometry->indexData().size(); ++index)
        {
            if (geometry->indexData()[index] >= 4)
            {
                secondFaceOffsetApplied = true;
                break;
            }
        }
    }

    context.expect(secondFaceOffsetApplied, "Shell surface applies Face vertex index offset");

    delete geometry;
}

void testSharedBoundaryDeduplication(TestContext& context)
{
    const MyBRep::Topology_Shell shell = createTwoFaceShell();
    BufferGeometry* geometry = MyBRep::Display::BRepShellBuilder::buildBoundary(shell, "TwoFaceShellBoundary");

    context.expect(geometry != 0, "Shell boundary Geometry created");
    context.expect(geometry != 0 && geometry->renderType() == RenderType::Lines, "Shell boundary uses Lines render type");

    // 两个矩形共有8个Edge Use，但中间公共Edge共享同一Topology_TEdge身份，因此唯一Edge数量为7，对应14个Line Index。
    context.expect(geometry != 0 && geometry->indexCount() == 14, "Shell shared Topology_Edge is emitted once");

    delete geometry;
}

void testReversedShell(TestContext& context)
{
    const MyBRep::Topology_Shell forwardShell = createTwoFaceShell();
    const MyBRep::Topology_Shell reversedShell = forwardShell.reversed();

    BufferGeometry* forwardSurface = MyBRep::Display::BRepShellBuilder::buildSurface(forwardShell, "ForwardShellSurface");
    BufferGeometry* reversedSurface = MyBRep::Display::BRepShellBuilder::buildSurface(reversedShell, "ReversedShellSurface");
    BufferGeometry* forwardBoundary = MyBRep::Display::BRepShellBuilder::buildBoundary(forwardShell, "ForwardShellBoundary");
    BufferGeometry* reversedBoundary = MyBRep::Display::BRepShellBuilder::buildBoundary(reversedShell, "ReversedShellBoundary");

    context.expect(forwardSurface != 0 && reversedSurface != 0, "Forward and Reversed Shell surfaces created");
    context.expect(forwardSurface != 0 && reversedSurface != 0 &&
                   forwardSurface->vertexCount() == reversedSurface->vertexCount() &&
                   forwardSurface->indexCount() == reversedSurface->indexCount(),
                   "Reversed Shell preserves surface mesh size");

    bool normalsOpposite = false;

    if (forwardSurface != 0 && reversedSurface != 0 &&
        forwardSurface->vertexCount() > 0 && reversedSurface->vertexCount() > 0)
    {
        normalsOpposite =
            MyMath::Vector3::dot(geometryNormal(*forwardSurface, 0), geometryNormal(*reversedSurface, 0)) < -0.999999;
    }

    context.expect(normalsOpposite, "Reversed Shell reverses Face normals");
    context.expect(forwardBoundary != 0 && reversedBoundary != 0 &&
                   forwardBoundary->indexCount() == reversedBoundary->indexCount(),
                   "Reversed Shell preserves unique boundary segment count");

    delete forwardSurface;
    delete reversedSurface;
    delete forwardBoundary;
    delete reversedBoundary;
}

void testGeneralAffinePlacement(TestContext& context)
{
    const MyBRep::Topology_Shell shell = createTwoFaceShell();

    MyMath::Matrix4 transform = MyMath::Matrix4::identity();
    transform(0, 1) = 0.5;  // XY剪切用于验证Shell Surface与Boundary共享一般仿射放置。
    transform(2, 0) = 0.25; // Z随X变化，使原XY平面Shell变为倾斜平面。
    transform(0, 3) = 7.0;  // X方向平移7个模型单位，避免只验证纯线性变换。
    transform(1, 3) = -3.0; // Y方向平移-3个模型单位。

    BufferGeometry* localSurface = MyBRep::Display::BRepShellBuilder::buildSurface(shell, "LocalShellSurface");
    BufferGeometry* transformedSurface = MyBRep::Display::BRepShellBuilder::buildSurface(shell, transform, "TransformedShellSurface");
    BufferGeometry* transformedBoundary = MyBRep::Display::BRepShellBuilder::buildBoundary(shell, transform, "TransformedShellBoundary");

    const bool created =
        localSurface != 0 &&
        transformedSurface != 0 &&
        transformedBoundary != 0 &&
        localSurface->vertexCount() == transformedSurface->vertexCount();

    context.expect(created, "General affine Shell Surface and Boundary created");

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

    context.expect(positionsCorrect, "General affine placement is baked into Shell surface vertices");
    context.expect(transformedBoundary != 0 && transformedBoundary->indexCount() == 14,
                   "General affine Shell boundary keeps shared Edge deduplication");

    delete localSurface;
    delete transformedSurface;
    delete transformedBoundary;
}

void testInvalidShellRejected(TestContext& context)
{
    const MyBRep::Topology_Shell shell;

    BufferGeometry* surface = MyBRep::Display::BRepShellBuilder::buildSurface(shell, "InvalidShellSurface");
    BufferGeometry* boundary = MyBRep::Display::BRepShellBuilder::buildBoundary(shell, "InvalidShellBoundary");

    context.expect(surface == 0, "Invalid Shell surface rejected");
    context.expect(boundary == 0, "Invalid Shell boundary rejected");

    delete surface;
    delete boundary;
}

}

int main()
{
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep MyOpenGL Shell Builder Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testDefaultOptions(context);
    testTwoFaceShellSetup(context);
    testSurfaceMerge(context);
    testSharedBoundaryDeduplication(context);
    testReversedShell(context);
    testGeneralAffinePlacement(context);
    testInvalidShellRejected(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}
