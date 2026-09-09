#include <QApplication>

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
#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

#include "MyBRepOpenGL/Display/BRepDisplayObject.h"
#include "MyBRepOpenGL/Viewer/BRepViewerWidget.h"

#include "MyOpenGL/Item/RenderItem.h"
#include "MyOpenGL/Item/RenderPart.h"
#include "MyOpenGL/Resource/Geometry.h"

namespace
{

const double TestTolerance = 1.0e-8; // Shell测试中Edge连接和Planar Face构造统一使用的几何容差。

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

std::vector<MyBRep::Topology_Face> createFoldedFaces()
{
    const MyBRep::Topology_Vertex leftBottom(MyMath::Vector3(-4.0, -2.0, 0.0));
    const MyBRep::Topology_Vertex sharedBottom(MyMath::Vector3(0.0, -2.0, 0.0));
    const MyBRep::Topology_Vertex sharedTop(MyMath::Vector3(0.0, 2.0, 0.0));
    const MyBRep::Topology_Vertex leftTop(MyMath::Vector3(-4.0, 2.0, 0.0));

    const MyBRep::Topology_Vertex outerBottom(MyMath::Vector3(0.0, -2.0, 4.0));
    const MyBRep::Topology_Vertex outerTop(MyMath::Vector3(0.0, 2.0, 4.0));

    const MyBRep::Topology_Edge leftBottomEdge = createLineEdge(leftBottom, sharedBottom);
    const MyBRep::Topology_Edge sharedEdge = createLineEdge(sharedBottom, sharedTop);
    const MyBRep::Topology_Edge leftTopEdge = createLineEdge(sharedTop, leftTop);
    const MyBRep::Topology_Edge leftSideEdge = createLineEdge(leftTop, leftBottom);

    const MyBRep::Topology_Edge foldBottomEdge = createLineEdge(sharedBottom, outerBottom);
    const MyBRep::Topology_Edge foldOuterEdge = createLineEdge(outerBottom, outerTop);
    const MyBRep::Topology_Edge foldTopEdge = createLineEdge(outerTop, sharedTop);

    std::vector<MyBRep::Topology_Edge> leftEdges;
    leftEdges.push_back(leftBottomEdge);
    leftEdges.push_back(sharedEdge);
    leftEdges.push_back(leftTopEdge);
    leftEdges.push_back(leftSideEdge);

    std::vector<MyBRep::Topology_Edge> foldEdges;
    foldEdges.push_back(foldBottomEdge);
    foldEdges.push_back(foldOuterEdge);
    foldEdges.push_back(foldTopEdge);
    foldEdges.push_back(sharedEdge.reversed());

    const MyBRep::Topology_Wire leftWire(leftEdges);
    const MyBRep::Topology_Wire foldWire(foldEdges);

    const MyBRep::Topology_Face leftFace = MyBRep::Modeling::createPlanarFace(leftWire, TestTolerance);

    const MyMath::Vector3 uDirection = MyMath::Vector3::unitZ();
    const MyMath::Vector3 vDirection = MyMath::Vector3::unitY();
    const MyMath::Vector3 normal = MyMath::Vector3::cross(uDirection, vDirection);

    const MyMath::CoordinateSystem foldCoordinateSystem =
        MyMath::CoordinateSystem::fromAxes(MyMath::Vector3::zero(), uDirection, vDirection, normal);

    const MyBRep::Topology_Face foldFace =
        MyBRep::Modeling::createPlanarFace(foldCoordinateSystem, foldWire, TestTolerance);

    std::vector<MyBRep::Topology_Face> faces;
    faces.push_back(leftFace);
    faces.push_back(foldFace);
    return faces;
}

MyBRep::Topology_Shell createFoldedShell()
{
    return MyBRep::Modeling::createShell(createFoldedFaces());
}

void testShellDisplayRegistration(TestContext& context)
{
    MyBRep::Display::BRepViewerWidget viewer;

    const std::size_t baseResourceCount = viewer.resourceManager().count();
    const std::size_t baseMaterialCount = viewer.materialManager().count();
    const std::size_t baseItemCount = viewer.itemManager().count();

    const MyBRep::Topology_Shell shell = createFoldedShell();
    const MyBRep::Display::BRepDisplayId id = viewer.addShell(shell, "FoldedShell");
    const MyBRep::Display::BRepDisplayObject object = viewer.display(id);

    context.expect(id != MyBRep::Display::InvalidBRepDisplayId, "Shell display registered");
    context.expect(object.isValid(), "Shell display object valid");
    context.expect(object.hasSurface(), "Shell display owns surface resource pair");
    context.expect(object.hasWireframe(), "Shell display owns wireframe resource pair");
    context.expect(viewer.displayCount() == 1, "Shell display count");
    context.expect(viewer.resourceManager().count() == baseResourceCount + 2, "Shell display registers two Geometry resources");
    context.expect(viewer.materialManager().count() == baseMaterialCount + 2, "Shell display registers two Materials");
    context.expect(viewer.itemManager().count() == baseItemCount + 1, "Shell display registers one RenderItem");

    RenderItem* item = viewer.itemManager().get(object.itemId);
    context.expect(item != 0 && item->partCount() == 2, "Shell display uses one RenderItem with two RenderParts");

    const RenderPart* boundaryPart = item != 0 ? item->partAt(0) : 0;
    const RenderPart* surfacePart = item != 0 ? item->partAt(1) : 0;

    context.expect(boundaryPart != 0 && boundaryPart->geometry() != 0 &&
                   boundaryPart->geometry()->renderType() == RenderType::Lines,
                   "Shell boundary RenderPart is created first");

    context.expect(surfacePart != 0 && surfacePart->geometry() != 0 &&
                   surfacePart->geometry()->renderType() == RenderType::Triangles,
                   "Shell surface RenderPart is created second");

    context.expect(boundaryPart != 0 && boundaryPart->geometry() != 0 &&
                   boundaryPart->geometry()->indexCount() == 14,
                   "Shell Viewer keeps shared Edge boundary deduplication");

    context.expect(surfacePart != 0 && surfacePart->geometry() != 0 &&
                   surfacePart->geometry()->indexCount() == 12,
                   "Shell Viewer keeps merged Face surface triangles");

    context.expect(viewer.removeDisplay(id), "Shell display removal succeeds");
    context.expect(viewer.displayCount() == 0, "Shell display removed from BRep registry");
    context.expect(viewer.resourceManager().count() == baseResourceCount, "Shell display Geometry resources released");
    context.expect(viewer.materialManager().count() == baseMaterialCount, "Shell display Materials released");
    context.expect(viewer.itemManager().count() == baseItemCount, "Shell display RenderItem released");
}

void testShellInstance(TestContext& context)
{
    MyBRep::Display::BRepViewerWidget viewer;

    MyMath::Matrix4 transform = MyMath::Matrix4::identity();
    transform(0, 1) = 0.4;  // XY剪切用于验证Shell Instance保留一般仿射放置。
    transform(2, 0) = 0.2;  // Z随X变化，验证Surface与Boundary共用同一仿射放置。
    transform(0, 3) = 8.0;  // X方向平移8个模型单位。

    const MyBRep::Shell shell = MyBRep::Modeling::makeShell(createFoldedFaces(), transform);
    const MyBRep::Display::BRepDisplayId id = viewer.addShell(shell, "ShellInstance");

    context.expect(id != MyBRep::Display::InvalidBRepDisplayId, "Shell Instance display registered");
    context.expect(viewer.display(id).hasSurface() && viewer.display(id).hasWireframe(),
                   "Shell Instance display contains Surface and Boundary");
    context.expect(viewer.removeDisplay(id), "Shell Instance display removal succeeds");
}

void testReversedShell(TestContext& context)
{
    MyBRep::Display::BRepViewerWidget viewer;
    const MyBRep::Topology_Shell shell = createFoldedShell().reversed();

    const MyBRep::Display::BRepDisplayId id = viewer.addShell(shell, "ReversedShell");

    context.expect(id != MyBRep::Display::InvalidBRepDisplayId, "Reversed Shell display registered");
    context.expect(viewer.display(id).isValid(), "Reversed Shell display object valid");
    context.expect(viewer.removeDisplay(id), "Reversed Shell display removal succeeds");
}

void testInvalidShellRejected(TestContext& context)
{
    MyBRep::Display::BRepViewerWidget viewer;
    const MyBRep::Topology_Shell shell;

    const MyBRep::Display::BRepDisplayId id = viewer.addShell(shell, "InvalidShell");

    context.expect(id == MyBRep::Display::InvalidBRepDisplayId, "Invalid Shell display rejected");
    context.expect(viewer.displayCount() == 0, "Rejected Shell leaves no display registration");
}

}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep MyOpenGL Shell Viewer Integration Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testShellDisplayRegistration(context);
    testShellInstance(context);
    testReversedShell(context);
    testInvalidShellRejected(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}