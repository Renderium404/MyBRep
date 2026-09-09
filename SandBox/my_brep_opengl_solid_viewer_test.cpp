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
#include "MyBRep/Modeling/Solid/SolidModeling.h"
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

const double TestTolerance = 1.0e-8; // 六面体共享Edge连接和Planar Face构造统一使用的几何容差。

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

    const MyBRep::Topology_Vertex v000(MyMath::Vector3(x0, y0, z0));
    const MyBRep::Topology_Vertex v100(MyMath::Vector3(x1, y0, z0));
    const MyBRep::Topology_Vertex v110(MyMath::Vector3(x1, y1, z0));
    const MyBRep::Topology_Vertex v010(MyMath::Vector3(x0, y1, z0));
    const MyBRep::Topology_Vertex v001(MyMath::Vector3(x0, y0, z1));
    const MyBRep::Topology_Vertex v101(MyMath::Vector3(x1, y0, z1));
    const MyBRep::Topology_Vertex v111(MyMath::Vector3(x1, y1, z1));
    const MyBRep::Topology_Vertex v011(MyMath::Vector3(x0, y1, z1));

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
    faces.push_back(createBoxFace(bottomSystem, e3.reversed(), e2.reversed(), e1.reversed(), e0.reversed()));
    faces.push_back(createBoxFace(topSystem, e4, e5, e6, e7));
    faces.push_back(createBoxFace(frontSystem, e0, e9, e4.reversed(), e8.reversed()));
    faces.push_back(createBoxFace(backSystem, e11, e6.reversed(), e10.reversed(), e2));
    faces.push_back(createBoxFace(leftSystem, e8, e7.reversed(), e11.reversed(), e3));
    faces.push_back(createBoxFace(rightSystem, e1, e10, e5.reversed(), e9.reversed()));

    return MyBRep::Modeling::createShell(faces);
}

MyBRep::Topology_Solid createBoxSolid(const MyMath::Vector3& center, double sizeX, double sizeY, double sizeZ)
{
    return MyBRep::Modeling::createSolid(createBoxShell(center, sizeX, sizeY, sizeZ));
}


void testSolidDisplayRegistration(TestContext& context)
{
    MyBRep::Display::BRepViewerWidget viewer;

    const std::size_t baseResourceCount = viewer.resourceManager().count();
    const std::size_t baseMaterialCount = viewer.materialManager().count();
    const std::size_t baseItemCount = viewer.itemManager().count();

    const MyBRep::Topology_Solid solid = createBoxSolid(MyMath::Vector3::zero(), 8.0, 6.0, 4.0);
    const MyBRep::Display::BRepDisplayId id = viewer.addSolid(solid, "BoxSolid");
    const MyBRep::Display::BRepDisplayObject object = viewer.display(id);

    context.expect(id != MyBRep::Display::InvalidBRepDisplayId, "Solid display registered");
    context.expect(object.isValid(), "Solid display object valid");
    context.expect(object.hasSurface(), "Solid display owns surface resource pair");
    context.expect(object.hasWireframe(), "Solid display owns boundary resource pair");
    context.expect(viewer.displayCount() == 1, "Solid display count");
    context.expect(viewer.resourceManager().count() == baseResourceCount + 2, "Solid display registers two Geometry resources");
    context.expect(viewer.materialManager().count() == baseMaterialCount + 2, "Solid display registers two Materials");
    context.expect(viewer.itemManager().count() == baseItemCount + 1, "Solid display registers one RenderItem");

    RenderItem* item = viewer.itemManager().get(object.itemId);
    context.expect(item != 0 && item->partCount() == 2, "Solid display uses one RenderItem with two RenderParts");

    const RenderPart* boundaryPart = item != 0 ? item->partAt(0) : 0;
    const RenderPart* surfacePart = item != 0 ? item->partAt(1) : 0;

    context.expect(boundaryPart != 0 && boundaryPart->geometry() != 0 &&
                   boundaryPart->geometry()->renderType() == RenderType::Lines,
                   "Solid boundary RenderPart is created first");

    context.expect(surfacePart != 0 && surfacePart->geometry() != 0 &&
                   surfacePart->geometry()->renderType() == RenderType::Triangles,
                   "Solid surface RenderPart is created second");

    context.expect(boundaryPart != 0 && boundaryPart->geometry() != 0 &&
                   boundaryPart->geometry()->indexCount() == 24,
                   "Solid Viewer keeps twelve unique shared Edges");

    context.expect(surfacePart != 0 && surfacePart->geometry() != 0 &&
                   surfacePart->geometry()->indexCount() == 36,
                   "Solid Viewer keeps twelve surface triangles");

    context.expect(viewer.removeDisplay(id), "Solid display removal succeeds");
    context.expect(viewer.displayCount() == 0, "Solid display removed from BRep registry");
    context.expect(viewer.resourceManager().count() == baseResourceCount, "Solid display Geometry resources released");
    context.expect(viewer.materialManager().count() == baseMaterialCount, "Solid display Materials released");
    context.expect(viewer.itemManager().count() == baseItemCount, "Solid display RenderItem released");
}

void testSolidInstance(TestContext& context)
{
    MyBRep::Display::BRepViewerWidget viewer;
    const MyBRep::Topology_Shell shell = createBoxShell(MyMath::Vector3::zero(), 8.0, 6.0, 4.0);

    MyMath::Matrix4 transform = MyMath::Matrix4::identity();
    transform(0, 1) = 0.35; // XY剪切用于验证Solid Instance保留一般可逆仿射放置。
    transform(2, 0) = 0.20; // Z随X变化，验证全部Face Surface和Boundary使用同一空间放置。
    transform(0, 3) = 12.0; // X方向平移12个模型单位。

    const MyBRep::Solid solid = MyBRep::Modeling::makeSolid(shell, transform);
    const MyBRep::Display::BRepDisplayId id = viewer.addSolid(solid, "SolidInstance");

    context.expect(id != MyBRep::Display::InvalidBRepDisplayId, "Solid Instance display registered");
    context.expect(viewer.display(id).hasSurface() && viewer.display(id).hasWireframe(),
                   "Solid Instance display contains Surface and Boundary");

    RenderItem* item = viewer.itemManager().get(viewer.display(id).itemId);
    const RenderPart* boundaryPart = item != 0 ? item->partAt(0) : 0;
    const RenderPart* surfacePart = item != 0 ? item->partAt(1) : 0;

    context.expect(boundaryPart != 0 && boundaryPart->geometry() != 0 &&
                   boundaryPart->geometry()->indexCount() == 24,
                   "Solid Instance keeps twelve unique Edges");

    context.expect(surfacePart != 0 && surfacePart->geometry() != 0 &&
                   surfacePart->geometry()->indexCount() == 36,
                   "Solid Instance keeps twelve surface triangles");

    context.expect(viewer.removeDisplay(id), "Solid Instance display removal succeeds");
}

void testReversedSolid(TestContext& context)
{
    MyBRep::Display::BRepViewerWidget viewer;
    const MyBRep::Topology_Solid solid = createBoxSolid(MyMath::Vector3::zero(), 8.0, 6.0, 4.0).reversed();

    const MyBRep::Display::BRepDisplayId id = viewer.addSolid(solid, "ReversedSolid");

    context.expect(id != MyBRep::Display::InvalidBRepDisplayId, "Reversed Solid display registered");
    context.expect(viewer.display(id).isValid(), "Reversed Solid display object valid");
    context.expect(viewer.removeDisplay(id), "Reversed Solid display removal succeeds");
}

void testWireframeCompatibility(TestContext& context)
{
    MyBRep::Display::BRepViewerWidget viewer;
    const MyBRep::Topology_Solid solid = createBoxSolid(MyMath::Vector3::zero(), 8.0, 6.0, 4.0);

    const MyBRep::Display::BRepDisplayId id = viewer.addWireframe(solid, "SolidWireframeOnly");
    const MyBRep::Display::BRepDisplayObject object = viewer.display(id);

    context.expect(id != MyBRep::Display::InvalidBRepDisplayId, "Existing Solid wireframe display still registers");
    context.expect(object.isValid(), "Solid wireframe display object valid");
    context.expect(!object.hasSurface(), "Solid wireframe display has no surface resources");
    context.expect(object.hasWireframe(), "Solid wireframe display keeps boundary resources");
    context.expect(viewer.removeDisplay(id), "Solid wireframe display removal still succeeds");
}

void testInvalidSolidRejected(TestContext& context)
{
    MyBRep::Display::BRepViewerWidget viewer;
    const MyBRep::Topology_Solid solid;

    const MyBRep::Display::BRepDisplayId id = viewer.addSolid(solid, "InvalidSolid");

    context.expect(id == MyBRep::Display::InvalidBRepDisplayId, "Invalid Solid display rejected");
    context.expect(viewer.displayCount() == 0, "Rejected Solid leaves no display registration");
}

}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep MyOpenGL Solid Viewer Integration Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testSolidDisplayRegistration(context);
    testSolidInstance(context);
    testReversedSolid(context);
    testWireframeCompatibility(context);
    testInvalidSolidRejected(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}
