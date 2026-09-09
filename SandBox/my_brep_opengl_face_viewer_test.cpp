#include <QApplication>

#include <iostream>
#include <string>
#include <vector>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"

#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Modeling/Wire/WireModeling.h"

#include "MyBRepOpenGL/Display/BRepDisplayObject.h"
#include "MyBRepOpenGL/Display/BRepDisplayStyle.h"
#include "MyBRepOpenGL/Viewer/BRepViewerWidget.h"

#include "MyOpenGL/Item/RenderItem.h"
#include "MyOpenGL/Item/RenderPart.h"
#include "MyOpenGL/Material/Material.h"
#include "MyOpenGL/Resource/BufferGeometry.h"

namespace
{

const double TestTolerance = 1.0e-8; // 平面Face构造统一使用的测试几何容差。

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

void testFaceDisplayRegistration(TestContext& context)
{
    MyBRep::Display::BRepViewerWidget viewer;

    const std::size_t baseResourceCount = viewer.resourceManager().count();
    const std::size_t baseMaterialCount = viewer.materialManager().count();
    const std::size_t baseItemCount = viewer.itemManager().count();

    std::vector<MyBRep::Topology_Wire> wires;
    wires.push_back(MyBRep::Modeling::createRectangle(10.0, 8.0));
    wires.push_back(MyBRep::Modeling::createCircle(2.0));

    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(wires, TestTolerance);

    MyBRep::Display::BRepDisplayStyle style;
    style.surface.meshing.chordTolerance = 0.01;
    style.wireframe.chordTolerance = 0.01;
    style.wireframe.lineWidth = 2.0f;

    const MyBRep::Display::BRepDisplayId id = viewer.addFace(face, "FaceDisplay", style);
    const MyBRep::Display::BRepDisplayObject object = viewer.display(id);

    context.expect(id != MyBRep::Display::InvalidBRepDisplayId, "Face display registered");
    context.expect(object.isValid(), "Face display object valid");
    context.expect(object.hasSurface(), "Face display owns surface resource pair");
    context.expect(object.hasWireframe(), "Face display owns wireframe resource pair");
    context.expect(viewer.displayCount() == 1, "Face display count");
    context.expect(viewer.resourceManager().count() == baseResourceCount + 2, "Face display registers two Geometry resources");
    context.expect(viewer.materialManager().count() == baseMaterialCount + 2, "Face display registers two Materials");
    context.expect(viewer.itemManager().count() == baseItemCount + 1, "Face display registers one RenderItem");

    RenderItem* item = viewer.itemManager().get(object.itemId);
    context.expect(item != 0 && item->partCount() == 2, "Face display uses one RenderItem with two RenderParts");

    const RenderPart* firstPart = item != 0 ? item->partAt(0) : 0;
    const RenderPart* secondPart = item != 0 ? item->partAt(1) : 0;

    context.expect(firstPart != 0 && firstPart->geometry() != 0 &&
                   firstPart->geometry()->renderType() == RenderType::Lines,
                   "Face boundary RenderPart is created first");

    context.expect(secondPart != 0 && secondPart->geometry() != 0 &&
                   secondPart->geometry()->renderType() == RenderType::Triangles,
                   "Face surface RenderPart is created second");

    context.expect(firstPart != 0 && firstPart->material() != 0 && !firstPart->material()->lightingEnabled(),
                   "Face boundary Material uses unlit color");

    context.expect(secondPart != 0 && secondPart->material() != 0 &&
                   secondPart->material()->lightingEnabled() == style.surfaceLightingEnabled,
                   "Face surface Material follows style lighting");

    context.expect(viewer.removeDisplay(id), "Face display removal succeeds");
    context.expect(viewer.displayCount() == 0, "Face display removed from BRep registry");
    context.expect(viewer.resourceManager().count() == baseResourceCount, "Face display Geometry resources released");
    context.expect(viewer.materialManager().count() == baseMaterialCount, "Face display Materials released");
    context.expect(viewer.itemManager().count() == baseItemCount, "Face display RenderItem released");
}

void testWireframeCompatibility(TestContext& context)
{
    MyBRep::Display::BRepViewerWidget viewer;
    const MyBRep::Topology_Wire wire = MyBRep::Modeling::createRectangle(6.0, 4.0);

    const MyBRep::Display::BRepDisplayId id = viewer.addWireframe(wire, "WireframeOnly");
    const MyBRep::Display::BRepDisplayObject object = viewer.display(id);

    context.expect(id != MyBRep::Display::InvalidBRepDisplayId, "Existing wireframe display still registers");
    context.expect(object.isValid(), "Wireframe display object valid");
    context.expect(!object.hasSurface(), "Wireframe display has no surface resources");
    context.expect(object.hasWireframe(), "Wireframe display keeps wireframe resources");
    context.expect(viewer.removeDisplay(id), "Wireframe display removal still succeeds");
}

void testAffineFaceDisplay(TestContext& context)
{
    MyBRep::Display::BRepViewerWidget viewer;
    const MyBRep::Topology_Face face =
        MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createRectangle(8.0, 6.0), TestTolerance);

    MyMath::Matrix4 transform = MyMath::Matrix4::identity();
    transform(0, 1) = 0.5; // XY剪切用于验证Viewer入口继续支持一般可逆仿射放置。
    transform(0, 3) = 10.0;

    const MyBRep::Display::BRepDisplayId id = viewer.addFace(face, transform, "AffineFace");

    context.expect(id != MyBRep::Display::InvalidBRepDisplayId, "General affine Face display registered");
    context.expect(viewer.display(id).hasSurface() && viewer.display(id).hasWireframe(),
                   "General affine Face display contains both parts");
    context.expect(viewer.removeDisplay(id), "General affine Face display removal succeeds");
}

void testUnsupportedFaceRejected(TestContext& context)
{
    MyBRep::Display::BRepViewerWidget viewer;

    const MyBRep::Topology_Face trimmed =
        MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createRectangle(8.0, 6.0), TestTolerance);
    const MyBRep::Topology_Face untrimmed(trimmed.geometryResource());

    const MyBRep::Display::BRepDisplayId id = viewer.addFace(untrimmed, "UntrimmedPlane");

    context.expect(id == MyBRep::Display::InvalidBRepDisplayId, "Untrimmed infinite Plane display rejected");
    context.expect(viewer.displayCount() == 0, "Rejected Face leaves no display registration");
}

}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep MyOpenGL Face Viewer Integration Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testFaceDisplayRegistration(context);
    testWireframeCompatibility(context);
    testAffineFaceDisplay(context);
    testUnsupportedFaceRejected(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}
