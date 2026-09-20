#include <QApplication>

#include <iostream>

#include "MyBRep/Instance/Face.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Modeling/Wire/WireModeling.h"
#include "MyBRepOpenGL/Display/BRepDisplayManager.h"
#include "MyBRepOpenGL/Viewer/BRepViewerWidget.h"

namespace
{

class TestContext
{
public:
    TestContext() : m_passed(0), m_failed(0) {}

    void expect(bool condition, const char* name)
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

    int passed() const { return m_passed; }
    int failed() const { return m_failed; }

private:
    int m_passed;
    int m_failed;
};

}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);
    TestContext context;
    MyBRep::Display::BRepViewerWidget viewer;

    const MyBRep::Topology_Wire rectangle = MyBRep::Modeling::createRectangle(8.0, 6.0);
    const MyBRep::Topology_Face topology = MyBRep::Modeling::createPlanarFace(rectangle, 1.0e-8);

    const MyBRep::Face first(topology);
    const MyBRep::Face second(first);

    context.expect(first.id() != second.id(), "Copied Face owns a different InstanceId");
    context.expect(viewer.displayManager().instanceCount() == 0, "Viewer identity map starts empty");

    const MyBRep::Display::BRepDisplayId firstDisplayId = viewer.addFace(first, "First");

    context.expect(firstDisplayId != MyBRep::Display::InvalidBRepDisplayId, "First Instance display is created");

    const MyBRep::Display::BRepDisplayObject firstDisplay = viewer.display(firstDisplayId);

    context.expect(viewer.displayManager().containsInstance(first.id()), "First InstanceId is registered");
    context.expect(viewer.displayManager().itemId(first.id()) == firstDisplay.itemId, "First Instance resolves its RenderItemId");
    context.expect(viewer.displayManager().instanceId(firstDisplay.itemId) == first.id(), "First RenderItem resolves its InstanceId");

    const MyBRep::Display::BRepDisplayId duplicateDisplayId = viewer.addFace(first, "DuplicateFirst");

    context.expect(duplicateDisplayId == MyBRep::Display::InvalidBRepDisplayId, "Same Instance cannot create a second RenderItem binding");
    context.expect(viewer.displayCount() == 1, "Rejected duplicate Instance does not leave a Display");
    context.expect(viewer.displayManager().instanceCount() == 1, "Rejected duplicate Instance does not change identity map");

    const MyBRep::Display::BRepDisplayId secondDisplayId = viewer.addFace(second, "Second");

    context.expect(secondDisplayId != MyBRep::Display::InvalidBRepDisplayId, "Copied Instance can create its own display");

    const MyBRep::Display::BRepDisplayObject secondDisplay = viewer.display(secondDisplayId);

    context.expect(firstDisplay.itemId != secondDisplay.itemId, "Different InstanceIds own different RenderItems");
    context.expect(viewer.displayManager().instanceCount() == 2, "Two Instance mappings are registered");
    context.expect(viewer.displayManager().itemId(second.id()) == secondDisplay.itemId, "Second Instance resolves its RenderItemId");

    context.expect(viewer.removeDisplay(firstDisplayId), "Removing first Display succeeds");
    context.expect(!viewer.displayManager().containsInstance(first.id()), "Removing Display removes first Instance mapping");
    context.expect(viewer.displayManager().containsInstance(second.id()), "Removing first Display keeps second Instance mapping");
    context.expect(viewer.displayManager().instanceCount() == 1, "One Instance mapping remains after first removal");

    context.expect(viewer.clearBRepDisplays(), "Clearing Viewer succeeds");
    context.expect(viewer.displayCount() == 0, "Viewer contains no Displays after clear");
    context.expect(viewer.displayManager().instanceCount() == 0, "Viewer identity map is empty after clear");

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}