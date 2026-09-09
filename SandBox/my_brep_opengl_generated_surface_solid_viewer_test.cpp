#include <QApplication>

#include <iostream>
#include <string>

#include "GeneratedSurfaceSolidFixtures.h"

#include "MyBRepOpenGL/Display/BRepDisplayObject.h"
#include "MyBRepOpenGL/Display/BRepDisplayStyle.h"
#include "MyBRepOpenGL/Viewer/BRepViewerWidget.h"

namespace
{

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

void testViewer(TestContext& context, MyBRep::Display::BRepViewerWidget& viewer)
{
    const GeneratedSurfaceSolidFixtures::ExtrusionSolidFixture extrusion =
        GeneratedSurfaceSolidFixtures::createExtrusionSolid(3.0, -2.0, 2.0);
    const GeneratedSurfaceSolidFixtures::RevolutionSolidFixture revolution =
        GeneratedSurfaceSolidFixtures::createRevolutionTorusSolid(5.0, 1.5);

    MyBRep::Display::BRepDisplayStyle extrusionStyle;
    extrusionStyle.surface.extrudedMeshing.boundaryChordTolerance = 0.04;
    extrusionStyle.surface.extrudedMeshing.surfaceChordTolerance = 0.03;
    extrusionStyle.surface.extrudedMeshing.minimumBoundarySubdivisionDepth = 1;

    MyBRep::Display::BRepDisplayStyle revolutionStyle;
    revolutionStyle.surface.revolvedMeshing.boundaryChordTolerance = 0.05;
    revolutionStyle.surface.revolvedMeshing.surfaceChordTolerance = 0.04;
    revolutionStyle.surface.revolvedMeshing.minimumBoundarySubdivisionDepth = 1;

    const MyBRep::Display::BRepDisplayId extrusionId =
        viewer.addSolid(extrusion.solid, "ExtrusionGeneratedSolid", extrusionStyle);
    const MyBRep::Display::BRepDisplayId revolutionId =
        viewer.addSolid(revolution.solid, "RevolutionGeneratedSolid", revolutionStyle);

    context.expect(extrusionId != MyBRep::Display::InvalidBRepDisplayId, "Viewer adds Extrusion Solid");
    context.expect(revolutionId != MyBRep::Display::InvalidBRepDisplayId, "Viewer adds Revolution Solid");
    context.expect(extrusionId != revolutionId, "Viewer allocates distinct display IDs");
    context.expect(viewer.displayCount() == 2, "Viewer tracks two generated-surface Solids");
    context.expect(viewer.containsDisplay(extrusionId), "Viewer contains Extrusion Solid display");
    context.expect(viewer.containsDisplay(revolutionId), "Viewer contains Revolution Solid display");

    const MyBRep::Display::BRepDisplayObject extrusionDisplay = viewer.display(extrusionId);
    const MyBRep::Display::BRepDisplayObject revolutionDisplay = viewer.display(revolutionId);

    context.expect(extrusionDisplay.isValid(), "Extrusion viewer display object valid");
    context.expect(extrusionDisplay.hasSurface() && extrusionDisplay.hasWireframe(), "Extrusion viewer display owns surface and wireframe");
    context.expect(revolutionDisplay.isValid(), "Revolution viewer display object valid");
    context.expect(revolutionDisplay.hasSurface() && revolutionDisplay.hasWireframe(), "Revolution viewer display owns surface and wireframe");

    context.expect(viewer.removeDisplay(extrusionId), "Viewer removes Extrusion Solid display");
    context.expect(viewer.displayCount() == 1 && !viewer.containsDisplay(extrusionId), "Extrusion display resources removed from viewer");
    context.expect(viewer.clearBRepDisplays(), "Viewer clears remaining generated-surface display");
    context.expect(viewer.displayCount() == 0, "Viewer generated-surface display set empty after clear");
}

}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);
    TestContext context;
    MyBRep::Display::BRepViewerWidget viewer;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep Generated Surface Solid Viewer Integration Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testViewer(context, viewer);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;
    return context.failed() == 0 ? 0 : 1;
}
