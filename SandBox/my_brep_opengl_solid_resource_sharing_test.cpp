#include <QApplication>
#include <QVector4D>

#include <iostream>
#include <vector>

#include "MyMath/CoordinateSystem.h"
#include "MyMath/Matrix3.h"
#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyBRep/Instance/Solid.h"
#include "MyBRep/Modeling/Edge/EdgeModeling.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Modeling/Shell/ShellModeling.h"
#include "MyBRep/Modeling/Solid/SolidModeling.h"
#include "MyBRep/Modeling/Wire/WireModeling.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"
#include "MyBRepOpenGL/Display/BRepDisplayObject.h"
#include "MyBRepOpenGL/Display/BRepDisplayStyle.h"
#include "MyBRepOpenGL/Viewer/BRepViewerWidget.h"

namespace SolidResourceSharingTest
{

class TestContext
{
public:
    TestContext()
        : m_passed(0)
        , m_failed(0)
    {
    }

    void expect(bool condition,const char* name)
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

    int passed() const{return m_passed;}
    int failed() const{return m_failed;}

private:
    int m_passed;
    int m_failed;
};


}

int main(int argc,char* argv[])
{
    QApplication application(argc, argv);
    SolidResourceSharingTest::TestContext context;
    MyBRep::Display::BRepViewerWidget viewer;

    MyBRep::Display::BRepSolidBuildOptions globalOptions = viewer.buildOptions();
    globalOptions.surface.meshing.chordTolerance = 0.05;
    context.expect(viewer.setBuildOptions(globalOptions), "Viewer accepts one global meshing option set before displays are created");


    const double sizeX = 4.0;
    const double sizeY = 5.0;
    const double sizeZ = 3.0;

    const MyBRep::Topology_Vertex v000(MyMath::Vector3(0.0, 0.0, 0.0));
    const MyBRep::Topology_Vertex v100(MyMath::Vector3(sizeX, 0.0, 0.0));
    const MyBRep::Topology_Vertex v110(MyMath::Vector3(sizeX, sizeY, 0.0));
    const MyBRep::Topology_Vertex v010(MyMath::Vector3(0.0, sizeY, 0.0));
    const MyBRep::Topology_Vertex v001(MyMath::Vector3(0.0, 0.0, sizeZ));
    const MyBRep::Topology_Vertex v101(MyMath::Vector3(sizeX, 0.0, sizeZ));
    const MyBRep::Topology_Vertex v111(MyMath::Vector3(sizeX, sizeY, sizeZ));
    const MyBRep::Topology_Vertex v011(MyMath::Vector3(0.0, sizeY, sizeZ));

    const MyBRep::Topology_Edge e0 = MyBRep::Modeling::createLine(v000, v100);
    const MyBRep::Topology_Edge e1 = MyBRep::Modeling::createLine(v100, v110);
    const MyBRep::Topology_Edge e2 = MyBRep::Modeling::createLine(v110, v010);
    const MyBRep::Topology_Edge e3 = MyBRep::Modeling::createLine(v010, v000);
    const MyBRep::Topology_Edge e4 = MyBRep::Modeling::createLine(v001, v101);
    const MyBRep::Topology_Edge e5 = MyBRep::Modeling::createLine(v101, v111);
    const MyBRep::Topology_Edge e6 = MyBRep::Modeling::createLine(v111, v011);
    const MyBRep::Topology_Edge e7 = MyBRep::Modeling::createLine(v011, v001);
    const MyBRep::Topology_Edge e8 = MyBRep::Modeling::createLine(v000, v001);
    const MyBRep::Topology_Edge e9 = MyBRep::Modeling::createLine(v100, v101);
    const MyBRep::Topology_Edge e10 = MyBRep::Modeling::createLine(v110, v111);
    const MyBRep::Topology_Edge e11 = MyBRep::Modeling::createLine(v010, v011);

    std::vector<MyBRep::Topology_Edge> bottomEdges;
    bottomEdges.push_back(e3.reversed());
    bottomEdges.push_back(e2.reversed());
    bottomEdges.push_back(e1.reversed());
    bottomEdges.push_back(e0.reversed());

    std::vector<MyBRep::Topology_Edge> topEdges;
    topEdges.push_back(e4);
    topEdges.push_back(e5);
    topEdges.push_back(e6);
    topEdges.push_back(e7);

    std::vector<MyBRep::Topology_Edge> frontEdges;
    frontEdges.push_back(e0);
    frontEdges.push_back(e9);
    frontEdges.push_back(e4.reversed());
    frontEdges.push_back(e8.reversed());

    std::vector<MyBRep::Topology_Edge> backEdges;
    backEdges.push_back(e11);
    backEdges.push_back(e6.reversed());
    backEdges.push_back(e10.reversed());
    backEdges.push_back(e2);

    std::vector<MyBRep::Topology_Edge> leftEdges;
    leftEdges.push_back(e8);
    leftEdges.push_back(e7.reversed());
    leftEdges.push_back(e11.reversed());
    leftEdges.push_back(e3);

    std::vector<MyBRep::Topology_Edge> rightEdges;
    rightEdges.push_back(e1);
    rightEdges.push_back(e10);
    rightEdges.push_back(e5.reversed());
    rightEdges.push_back(e9.reversed());

    const MyBRep::Topology_Wire bottomWire = MyBRep::Modeling::createWire(bottomEdges);
    const MyBRep::Topology_Wire topWire = MyBRep::Modeling::createWire(topEdges);
    const MyBRep::Topology_Wire frontWire = MyBRep::Modeling::createWire(frontEdges);
    const MyBRep::Topology_Wire backWire = MyBRep::Modeling::createWire(backEdges);
    const MyBRep::Topology_Wire leftWire = MyBRep::Modeling::createWire(leftEdges);
    const MyBRep::Topology_Wire rightWire = MyBRep::Modeling::createWire(rightEdges);

    const MyMath::CoordinateSystem bottomCS = MyMath::CoordinateSystem::fromAxes(
        MyMath::Vector3::zero(), MyMath::Vector3::unitX(), -MyMath::Vector3::unitY(), -MyMath::Vector3::unitZ());
    const MyMath::CoordinateSystem topCS = MyMath::CoordinateSystem::fromAxes(
        MyMath::Vector3(0.0, 0.0, sizeZ), MyMath::Vector3::unitX(), MyMath::Vector3::unitY(), MyMath::Vector3::unitZ());
    const MyMath::CoordinateSystem frontCS = MyMath::CoordinateSystem::fromAxes(
        MyMath::Vector3::zero(), MyMath::Vector3::unitX(), MyMath::Vector3::unitZ(), -MyMath::Vector3::unitY());
    const MyMath::CoordinateSystem backCS = MyMath::CoordinateSystem::fromAxes(
        MyMath::Vector3(0.0, sizeY, 0.0), MyMath::Vector3::unitX(), -MyMath::Vector3::unitZ(), MyMath::Vector3::unitY());
    const MyMath::CoordinateSystem leftCS = MyMath::CoordinateSystem::fromAxes(
        MyMath::Vector3::zero(), MyMath::Vector3::unitY(), -MyMath::Vector3::unitZ(), -MyMath::Vector3::unitX());
    const MyMath::CoordinateSystem rightCS = MyMath::CoordinateSystem::fromAxes(
        MyMath::Vector3(sizeX, 0.0, 0.0), MyMath::Vector3::unitY(), MyMath::Vector3::unitZ(), MyMath::Vector3::unitX());

    std::vector<MyBRep::Topology_Face> faces;
    faces.reserve(6);
    faces.push_back(MyBRep::Modeling::createPlanarFace(bottomCS, bottomWire));
    faces.push_back(MyBRep::Modeling::createPlanarFace(topCS, topWire));
    faces.push_back(MyBRep::Modeling::createPlanarFace(frontCS, frontWire));
    faces.push_back(MyBRep::Modeling::createPlanarFace(backCS, backWire));
    faces.push_back(MyBRep::Modeling::createPlanarFace(leftCS, leftWire));
    faces.push_back(MyBRep::Modeling::createPlanarFace(rightCS, rightWire));

    const MyBRep::Topology_Shell boxShell = MyBRep::Modeling::createShell(faces);
    const MyBRep::Topology_Solid boxSolid = MyBRep::Modeling::createSolid(boxShell);

    MyBRep::Display::BRepDisplayStyle firstStyle;
    MyBRep::Display::BRepDisplayStyle secondStyle;
    firstStyle.surfaceColor = QVector4D(0.8f, 0.2f, 0.2f, 1.0f);
    secondStyle.surfaceColor = QVector4D(0.2f, 0.4f, 0.9f, 1.0f);

    const MyBRep::Solid first(boxSolid);
    const MyBRep::Solid second(boxSolid, MyMath::Matrix4::fromTranslation(MyMath::Vector3(8.0, -3.0, 2.0)));

    const MyBRep::Display::BRepDisplayId firstId = viewer.addSolid(first, "First", firstStyle);
    const MyBRep::Display::BRepDisplayId secondId = viewer.addSolid(second, "Second", secondStyle);

    const MyBRep::Display::BRepDisplayObject firstDisplay = viewer.display(firstId);
    const MyBRep::Display::BRepDisplayObject secondDisplay = viewer.display(secondId);

    context.expect(firstId != MyBRep::Display::InvalidBRepDisplayId && secondId != MyBRep::Display::InvalidBRepDisplayId,
                   "Two Solid instances added");
    context.expect(firstDisplay.itemId != secondDisplay.itemId, "Each Solid instance owns a different RenderItem");
    context.expect(firstDisplay.solidGeometryResourceId != MyBRep::Display::InvalidBRepSolidGeometryResourceId,
                   "First Solid uses shared Geometry resource");
    context.expect(firstDisplay.solidGeometryResourceId == secondDisplay.solidGeometryResourceId,
                   "Same Topology and meshing options share one logical Geometry resource");
    context.expect(firstDisplay.surfaceGeometryId == secondDisplay.surfaceGeometryId,
                   "Surface BufferGeometry is shared");
    context.expect(firstDisplay.wireframeGeometryId == secondDisplay.wireframeGeometryId,
                   "Boundary BufferGeometry is shared");
    context.expect(firstDisplay.surfaceMaterialId != secondDisplay.surfaceMaterialId,
                   "Surface Material remains instance-owned");
    context.expect(firstDisplay.wireframeMaterialId != secondDisplay.wireframeMaterialId,
                   "Wireframe Material remains instance-owned");

    MyBRep::Display::BRepSolidBuildOptions changedOptions = viewer.buildOptions();
    changedOptions.surface.meshing.chordTolerance *= 0.5;
    context.expect(!viewer.setBuildOptions(changedOptions),
                   "Viewer rejects changing global meshing options while displays exist");

    const MyBRep::Display::BRepDisplayId reversedId = viewer.addSolid(boxSolid.reversed(), "Reversed", firstStyle);
    const MyBRep::Display::BRepDisplayObject reversedDisplay = viewer.display(reversedId);

    context.expect(reversedDisplay.solidGeometryResourceId != firstDisplay.solidGeometryResourceId,
                   "Reversed Topology_Solid does not reuse Forward Geometry resource");

    const MyBRep::Topology_Solid independentSolid = MyBRep::Modeling::createSolid(boxShell);
    const MyBRep::Display::BRepDisplayId independentId = viewer.addSolid(independentSolid, "Independent", firstStyle);
    const MyBRep::Display::BRepDisplayObject independentDisplay = viewer.display(independentId);

    context.expect(independentDisplay.solidGeometryResourceId != firstDisplay.solidGeometryResourceId,
                   "Geometrically equal but different Topology identity does not share resource");

    const MyMath::Matrix3 shearLinear(1.0, 0.25, 0.0,
                                      0.0, 1.0, 0.0,
                                      0.0, 0.0, 1.0);
    const MyMath::Matrix4 shearTransform =
        MyMath::Matrix4::fromAffine(shearLinear, MyMath::Vector3(2.0, 0.0, 0.0));
    const MyBRep::Display::BRepDisplayId shearId =
        viewer.addSolid(boxSolid, shearTransform, "Shear", firstStyle);
    const MyBRep::Display::BRepDisplayObject shearDisplay = viewer.display(shearId);

    context.expect(shearId != MyBRep::Display::InvalidBRepDisplayId, "General affine Shear display remains supported");
    context.expect(shearDisplay.solidGeometryResourceId == MyBRep::Display::InvalidBRepSolidGeometryResourceId,
                   "Shear falls back to baked non-shared Geometry");

    context.expect(viewer.removeDisplay(firstId), "Removing one shared instance succeeds");
    context.expect(viewer.containsDisplay(secondId) && viewer.display(secondId).isValid(),
                   "Removing one instance keeps the other shared instance valid");

    context.expect(viewer.clearBRepDisplays(), "Clearing all BRep displays and shared resources succeeds");
    context.expect(viewer.displayCount() == 0, "Viewer contains no displays after clear");

    context.expect(viewer.setBuildOptions(changedOptions),
                   "Viewer accepts changing the one global meshing option set after displays are cleared");

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}
