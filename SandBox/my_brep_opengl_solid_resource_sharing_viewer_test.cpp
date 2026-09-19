#include <QApplication>
#include <QLabel>
#include <QVBoxLayout>
#include <QVector4D>
#include <QWidget>

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
#include "MyBRepOpenGL/Display/BRepDisplayStyle.h"
#include "MyBRepOpenGL/Viewer/BRepViewerWidget.h"

int main(int argc,char* argv[])
{
    QApplication application(argc, argv);


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

    const MyBRep::Solid first(boxSolid, MyMath::Matrix4::fromTranslation(MyMath::Vector3(-7.0, 0.0, 0.0)));
    const MyBRep::Solid second(boxSolid, MyMath::Matrix4::identity());
    const MyBRep::Solid third(boxSolid, MyMath::Matrix4::fromTranslation(MyMath::Vector3(7.0, 0.0, 0.0)));

    MyBRep::Display::BRepDisplayStyle firstStyle;
    MyBRep::Display::BRepDisplayStyle secondStyle;
    MyBRep::Display::BRepDisplayStyle thirdStyle;
    firstStyle.surfaceColor = QVector4D(0.85f, 0.25f, 0.22f, 1.0f);
    secondStyle.surfaceColor = QVector4D(0.24f, 0.62f, 0.86f, 1.0f);
    thirdStyle.surfaceColor = QVector4D(0.28f, 0.72f, 0.38f, 1.0f);

    QWidget window;
    window.setWindowTitle("MyBRep Shared Solid Geometry Resource");
    window.resize(1280, 760);

    QVBoxLayout* layout = new QVBoxLayout(&window);
    QLabel* label = new QLabel("Three RenderItems / One shared Topology_Solid Geometry resource", &window);
    label->setAlignment(Qt::AlignCenter);

    MyBRep::Display::BRepViewerWidget* viewer = new MyBRep::Display::BRepViewerWidget(&window);

    const MyBRep::Display::BRepDisplayId firstId = viewer->addSolid(first, "FirstInstance", firstStyle);
    const MyBRep::Display::BRepDisplayId secondId = viewer->addSolid(second, "SecondInstance", secondStyle);
    const MyBRep::Display::BRepDisplayId thirdId = viewer->addSolid(third, "ThirdInstance", thirdStyle);

    if (firstId == MyBRep::Display::InvalidBRepDisplayId ||
        secondId == MyBRep::Display::InvalidBRepDisplayId ||
        thirdId == MyBRep::Display::InvalidBRepDisplayId)
    {
        return 1;
    }

    const MyBRep::Display::BRepDisplayObject firstDisplay = viewer->display(firstId);
    const MyBRep::Display::BRepDisplayObject secondDisplay = viewer->display(secondId);
    const MyBRep::Display::BRepDisplayObject thirdDisplay = viewer->display(thirdId);

    if (firstDisplay.itemId == secondDisplay.itemId ||
        firstDisplay.itemId == thirdDisplay.itemId ||
        firstDisplay.solidGeometryResourceId != secondDisplay.solidGeometryResourceId ||
        firstDisplay.solidGeometryResourceId != thirdDisplay.solidGeometryResourceId)
    {
        return 1;
    }

    layout->addWidget(label);
    layout->addWidget(viewer, 1);

    window.show();
    application.processEvents();
    viewer->fitItemsToView();
    return application.exec();
}
