#include <QApplication>
#include <QTimer>
#include <QVector4D>

#include <vector>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Modeling/Wire/WireModeling.h"
#include "MyBRepOpenGL/Display/BRepDisplayStyle.h"
#include "MyBRepOpenGL/Viewer/BRepViewerWidget.h"

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);

    MyBRep::Display::BRepViewerWidget viewer;
    viewer.resize(1200, 800);
    viewer.setWindowTitle("MyBRep OpenGL Viewer Test");

    // 第一组：左侧矩形。
    const MyBRep::Topology_Wire rectangle =MyBRep::Modeling::createRectangle(20.0, 12.0);

    const MyMath::Matrix4 rectangleTransform =MyMath::Matrix4::fromTranslation(MyMath::Vector3(-28.0, 0.0, 0.0));

    MyBRep::Display::BRepDisplayStyle rectangleStyle;
    rectangleStyle.wireColor = QVector4D(0.15f, 0.35f, 0.85f, 1.0f);
    rectangleStyle.wireframe.lineWidth = 2.0f;

    viewer.addWireframe(rectangle,rectangleTransform,"Rectangle",rectangleStyle);

    // 第二组：中间圆。
    const MyBRep::Topology_Wire circle =MyBRep::Modeling::createCircle(8.0);

    MyBRep::Display::BRepDisplayStyle circleStyle;
    circleStyle.wireColor = QVector4D(0.85f, 0.25f, 0.20f, 1.0f);
    circleStyle.wireframe.lineWidth = 2.0f;
    circleStyle.wireframe.chordTolerance = 0.02;

    viewer.addWireframe(circle,"Circle",circleStyle);

    // 第三组：右侧带内孔平面Face。
    std::vector<MyBRep::Topology_Wire> faceWires;
    faceWires.push_back(MyBRep::Modeling::createRectangle(22.0, 16.0));
    faceWires.push_back(MyBRep::Modeling::createCircle(4.0));

    const MyBRep::Topology_Face face =MyBRep::Modeling::createPlanarFace(faceWires, 1.0e-8);

    const MyMath::Matrix4 faceTransform =MyMath::Matrix4::fromTranslation(MyMath::Vector3(30.0, 0.0, 0.0));

    MyBRep::Display::BRepDisplayStyle faceStyle;
    faceStyle.wireColor = QVector4D(0.10f, 0.60f, 0.25f, 1.0f);
    faceStyle.wireframe.lineWidth = 2.0f;
    faceStyle.wireframe.chordTolerance = 0.02;

    viewer.addWireframe(
        face,
        faceTransform,
        "FaceWithHole",
        faceStyle);

    viewer.show();

    // Viewer完成首次尺寸建立后再执行Fit，保证相机使用有效Viewport。
    QTimer::singleShot(0, [&viewer]()
    {
        viewer.fitItemsToView();
    });

    return application.exec();
}