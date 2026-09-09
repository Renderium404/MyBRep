#include <QApplication>
#include <QTimer>
#include <QVector4D>

#include <vector>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"

#include "MyBRep/Modeling/Edge/EdgeModeling.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Modeling/Wire/WireModeling.h"

#include "MyBRepOpenGL/Display/BRepDisplayStyle.h"
#include "MyBRepOpenGL/Viewer/BRepViewerWidget.h"

namespace
{

const int ViewerWidth = 1280;                       // 测试窗口宽度，保证两行测试对象能够同时显示。
const int ViewerHeight = 820;                       // 测试窗口高度，保证默认Fit后仍保留足够观察区域。
const double Pi = 3.14159265358979323846;           // 圆弧测试使用的圆周率常量。
const double FaceTolerance = 1.0e-8;                // 平面Face构造容差，与现有二维Face测试保持同一数量级。
const double CurveChordTolerance = 0.02;            // 曲线显示弦误差，用于兼顾圆弧平滑度和测试规模。
const float TestLineWidth = 2.0f;                   // 测试线宽，便于在高分辨率窗口中观察边界。

}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);

    MyBRep::Display::BRepViewerWidget viewer;
    viewer.resize(ViewerWidth, ViewerHeight);
    viewer.setWindowTitle("MyBRep OpenGL Viewer Test");

    /// 第一行：基础Topology对象。

    // 左上：三维直线Edge，用于验证单Edge入口和非XY平面点。
    const MyBRep::Topology_Edge lineEdge = MyBRep::Modeling::createLine(
        MyMath::Vector3(-8.0, 0.0, -3.0),
        MyMath::Vector3(8.0, 0.0, 3.0));

    const MyMath::Matrix4 lineTransform = MyMath::Matrix4::fromTranslation(MyMath::Vector3(-42.0, 18.0, 0.0));

    MyBRep::Display::BRepDisplayStyle lineStyle;
    lineStyle.wireColor = QVector4D(0.15f, 0.35f, 0.85f, 1.0f);
    lineStyle.wireframe.lineWidth = TestLineWidth;

    viewer.addWireframe(lineEdge, lineTransform, "LineEdge", lineStyle);

    // 中上：270度圆弧Edge，用于验证曲线Edge自适应离散。
    const MyBRep::Topology_Edge arcEdge = MyBRep::Modeling::createArc(
        MyMath::Vector3(0.0, 0.0, 0.0),
        8.0,
        0.0,
        Pi * 1.5);

    const MyMath::Matrix4 arcTransform = MyMath::Matrix4::fromTranslation(MyMath::Vector3(-14.0, 18.0, 0.0));

    MyBRep::Display::BRepDisplayStyle arcStyle;
    arcStyle.wireColor = QVector4D(0.85f, 0.25f, 0.20f, 1.0f);
    arcStyle.wireframe.lineWidth = TestLineWidth;
    arcStyle.wireframe.chordTolerance = CurveChordTolerance;

    viewer.addWireframe(arcEdge, arcTransform, "ArcEdge", arcStyle);

    // 右上：开放折线Wire，用于验证多Edge开放Wire。
    std::vector<MyMath::Vector3> polylinePoints;
    polylinePoints.push_back(MyMath::Vector3(-9.0, -5.0, 0.0));
    polylinePoints.push_back(MyMath::Vector3(-4.0, 5.0, 0.0));
    polylinePoints.push_back(MyMath::Vector3(1.0, -2.0, 0.0));
    polylinePoints.push_back(MyMath::Vector3(8.0, 4.0, 0.0));

    const MyBRep::Topology_Wire polyline = MyBRep::Modeling::createPolyline(polylinePoints);
    const MyMath::Matrix4 polylineTransform = MyMath::Matrix4::fromTranslation(MyMath::Vector3(16.0, 18.0, 0.0));

    MyBRep::Display::BRepDisplayStyle polylineStyle;
    polylineStyle.wireColor = QVector4D(0.65f, 0.25f, 0.75f, 1.0f);
    polylineStyle.wireframe.lineWidth = TestLineWidth;

    viewer.addWireframe(polyline, polylineTransform, "Polyline", polylineStyle);

    // 最右上：带圆形内环的Planar Face，用于验证Face全部裁剪Wire边界。
    std::vector<MyBRep::Topology_Wire> faceWires;
    faceWires.push_back(MyBRep::Modeling::createRectangle(22.0, 16.0));
    faceWires.push_back(MyBRep::Modeling::createCircle(4.0));

    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(faceWires, FaceTolerance);
    const MyMath::Matrix4 faceTransform = MyMath::Matrix4::fromTranslation(MyMath::Vector3(44.0, 18.0, 0.0));

    MyBRep::Display::BRepDisplayStyle faceStyle;
    faceStyle.wireColor = QVector4D(0.10f, 0.60f, 0.25f, 1.0f);
    faceStyle.wireframe.lineWidth = TestLineWidth;
    faceStyle.wireframe.chordTolerance = CurveChordTolerance;

    viewer.addWireframe(face, faceTransform, "FaceWithHole", faceStyle);

    /// 第二行：Instance和一般仿射变换。

    // 左下：Wire实例，用于验证Instance入口直接使用其localToWorld放置。
    const MyMath::Matrix4 rectangleInstanceTransform = MyMath::Matrix4::fromTranslation(MyMath::Vector3(-28.0, -18.0, 0.0));
    const MyBRep::Wire rectangleInstance = MyBRep::Modeling::makeRectangle(20.0, 12.0, rectangleInstanceTransform);

    MyBRep::Display::BRepDisplayStyle rectangleInstanceStyle;
    rectangleInstanceStyle.wireColor = QVector4D(0.20f, 0.55f, 0.85f, 1.0f);
    rectangleInstanceStyle.wireframe.lineWidth = TestLineWidth;

    viewer.addWireframe(rectangleInstance, "RectangleInstance", rectangleInstanceStyle);

    // 中下：圆Wire实例，用于验证闭合单Edge Wire的Instance入口。
    const MyMath::Matrix4 circleInstanceTransform = MyMath::Matrix4::fromTranslation(MyMath::Vector3(0.0, -18.0, 0.0));
    const MyBRep::Wire circleInstance = MyBRep::Modeling::makeCircle(8.0, circleInstanceTransform);

    MyBRep::Display::BRepDisplayStyle circleInstanceStyle;
    circleInstanceStyle.wireColor = QVector4D(0.90f, 0.55f, 0.10f, 1.0f);
    circleInstanceStyle.wireframe.lineWidth = TestLineWidth;
    circleInstanceStyle.wireframe.chordTolerance = CurveChordTolerance;

    viewer.addWireframe(circleInstance, "CircleInstance", circleInstanceStyle);

    // 右下：一般仿射剪切Wire，用于确认Matrix4不是被降级为MyOpenGL的TRS Transform。
    MyMath::Matrix4 shearTransform = MyMath::Matrix4::identity();
    shearTransform(0, 1) = 0.6;                     // XY剪切系数0.6，用于产生肉眼可辨认的非TRS仿射效果。
    shearTransform(0, 3) = 30.0;                    // X方向平移30个模型单位，将剪切对象放置在右下区域。
    shearTransform(1, 3) = -18.0;                   // Y方向平移-18个模型单位，将剪切对象放置在第二行。

    const MyBRep::Wire shearedRectangle = MyBRep::Modeling::makeRectangle(20.0, 12.0, shearTransform);

    MyBRep::Display::BRepDisplayStyle shearStyle;
    shearStyle.wireColor = QVector4D(0.10f, 0.65f, 0.65f, 1.0f);
    shearStyle.wireframe.lineWidth = TestLineWidth;

    viewer.addWireframe(shearedRectangle, "ShearedRectangle", shearStyle);

    viewer.show();

    // Viewer完成首次尺寸建立后再Fit，避免零尺寸Viewport影响相机适配。
    QTimer::singleShot(0, [&viewer]()
    {
        viewer.fitItemsToView();
    });

    return application.exec();
}