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

namespace
{

const int ViewerWidth = 1200;             // 可视化测试窗口宽度。
const int ViewerHeight = 760;             // 可视化测试窗口高度。
const double FaceTolerance = 1.0e-8;      // 平面Face构造容差。
const double CurveChordTolerance = 0.02;  // 圆形边界和Face网格统一使用的曲线离散弦误差。
const float BoundaryLineWidth = 2.0f;     // 边界显示线宽，便于观察Surface与B-Rep边界叠加效果。

}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);

    MyBRep::Display::BRepViewerWidget viewer;
    viewer.resize(ViewerWidth, ViewerHeight);
    viewer.setWindowTitle("MyBRep OpenGL Face Viewer Test");

    // 左侧：普通矩形Face。
    const MyBRep::Topology_Face rectangleFace =
        MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createRectangle(20.0, 14.0), FaceTolerance);

    MyBRep::Display::BRepDisplayStyle rectangleStyle;
    rectangleStyle.surfaceColor = QVector4D(0.70f, 0.78f, 0.90f, 1.0f);
    rectangleStyle.wireColor = QVector4D(0.08f, 0.12f, 0.18f, 1.0f);
    rectangleStyle.wireframe.lineWidth = BoundaryLineWidth;

    const MyMath::Matrix4 rectangleTransform =
        MyMath::Matrix4::fromTranslation(MyMath::Vector3(-30.0, 0.0, 0.0));

    viewer.addFace(rectangleFace, rectangleTransform, "RectangleFace", rectangleStyle);

    // 中间：外矩形加圆孔Face，重点验证三角表面不会覆盖内孔且边界线完整显示。
    std::vector<MyBRep::Topology_Wire> holeWires;
    holeWires.push_back(MyBRep::Modeling::createRectangle(22.0, 16.0));
    holeWires.push_back(MyBRep::Modeling::createCircle(4.5));

    const MyBRep::Topology_Face holeFace =
        MyBRep::Modeling::createPlanarFace(holeWires, FaceTolerance);

    MyBRep::Display::BRepDisplayStyle holeStyle;
    holeStyle.surfaceColor = QVector4D(0.78f, 0.88f, 0.72f, 1.0f);
    holeStyle.wireColor = QVector4D(0.08f, 0.18f, 0.08f, 1.0f);
    holeStyle.surface.meshing.chordTolerance = CurveChordTolerance;
    holeStyle.wireframe.chordTolerance = CurveChordTolerance;
    holeStyle.wireframe.lineWidth = BoundaryLineWidth;

    viewer.addFace(holeFace, "FaceWithHole", holeStyle);

    // 右侧：一般仿射剪切Face，验证Surface与Boundary使用同一Matrix4烘焙后仍然完全重合。
    const MyBRep::Topology_Face affineFace =
        MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createRectangle(20.0, 14.0), FaceTolerance);

    MyMath::Matrix4 affineTransform = MyMath::Matrix4::identity();
    affineTransform(0, 1) = 0.6;   // XY剪切系数0.6，使矩形显示为明显平行四边形。
    affineTransform(2, 0) = 0.25;  // Z随X变化，使Face离开世界XY平面并可观察一般仿射法向处理。
    affineTransform(0, 3) = 30.0;  // X方向平移30个模型单位，将对象放到窗口右侧。

    MyBRep::Display::BRepDisplayStyle affineStyle;
    affineStyle.surfaceColor = QVector4D(0.90f, 0.76f, 0.68f, 1.0f);
    affineStyle.wireColor = QVector4D(0.22f, 0.08f, 0.05f, 1.0f);
    affineStyle.wireframe.lineWidth = BoundaryLineWidth;

    viewer.addFace(affineFace, affineTransform, "AffineFace", affineStyle);

    viewer.show();

    // Viewer完成首次Viewport建立后再执行Fit，保证相机适配使用有效窗口尺寸。
    QTimer::singleShot(0, [&viewer]()
    {
        viewer.fitItemsToView();
    });

    return application.exec();
}
