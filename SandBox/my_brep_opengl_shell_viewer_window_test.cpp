#include <QApplication>
#include <QTimer>
#include <QVector4D>

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

#include "MyBRepOpenGL/Display/BRepDisplayStyle.h"
#include "MyBRepOpenGL/Viewer/BRepViewerWidget.h"

namespace
{

const int ViewerWidth = 1200;        // Shell可视化测试窗口宽度。
const int ViewerHeight = 760;        // Shell可视化测试窗口高度。
const double FaceTolerance = 1.0e-8; // Edge连接和Planar Face构造统一使用的几何容差。
const float BoundaryLineWidth = 2.0f;// Shell边界显示线宽，便于观察共享Edge和深度遮挡。

MyBRep::Topology_Edge createLineEdge(const MyBRep::Topology_Vertex& startVertex, const MyBRep::Topology_Vertex& endVertex)
{
    const MyMath::Vector3 direction = endVertex.point() - startVertex.point();
    const double length = direction.length();

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> geometry(
        new MyBRep::Geometry_Line(startVertex.point(), direction));

    return MyBRep::Topology_Edge(startVertex, endVertex, geometry, 0.0, length, FaceTolerance);
}

std::vector<MyBRep::Topology_Face> createFoldedFaces()
{
    const MyBRep::Topology_Vertex leftBottom(MyMath::Vector3(-14.0, -6.0, 0.0));
    const MyBRep::Topology_Vertex sharedBottom(MyMath::Vector3(0.0, -6.0, 0.0));
    const MyBRep::Topology_Vertex sharedTop(MyMath::Vector3(0.0, 6.0, 0.0));
    const MyBRep::Topology_Vertex leftTop(MyMath::Vector3(-14.0, 6.0, 0.0));

    const MyBRep::Topology_Vertex outerBottom(MyMath::Vector3(0.0, -6.0, 12.0));
    const MyBRep::Topology_Vertex outerTop(MyMath::Vector3(0.0, 6.0, 12.0));

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

    const MyBRep::Topology_Face leftFace = MyBRep::Modeling::createPlanarFace(leftWire, FaceTolerance);

    const MyMath::Vector3 uDirection = MyMath::Vector3::unitZ();
    const MyMath::Vector3 vDirection = MyMath::Vector3::unitY();
    const MyMath::Vector3 normal = MyMath::Vector3::cross(uDirection, vDirection);

    const MyMath::CoordinateSystem foldCoordinateSystem =
        MyMath::CoordinateSystem::fromAxes(MyMath::Vector3::zero(), uDirection, vDirection, normal);

    const MyBRep::Topology_Face foldFace =
        MyBRep::Modeling::createPlanarFace(foldCoordinateSystem, foldWire, FaceTolerance);

    std::vector<MyBRep::Topology_Face> faces;
    faces.push_back(leftFace);
    faces.push_back(foldFace);
    return faces;
}

}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);

    MyBRep::Display::BRepViewerWidget viewer;
    viewer.resize(ViewerWidth, ViewerHeight);
    viewer.setWindowTitle("MyBRep OpenGL Shell Viewer Test");

    const std::vector<MyBRep::Topology_Face> faces = createFoldedFaces();
    const MyBRep::Topology_Shell shell = MyBRep::Modeling::createShell(faces);

    MyBRep::Display::BRepDisplayStyle leftStyle;
    leftStyle.surfaceColor = QVector4D(0.72f, 0.82f, 0.92f, 1.0f);
    leftStyle.wireColor = QVector4D(0.06f, 0.10f, 0.16f, 1.0f);
    leftStyle.wireframe.lineWidth = BoundaryLineWidth;

    const MyMath::Matrix4 leftTransform =MyMath::Matrix4::fromTranslation(MyMath::Vector3(-18.0, 0.0, 0.0));

    viewer.addShell(shell, leftTransform, "FoldedShell", leftStyle);

    MyMath::Matrix4 affineTransform = MyMath::Matrix4::identity();
    affineTransform(0, 1) = 0.35; // XY剪切使第二个Shell与左侧标准Shell具有明显不同的仿射形状。
    affineTransform(2, 0) = 0.20; // Z随X变化，验证两个不同Plane Face经过统一一般仿射后仍保持Surface/Boundary一致。
    affineTransform(0, 3) = 22.0; // X方向平移22个模型单位，将第二个Shell放到窗口右侧。

    MyBRep::Display::BRepDisplayStyle rightStyle;
    rightStyle.surfaceColor = QVector4D(0.88f, 0.76f, 0.66f, 1.0f);
    rightStyle.wireColor = QVector4D(0.20f, 0.07f, 0.04f, 1.0f);
    rightStyle.wireframe.lineWidth = BoundaryLineWidth;

    viewer.addShell(shell, affineTransform, "AffineShell", rightStyle);

    viewer.show();

    // Viewer完成首次Viewport建立后再执行Fit，保证相机适配使用有效窗口尺寸。
    QTimer::singleShot(0, [&viewer]()
    {
        viewer.fitItemsToView();
    });

    return application.exec();
}