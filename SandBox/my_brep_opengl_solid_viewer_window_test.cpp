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
#include "MyBRep/Modeling/Solid/SolidModeling.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

#include "MyBRepOpenGL/Display/BRepDisplayStyle.h"
#include "MyBRepOpenGL/Viewer/BRepViewerWidget.h"

namespace
{

const int ViewerWidth = 1200;        // Solid可视化测试窗口宽度。
const int ViewerHeight = 760;        // Solid可视化测试窗口高度。
const double TestTolerance = 1.0e-8; // 六面体共享Edge连接和Planar Face构造统一使用的几何容差。
const float BoundaryLineWidth = 2.0f;// Solid边界显示线宽，便于观察十二条共享B-Rep棱。


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


}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);

    MyBRep::Display::BRepViewerWidget viewer;
    viewer.resize(ViewerWidth, ViewerHeight);
    viewer.setWindowTitle("MyBRep OpenGL Solid Viewer Test");

    const MyBRep::Topology_Solid solid =
        createBoxSolid(MyMath::Vector3::zero(), 20.0, 14.0, 12.0);

    MyBRep::Display::BRepDisplayStyle leftStyle;
    leftStyle.surfaceColor = QVector4D(0.72f, 0.82f, 0.92f, 1.0f);
    leftStyle.wireColor = QVector4D(0.06f, 0.10f, 0.16f, 1.0f);
    leftStyle.wireframe.lineWidth = BoundaryLineWidth;

    const MyMath::Matrix4 leftTransform =
        MyMath::Matrix4::fromTranslation(MyMath::Vector3(-22.0, 0.0, 0.0));

    viewer.addSolid(solid, leftTransform, "BoxSolid", leftStyle);

    MyMath::Matrix4 affineTransform = MyMath::Matrix4::identity();
    affineTransform(0, 1) = 0.30; // XY剪切使右侧六面体形成明显斜盒形状。
    affineTransform(2, 0) = 0.18; // Z随X变化，验证六个Face在一般仿射下仍保持Surface/Boundary一致。
    affineTransform(0, 3) = 24.0; // X方向平移24个模型单位，将第二个Solid放到窗口右侧。

    MyBRep::Display::BRepDisplayStyle rightStyle;
    rightStyle.surfaceColor = QVector4D(0.88f, 0.76f, 0.66f, 1.0f);
    rightStyle.wireColor = QVector4D(0.20f, 0.07f, 0.04f, 1.0f);
    rightStyle.wireframe.lineWidth = BoundaryLineWidth;

    viewer.addSolid(solid, affineTransform, "AffineSolid", rightStyle);

    viewer.show();

    // Viewer完成首次Viewport建立后再执行Fit，保证相机适配使用有效窗口尺寸。
    QTimer::singleShot(0, [&viewer]()
    {
        viewer.fitItemsToView();
    });

    return application.exec();
}
