#include <QApplication>
#include <QTimer>
#include <QVector4D>

#include <vector>

#include "MyMath/CoordinateSystem.h"
#include "MyMath/Matrix4.h"
#include "MyMath/Vector2.h"
#include "MyMath/Vector3.h"

#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Curve/Geometry_Circle.h"
#include "MyBRep/Geometry/Curve/Geometry_Curve.h"
#include "MyBRep/Geometry/Curve/Geometry_Line.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Curve2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Line2D.h"
#include "MyBRep/Geometry/Surface/Geometry_ConicalSurface.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Modeling/Shell/ShellModeling.h"
#include "MyBRep/Modeling/Solid/SolidModeling.h"
#include "MyBRep/Topology/Topology_Builder.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

#include "MyBRepOpenGL/Display/BRepDisplayStyle.h"
#include "MyBRepOpenGL/Viewer/BRepViewerWidget.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 完整圆锥可视化测试统一使用的圆周率。
const double TwoPi = Pi * 2.0;                        // 圆锥侧面的完整U参数周期。
const double TestTolerance = 1.0e-8;                 // Topology和Curve-on-Surface连接统一使用的三维容差。
const int ViewerWidth = 1200;                        // 完整圆锥Solid可视化测试窗口宽度。
const int ViewerHeight = 760;                        // 完整圆锥Solid可视化测试窗口高度。
const float BoundaryLineWidth = 2.0f;                // base与seam边使用较明显线宽，便于观察共享边去重。

MyBRep::Topology_Solid createConeSolid(double semiAngle, double height)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> sideSurface(new MyBRep::Geometry_ConicalSurface(MyMath::CoordinateSystem::identity(), semiAngle));
    const MyBRep::Geometry_ConicalSurface& cone = static_cast<const MyBRep::Geometry_ConicalSurface&>(*sideSurface);

    const MyBRep::Topology_Vertex apexVertex(cone.pointAt(0.0, 0.0));
    const MyBRep::Topology_Vertex baseVertex(cone.pointAt(0.0, height));
    const double baseRadius = height * cone.radialSlope(); // V=height截面的圆锥底圆半径。
    const MyMath::Vector3 baseCenter = cone.apex() + cone.axisDir() * height;

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> baseCircle(new MyBRep::Geometry_Circle(baseCenter, baseRadius, cone.xDir(), cone.yDir()));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> seamLine(new MyBRep::Geometry_Line(apexVertex.point(), baseVertex.point() - apexVertex.point()));

    MyBRep::Topology_Edge baseEdge(baseVertex, baseVertex, baseCircle, 0.0, TwoPi, TestTolerance);
    MyBRep::Topology_Edge seamEdge(apexVertex, baseVertex, seamLine, 0.0, (baseVertex.point() - apexVertex.point()).length(), TestTolerance);

    const MyMath::Vector2 baseDirection(TwoPi, 0.0);
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> baseCurve(new MyBRep::Geometry_Line2D(MyMath::Vector2(0.0, height), baseDirection));
    MyBRep::Topology_Builder::addCurveOnSurface(baseEdge, sideSurface, baseCurve, 0.0, baseDirection.length(), TestTolerance);

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> firstSeamCurve(new MyBRep::Geometry_Line2D(MyMath::Vector2(0.0, 0.0), MyMath::Vector2(0.0, 1.0)));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> secondSeamCurve(new MyBRep::Geometry_Line2D(MyMath::Vector2(TwoPi, 0.0), MyMath::Vector2(0.0, 1.0)));
    MyBRep::Topology_Builder::addCurveOnClosedSurface(seamEdge, sideSurface, firstSeamCurve, 0.0, height, secondSeamCurve, 0.0, height, TestTolerance);

    std::vector<MyBRep::Topology_Edge> sideEdges;
    sideEdges.push_back(seamEdge);
    sideEdges.push_back(baseEdge);
    sideEdges.push_back(seamEdge.reversed());

    const MyBRep::Topology_Face sideFace = MyBRep::Modeling::createFace(sideSurface, std::vector<MyBRep::Topology_Wire>(1, MyBRep::Topology_Wire(sideEdges)));

    const MyMath::CoordinateSystem baseSystem = MyMath::CoordinateSystem::fromAxes(baseCenter, cone.xDir(), cone.yDir(), cone.axisDir());
    const MyBRep::Topology_Face baseFace = MyBRep::Modeling::createPlanarFace(baseSystem, MyBRep::Topology_Wire(std::vector<MyBRep::Topology_Edge>(1, baseEdge.reversed())), TestTolerance);

    std::vector<MyBRep::Topology_Face> faces;
    faces.push_back(baseFace);
    faces.push_back(sideFace);
    return MyBRep::Modeling::createSolid(MyBRep::Modeling::createShell(faces));
}

}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);

    MyBRep::Display::BRepViewerWidget viewer;
    viewer.resize(ViewerWidth, ViewerHeight);
    viewer.setWindowTitle("MyBRep Mixed Plane Conical Solid Viewer Test");

    const MyBRep::Topology_Solid cone = createConeSolid(Pi / 6.0, 16.0);

    MyBRep::Display::BRepDisplayStyle leftStyle;
    leftStyle.surfaceColor = QVector4D(0.72f, 0.84f, 0.92f, 1.0f);
    leftStyle.wireColor = QVector4D(0.05f, 0.09f, 0.16f, 1.0f);
    leftStyle.wireframe.lineWidth = BoundaryLineWidth;
    leftStyle.wireframe.chordTolerance = 0.03;
    leftStyle.surface.meshing.chordTolerance = 0.02;
    leftStyle.surface.conicalMeshing.boundaryChordTolerance = 0.03;
    leftStyle.surface.conicalMeshing.surfaceChordTolerance = 0.03;
    leftStyle.surface.conicalMeshing.minimumBoundarySubdivisionDepth = 1;

    const MyMath::Matrix4 leftTransform = MyMath::Matrix4::fromTranslation(MyMath::Vector3(-16.0, 0.0, -8.0));
    viewer.addSolid(cone, leftTransform, "MixedCone", leftStyle);

    MyMath::Matrix4 affineTransform = MyMath::Matrix4::identity();
    affineTransform(0, 1) = 0.30; // XY剪切将右侧圆锥变为明显的一般仿射形变实体。
    affineTransform(2, 0) = 0.18; // Z随X变化，进一步验证非TRS法向与顶点变换链。
    affineTransform(0, 3) = 16.0; // X方向平移16个模型单位，将第二个实体放到窗口右侧。
    affineTransform(2, 3) = -8.0; // Z方向下移8个模型单位，使两个实体视觉中心接近一致。

    MyBRep::Display::BRepDisplayStyle rightStyle = leftStyle;
    rightStyle.surfaceColor = QVector4D(0.91f, 0.74f, 0.64f, 1.0f);
    rightStyle.wireColor = QVector4D(0.22f, 0.06f, 0.04f, 1.0f);

    viewer.addSolid(cone, affineTransform, "AffineMixedCone", rightStyle);

    viewer.show();

    // Viewer完成首次Viewport建立后再Fit，保证相机使用有效窗口尺寸包含两个完整圆锥实体。
    QTimer::singleShot(0, [&viewer]()
    {
        viewer.fitItemsToView();
    });

    return application.exec();
}
