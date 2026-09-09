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
#include "MyBRep/Geometry/Curve2D/Geometry_Curve2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Line2D.h"
#include "MyBRep/Geometry/Surface/Geometry_SphericalSurface.h"
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

const double Pi = 3.1415926535897932384626433832795; // 完整球形可视化测试统一使用的圆周率。
const double HalfPi = Pi * 0.5;                       // 球面南北极对应的V参数绝对值。
const double TwoPi = Pi * 2.0;                        // 球面U参数完整周期。
const double TestTolerance = 1.0e-8;                 // Topology和Curve-on-Surface连接统一使用的三维容差。
const int ViewerWidth = 1200;                        // 完整球形Solid可视化测试窗口宽度。
const int ViewerHeight = 760;                        // 完整球形Solid可视化测试窗口高度。
const float BoundaryLineWidth = 2.0f;                // seam边使用较明显线宽，便于观察周期拓扑边界。

MyBRep::Topology_Solid createSphereSolid(double radius)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface(new MyBRep::Geometry_SphericalSurface(MyMath::CoordinateSystem::identity(), radius));
    const MyBRep::Geometry_SphericalSurface& sphere = static_cast<const MyBRep::Geometry_SphericalSurface&>(*surface);

    const MyBRep::Topology_Vertex southVertex(sphere.pointAt(0.0, -HalfPi));
    const MyBRep::Topology_Vertex northVertex(sphere.pointAt(0.0, HalfPi));

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> seamGeometry(new MyBRep::Geometry_Circle(sphere.center(), sphere.radius(), sphere.xDir(), sphere.zDir()));
    MyBRep::Topology_Edge seamEdge(southVertex, northVertex, seamGeometry, -HalfPi, HalfPi, TestTolerance);

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> firstSeamCurve(new MyBRep::Geometry_Line2D(MyMath::Vector2(0.0, -HalfPi), MyMath::Vector2(0.0, 1.0)));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> secondSeamCurve(new MyBRep::Geometry_Line2D(MyMath::Vector2(TwoPi, -HalfPi), MyMath::Vector2(0.0, 1.0)));
    MyBRep::Topology_Builder::addCurveOnClosedSurface(seamEdge, surface, firstSeamCurve, 0.0, Pi, secondSeamCurve, 0.0, Pi, TestTolerance);

    std::vector<MyBRep::Topology_Edge> edges;
    edges.push_back(seamEdge);
    edges.push_back(seamEdge.reversed());

    const MyBRep::Topology_Face face = MyBRep::Modeling::createFace(surface, std::vector<MyBRep::Topology_Wire>(1, MyBRep::Topology_Wire(edges)));
    return MyBRep::Modeling::createSolid(MyBRep::Modeling::createShell(std::vector<MyBRep::Topology_Face>(1, face)));
}

}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);

    MyBRep::Display::BRepViewerWidget viewer;
    viewer.resize(ViewerWidth, ViewerHeight);
    viewer.setWindowTitle("MyBRep Full Sphere Solid Viewer Test");

    const MyBRep::Topology_Solid sphere = createSphereSolid(9.0);

    MyBRep::Display::BRepDisplayStyle leftStyle;
    leftStyle.surfaceColor = QVector4D(0.70f, 0.82f, 0.94f, 1.0f);
    leftStyle.wireColor = QVector4D(0.05f, 0.09f, 0.16f, 1.0f);
    leftStyle.wireframe.lineWidth = BoundaryLineWidth;
    leftStyle.wireframe.chordTolerance = 0.03;
    leftStyle.surface.sphericalMeshing.boundaryChordTolerance = 0.03;
    leftStyle.surface.sphericalMeshing.surfaceChordTolerance = 0.03;
    leftStyle.surface.sphericalMeshing.minimumBoundarySubdivisionDepth = 1;

    const MyMath::Matrix4 leftTransform = MyMath::Matrix4::fromTranslation(MyMath::Vector3(-22.0, 0.0, 0.0));
    viewer.addSolid(sphere, leftTransform, "FullSphere", leftStyle);

    MyMath::Matrix4 affineTransform = MyMath::Matrix4::identity();
    affineTransform(0, 1) = 0.32; // XY剪切将右侧球体变为明显的一般仿射椭球状实体。
    affineTransform(2, 0) = 0.18; // Z随X变化，进一步验证非TRS法向与顶点变换链。
    affineTransform(0, 3) = 22.0; // X方向平移22个模型单位，将第二个实体放到窗口右侧。

    MyBRep::Display::BRepDisplayStyle rightStyle = leftStyle;
    rightStyle.surfaceColor = QVector4D(0.90f, 0.75f, 0.66f, 1.0f);
    rightStyle.wireColor = QVector4D(0.22f, 0.06f, 0.04f, 1.0f);

    viewer.addSolid(sphere, affineTransform, "AffineFullSphere", rightStyle);

    viewer.show();

    // Viewer完成首次Viewport建立后再Fit，保证相机使用有效窗口尺寸包含两个完整球形实体。
    QTimer::singleShot(0, [&viewer]()
    {
        viewer.fitItemsToView();
    });

    return application.exec();
}
