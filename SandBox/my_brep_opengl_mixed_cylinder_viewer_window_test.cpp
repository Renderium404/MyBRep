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
#include "MyBRep/Geometry/Surface/Geometry_CylindricalSurface.h"
#include "MyBRep/Mesh/FaceMesher.h"
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

const double Pi = 3.1415926535897932384626433832795; // 圆柱实体测试统一使用的圆周率。
const double TwoPi = Pi * 2.0;                        // 完整圆柱侧面的U参数周期。
const double TestTolerance = 1.0e-8;                 // Topology和Curve-on-Surface连接统一使用的三维容差。

struct CylinderSolidFixture
{
    CylinderSolidFixture()
    {
    }

    MyBRep::Topology_Solid solid;
    MyBRep::Topology_Edge bottomEdge;
    MyBRep::Topology_Edge topEdge;
    MyBRep::Topology_Edge seamEdge;
};

void addLineCurveOnSurface(MyBRep::Topology_Edge& edge, const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>& surface, const MyMath::Vector2& firstUV, const MyMath::Vector2& lastUV)
{
    const MyMath::Vector2 direction = lastUV - firstUV;
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> curve(new MyBRep::Geometry_Line2D(firstUV, direction));
    MyBRep::Topology_Builder::addCurveOnSurface(edge, surface, curve, 0.0, direction.length(), TestTolerance);
}

CylinderSolidFixture createCylinderSolid(double radius, double height)
{
    const double bottomV = -height * 0.5; // 圆柱轴向下端参数。
    const double topV = height * 0.5;     // 圆柱轴向上端参数。

    CylinderSolidFixture fixture;

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> sideSurface(new MyBRep::Geometry_CylindricalSurface(MyMath::CoordinateSystem::identity(), radius));
    const MyBRep::Geometry_CylindricalSurface& cylinder = static_cast<const MyBRep::Geometry_CylindricalSurface&>(*sideSurface);

    const MyBRep::Topology_Vertex bottomVertex(cylinder.pointAt(0.0, bottomV));
    const MyBRep::Topology_Vertex topVertex(cylinder.pointAt(0.0, topV));

    const MyMath::Vector3 bottomCenter = cylinder.axisOrigin() + cylinder.axisDir() * bottomV;
    const MyMath::Vector3 topCenter = cylinder.axisOrigin() + cylinder.axisDir() * topV;

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> bottomCircle(new MyBRep::Geometry_Circle(bottomCenter, radius, cylinder.xDir(), cylinder.yDir()));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> topCircle(new MyBRep::Geometry_Circle(topCenter, radius, cylinder.xDir(), cylinder.yDir()));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> seamLine(new MyBRep::Geometry_Line(bottomVertex.point(), topVertex.point() - bottomVertex.point()));

    fixture.bottomEdge = MyBRep::Topology_Edge(bottomVertex, bottomVertex, bottomCircle, 0.0, TwoPi, TestTolerance);
    fixture.topEdge = MyBRep::Topology_Edge(topVertex, topVertex, topCircle, 0.0, TwoPi, TestTolerance);
    fixture.seamEdge = MyBRep::Topology_Edge(bottomVertex, topVertex, seamLine, 0.0, height, TestTolerance);

    addLineCurveOnSurface(fixture.bottomEdge, sideSurface, MyMath::Vector2(0.0, bottomV), MyMath::Vector2(TwoPi, bottomV));
    addLineCurveOnSurface(fixture.topEdge, sideSurface, MyMath::Vector2(0.0, topV), MyMath::Vector2(TwoPi, topV));

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> firstSeamCurve(new MyBRep::Geometry_Line2D(MyMath::Vector2(0.0, bottomV), MyMath::Vector2(0.0, 1.0)));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> secondSeamCurve(new MyBRep::Geometry_Line2D(MyMath::Vector2(TwoPi, bottomV), MyMath::Vector2(0.0, 1.0)));

    MyBRep::Topology_Builder::addCurveOnClosedSurface(fixture.seamEdge, sideSurface, firstSeamCurve, 0.0, height, secondSeamCurve, 0.0, height, TestTolerance);

    std::vector<MyBRep::Topology_Edge> sideEdges;
    sideEdges.push_back(fixture.bottomEdge.reversed());
    sideEdges.push_back(fixture.seamEdge);
    sideEdges.push_back(fixture.topEdge);
    sideEdges.push_back(fixture.seamEdge.reversed());

    std::vector<MyBRep::Topology_Wire> sideWires;
    sideWires.push_back(MyBRep::Topology_Wire(sideEdges));
    const MyBRep::Topology_Face sideFace = MyBRep::Modeling::createFace(sideSurface, sideWires);

    const MyMath::CoordinateSystem bottomSystem = MyMath::CoordinateSystem::fromAxes(bottomCenter, MyMath::Vector3::unitX(), -MyMath::Vector3::unitY(), -MyMath::Vector3::unitZ());
    const MyMath::CoordinateSystem topSystem = MyMath::CoordinateSystem::fromAxes(topCenter, MyMath::Vector3::unitX(), MyMath::Vector3::unitY(), MyMath::Vector3::unitZ());

    std::vector<MyBRep::Topology_Edge> bottomEdges;
    bottomEdges.push_back(fixture.bottomEdge);
    const MyBRep::Topology_Face bottomFace = MyBRep::Modeling::createPlanarFace(bottomSystem, MyBRep::Topology_Wire(bottomEdges), TestTolerance);

    std::vector<MyBRep::Topology_Edge> topEdges;
    topEdges.push_back(fixture.topEdge.reversed());
    const MyBRep::Topology_Face topFace = MyBRep::Modeling::createPlanarFace(topSystem, MyBRep::Topology_Wire(topEdges), TestTolerance);

    std::vector<MyBRep::Topology_Face> faces;
    faces.push_back(bottomFace);
    faces.push_back(topFace);
    faces.push_back(sideFace);

    fixture.solid = MyBRep::Modeling::createSolid(MyBRep::Modeling::createShell(faces));
    return fixture;
}

std::vector<MyBRep::Topology_Edge> uniqueEdges(const MyBRep::Topology_Shell& shell)
{
    std::vector<MyBRep::Topology_Edge> result;

    for (std::size_t faceIndex = 0; faceIndex < shell.faceCount(); ++faceIndex)
    {
        const MyBRep::Topology_Face face = shell.face(faceIndex);

        for (std::size_t wireIndex = 0; wireIndex < face.wireCount(); ++wireIndex)
        {
            const MyBRep::Topology_Wire wire = face.wire(wireIndex);

            for (std::size_t edgeIndex = 0; edgeIndex < wire.edgeCount(); ++edgeIndex)
            {
                const MyBRep::Topology_Edge edge = wire.edge(edgeIndex);
                bool exists = false;

                for (std::size_t current = 0; current < result.size(); ++current)
                {
                    if (result[current].isSame(edge))
                    {
                        exists = true;
                        break;
                    }
                }

                if (!exists)
                {
                    result.push_back(edge);
                }
            }
        }
    }

    return result;
}


const int ViewerWidth = 1200;         // 混合曲面圆柱Solid可视化测试窗口宽度。
const int ViewerHeight = 760;         // 混合曲面圆柱Solid可视化测试窗口高度。
const float BoundaryLineWidth = 2.0f; // 圆边和seam边使用较明显线宽，便于观察拓扑边界。

}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);

    MyBRep::Display::BRepViewerWidget viewer;
    viewer.resize(ViewerWidth, ViewerHeight);
    viewer.setWindowTitle("MyBRep Mixed-Surface Cylinder Solid Viewer Test");

    const CylinderSolidFixture fixture = createCylinderSolid(8.0, 18.0);

    MyBRep::Display::BRepDisplayStyle leftStyle;
    leftStyle.surfaceColor = QVector4D(0.72f, 0.82f, 0.92f, 1.0f);
    leftStyle.wireColor = QVector4D(0.06f, 0.10f, 0.16f, 1.0f);
    leftStyle.wireframe.lineWidth = BoundaryLineWidth;
    leftStyle.wireframe.chordTolerance = 0.03;
    leftStyle.surface.meshing.chordTolerance = 0.03;
    leftStyle.surface.cylindricalMeshing.boundaryChordTolerance = 0.03;
    leftStyle.surface.cylindricalMeshing.surfaceChordTolerance = 0.03;

    const MyMath::Matrix4 leftTransform = MyMath::Matrix4::fromTranslation(MyMath::Vector3(-20.0, 0.0, 0.0));
    viewer.addSolid(fixture.solid, leftTransform, "MixedCylinder", leftStyle);

    MyMath::Matrix4 affineTransform = MyMath::Matrix4::identity();
    affineTransform(0, 1) = 0.30; // XY剪切使右侧圆柱实体形成明显的一般仿射形变。
    affineTransform(2, 0) = 0.18; // Z随X变化，使圆形端盖和圆柱侧面共同发生非TRS仿射变化。
    affineTransform(0, 3) = 22.0; // X方向平移22个模型单位，将第二个实体放到窗口右侧。

    MyBRep::Display::BRepDisplayStyle rightStyle = leftStyle;
    rightStyle.surfaceColor = QVector4D(0.88f, 0.76f, 0.66f, 1.0f);
    rightStyle.wireColor = QVector4D(0.20f, 0.07f, 0.04f, 1.0f);

    viewer.addSolid(fixture.solid, affineTransform, "AffineMixedCylinder", rightStyle);

    viewer.show();

    // Viewer完成首次Viewport建立后再Fit，保证相机使用有效窗口尺寸包含两个实体。
    QTimer::singleShot(0, [&viewer]()
    {
        viewer.fitItemsToView();
    });

    return application.exec();
}
