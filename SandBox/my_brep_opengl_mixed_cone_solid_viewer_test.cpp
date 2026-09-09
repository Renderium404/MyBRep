#include <QApplication>

#include <iostream>
#include <string>
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
#include "MyBRep/Instance/Solid.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Modeling/Shell/ShellModeling.h"
#include "MyBRep/Modeling/Solid/SolidModeling.h"
#include "MyBRep/Topology/Topology_Builder.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

#include "MyBRepOpenGL/Builder/BRepSolidBuilder.h"
#include "MyBRepOpenGL/Display/BRepDisplayObject.h"
#include "MyBRepOpenGL/Display/BRepDisplayStyle.h"
#include "MyBRepOpenGL/Viewer/BRepViewerWidget.h"
#include "MyOpenGL/Item/RenderItem.h"
#include "MyOpenGL/Item/RenderPart.h"
#include "MyOpenGL/Resource/Geometry.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 完整圆锥Viewer测试统一使用的圆周率。
const double TwoPi = Pi * 2.0;                        // 圆锥侧面的完整U参数周期。
const double TestTolerance = 1.0e-8;                 // Topology和Curve-on-Surface连接统一使用的三维容差。

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

MyBRep::Display::BRepDisplayStyle displayStyle()
{
    MyBRep::Display::BRepDisplayStyle style;
    style.surface.meshing.chordTolerance = 0.02;
    style.surface.conicalMeshing.boundaryChordTolerance = 0.03;
    style.surface.conicalMeshing.surfaceChordTolerance = 0.02;
    style.surface.conicalMeshing.minimumBoundarySubdivisionDepth = 1;
    style.wireframe.chordTolerance = 0.03;
    return style;
}

class TestContext
{
public:
    TestContext() : m_passed(0), m_failed(0)
    {
    }

    void expect(bool condition, const std::string& name)
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

    int passed() const
    {
        return m_passed;
    }

    int failed() const
    {
        return m_failed;
    }

private:
    int m_passed;
    int m_failed;
};

void testConeSolidViewer(TestContext& context)
{
    MyBRep::Display::BRepViewerWidget viewer;
    const MyBRep::Topology_Solid solid = createConeSolid(Pi / 6.0, 8.0);
    const MyBRep::Display::BRepDisplayStyle style = displayStyle();

    MyBRep::Display::BRepSolidBuildOptions buildOptions;
    buildOptions.surface = style.surface;
    buildOptions.wireframe = style.wireframe;

    BufferGeometry* expectedSurface = MyBRep::Display::BRepSolidBuilder::buildSurface(solid, "ExpectedConeSurface", buildOptions);
    BufferGeometry* expectedBoundary = MyBRep::Display::BRepSolidBuilder::buildBoundary(solid, "ExpectedConeBoundary", buildOptions);

    const int expectedSurfaceIndices = expectedSurface != 0 ? expectedSurface->indexCount() : 0;
    const int expectedBoundaryIndices = expectedBoundary != 0 ? expectedBoundary->indexCount() : 0;

    delete expectedSurface;
    delete expectedBoundary;

    const std::size_t baseResourceCount = viewer.resourceManager().count();
    const std::size_t baseMaterialCount = viewer.materialManager().count();
    const std::size_t baseItemCount = viewer.itemManager().count();

    const MyBRep::Display::BRepDisplayId id = viewer.addSolid(solid, "MixedConeSolid", style);
    const MyBRep::Display::BRepDisplayObject object = viewer.display(id);

    context.expect(id != MyBRep::Display::InvalidBRepDisplayId, "Mixed cone Solid Viewer display registered");
    context.expect(object.isValid() && object.hasSurface() && object.hasWireframe(), "Mixed cone Viewer display object owns Surface and Boundary");
    context.expect(viewer.resourceManager().count() == baseResourceCount + 2, "Mixed cone Viewer registers two Geometry resources");
    context.expect(viewer.materialManager().count() == baseMaterialCount + 2, "Mixed cone Viewer registers two Materials");
    context.expect(viewer.itemManager().count() == baseItemCount + 1, "Mixed cone Viewer registers one RenderItem");

    RenderItem* item = viewer.itemManager().get(object.itemId);
    const RenderPart* boundaryPart = item != 0 ? item->partAt(0) : 0;
    const RenderPart* surfacePart = item != 0 ? item->partAt(1) : 0;

    context.expect(item != 0 && item->partCount() == 2, "Mixed cone Viewer uses two RenderParts");
    context.expect(boundaryPart != 0 && boundaryPart->geometry() != 0 && boundaryPart->geometry()->renderType() == RenderType::Lines, "Mixed cone Boundary Part is first");
    context.expect(surfacePart != 0 && surfacePart->geometry() != 0 && surfacePart->geometry()->renderType() == RenderType::Triangles, "Mixed cone Surface Part is second");
    context.expect(boundaryPart != 0 && boundaryPart->geometry() != 0 && boundaryPart->geometry()->indexCount() == expectedBoundaryIndices, "Viewer preserves globally deduplicated cone boundary");
    context.expect(surfacePart != 0 && surfacePart->geometry() != 0 && surfacePart->geometry()->indexCount() == expectedSurfaceIndices, "Viewer preserves complete mixed cone surface mesh");

    context.expect(viewer.removeDisplay(id), "Mixed cone Viewer display removal succeeds");
    const bool resourcesReleased = viewer.resourceManager().count() == baseResourceCount && viewer.materialManager().count() == baseMaterialCount && viewer.itemManager().count() == baseItemCount;
    context.expect(resourcesReleased, "Mixed cone Viewer resources return to baseline");
}

void testAffineConeSolidInstance(TestContext& context)
{
    MyBRep::Display::BRepViewerWidget viewer;
    const MyBRep::Topology_Solid solid = createConeSolid(Pi / 6.0, 8.0);

    MyMath::Matrix4 transform = MyMath::Matrix4::identity();
    transform(0, 1) = 0.25; // XY剪切验证完整圆锥Solid Instance的一般仿射显示。
    transform(2, 0) = 0.15; // Z随X变化，进一步验证非TRS法向与顶点变换链。
    transform(0, 3) = 12.0; // X方向平移12个模型单位。

    const MyBRep::Solid solidInstance(solid, transform);
    const MyBRep::Display::BRepDisplayId id = viewer.addSolid(solidInstance, "AffineMixedCone", displayStyle());

    context.expect(id != MyBRep::Display::InvalidBRepDisplayId, "Affine mixed cone Solid Instance display registered");
    context.expect(viewer.display(id).isValid(), "Affine mixed cone display object valid");
    context.expect(viewer.removeDisplay(id), "Affine mixed cone display removal succeeds");
}

}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep MyOpenGL Mixed Cone Solid Viewer Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testConeSolidViewer(context);
    testAffineConeSolidInstance(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}
