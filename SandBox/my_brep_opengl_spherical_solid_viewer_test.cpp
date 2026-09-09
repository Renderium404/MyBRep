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
#include "MyBRep/Geometry/Curve2D/Geometry_Curve2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Line2D.h"
#include "MyBRep/Geometry/Surface/Geometry_SphericalSurface.h"
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

const double Pi = 3.1415926535897932384626433832795; // 完整球形Viewer测试统一使用的圆周率。
const double HalfPi = Pi * 0.5;                       // 球面南北极对应的V参数绝对值。
const double TwoPi = Pi * 2.0;                        // 球面U参数完整周期。
const double TestTolerance = 1.0e-8;                 // Topology和Curve-on-Surface连接统一使用的三维容差。

struct SphereSolidFixture
{
    SphereSolidFixture()
    {
    }

    MyBRep::Topology_Solid solid;
    MyBRep::Topology_Edge seamEdge;
};

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

SphereSolidFixture createSphereSolid(double radius)
{
    SphereSolidFixture fixture;

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface(new MyBRep::Geometry_SphericalSurface(MyMath::CoordinateSystem::identity(), radius));
    const MyBRep::Geometry_SphericalSurface& sphere = static_cast<const MyBRep::Geometry_SphericalSurface&>(*surface);

    const MyBRep::Topology_Vertex southVertex(sphere.pointAt(0.0, -HalfPi));
    const MyBRep::Topology_Vertex northVertex(sphere.pointAt(0.0, HalfPi));

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> seamGeometry(new MyBRep::Geometry_Circle(sphere.center(), sphere.radius(), sphere.xDir(), sphere.zDir()));
    fixture.seamEdge = MyBRep::Topology_Edge(southVertex, northVertex, seamGeometry, -HalfPi, HalfPi, TestTolerance);

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> firstSeamCurve(new MyBRep::Geometry_Line2D(MyMath::Vector2(0.0, -HalfPi), MyMath::Vector2(0.0, 1.0)));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> secondSeamCurve(new MyBRep::Geometry_Line2D(MyMath::Vector2(TwoPi, -HalfPi), MyMath::Vector2(0.0, 1.0)));
    MyBRep::Topology_Builder::addCurveOnClosedSurface(fixture.seamEdge, surface, firstSeamCurve, 0.0, Pi, secondSeamCurve, 0.0, Pi, TestTolerance);

    std::vector<MyBRep::Topology_Edge> edges;
    edges.push_back(fixture.seamEdge);
    edges.push_back(fixture.seamEdge.reversed());

    const MyBRep::Topology_Face face = MyBRep::Modeling::createFace(surface, std::vector<MyBRep::Topology_Wire>(1, MyBRep::Topology_Wire(edges)));
    fixture.solid = MyBRep::Modeling::createSolid(MyBRep::Modeling::createShell(std::vector<MyBRep::Topology_Face>(1, face)));
    return fixture;
}

MyBRep::Display::BRepDisplayStyle displayStyle()
{
    MyBRep::Display::BRepDisplayStyle style;
    style.surface.sphericalMeshing.boundaryChordTolerance = 0.03;
    style.surface.sphericalMeshing.surfaceChordTolerance = 0.02;
    style.surface.sphericalMeshing.minimumBoundarySubdivisionDepth = 1;
    style.wireframe.chordTolerance = 0.03;
    return style;
}

void testSphereSolidViewer(TestContext& context)
{
    MyBRep::Display::BRepViewerWidget viewer;
    const SphereSolidFixture fixture = createSphereSolid(5.0);
    const MyBRep::Display::BRepDisplayStyle style = displayStyle();

    MyBRep::Display::BRepSolidBuildOptions buildOptions;
    buildOptions.surface = style.surface;
    buildOptions.wireframe = style.wireframe;

    BufferGeometry* expectedSurface = MyBRep::Display::BRepSolidBuilder::buildSurface(fixture.solid, "ExpectedSphereSurface", buildOptions);
    BufferGeometry* expectedBoundary = MyBRep::Display::BRepSolidBuilder::buildBoundary(fixture.solid, "ExpectedSphereBoundary", buildOptions);

    const int expectedSurfaceIndices = expectedSurface != 0 ? expectedSurface->indexCount() : 0;
    const int expectedBoundaryIndices = expectedBoundary != 0 ? expectedBoundary->indexCount() : 0;

    delete expectedSurface;
    delete expectedBoundary;

    const std::size_t baseResourceCount = viewer.resourceManager().count();
    const std::size_t baseMaterialCount = viewer.materialManager().count();
    const std::size_t baseItemCount = viewer.itemManager().count();

    const MyBRep::Display::BRepDisplayId id = viewer.addSolid(fixture.solid, "FullSphereSolid", style);
    const MyBRep::Display::BRepDisplayObject object = viewer.display(id);

    context.expect(id != MyBRep::Display::InvalidBRepDisplayId, "Full sphere Solid Viewer display registered");
    context.expect(object.isValid() && object.hasSurface() && object.hasWireframe(), "Full sphere Viewer display object owns Surface and Boundary");
    context.expect(viewer.resourceManager().count() == baseResourceCount + 2, "Full sphere Viewer registers two Geometry resources");
    context.expect(viewer.materialManager().count() == baseMaterialCount + 2, "Full sphere Viewer registers two Materials");
    context.expect(viewer.itemManager().count() == baseItemCount + 1, "Full sphere Viewer registers one RenderItem");

    RenderItem* item = viewer.itemManager().get(object.itemId);
    const RenderPart* boundaryPart = item != 0 ? item->partAt(0) : 0;
    const RenderPart* surfacePart = item != 0 ? item->partAt(1) : 0;

    context.expect(item != 0 && item->partCount() == 2, "Full sphere Viewer uses two RenderParts");
    context.expect(boundaryPart != 0 && boundaryPart->geometry() != 0 && boundaryPart->geometry()->renderType() == RenderType::Lines, "Full sphere seam Boundary Part is first");
    context.expect(surfacePart != 0 && surfacePart->geometry() != 0 && surfacePart->geometry()->renderType() == RenderType::Triangles, "Full sphere Surface Part is second");
    context.expect(boundaryPart != 0 && boundaryPart->geometry() != 0 && boundaryPart->geometry()->indexCount() == expectedBoundaryIndices, "Viewer preserves single deduplicated sphere seam");
    context.expect(surfacePart != 0 && surfacePart->geometry() != 0 && surfacePart->geometry()->indexCount() == expectedSurfaceIndices, "Viewer preserves complete spherical surface mesh");

    context.expect(viewer.removeDisplay(id), "Full sphere Viewer display removal succeeds");
    const bool resourcesReleased = viewer.resourceManager().count() == baseResourceCount && viewer.materialManager().count() == baseMaterialCount && viewer.itemManager().count() == baseItemCount;
    context.expect(resourcesReleased, "Full sphere Viewer resources return to baseline");
}

void testAffineSphereSolidInstance(TestContext& context)
{
    MyBRep::Display::BRepViewerWidget viewer;
    const SphereSolidFixture fixture = createSphereSolid(5.0);

    MyMath::Matrix4 transform = MyMath::Matrix4::identity();
    transform(0, 1) = 0.28; // XY剪切验证完整球形Solid Instance的一般仿射显示。
    transform(2, 0) = 0.16; // Z随X变化，使球面产生明显非TRS形变。
    transform(0, 3) = 12.0; // X方向平移12个模型单位。

    const MyBRep::Solid solidInstance(fixture.solid, transform);
    const MyBRep::Display::BRepDisplayId id = viewer.addSolid(solidInstance, "AffineFullSphere", displayStyle());

    context.expect(id != MyBRep::Display::InvalidBRepDisplayId, "Affine full sphere Solid Instance display registered");
    context.expect(viewer.display(id).isValid(), "Affine full sphere display object valid");
    context.expect(viewer.removeDisplay(id), "Affine full sphere display removal succeeds");
}

}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep MyOpenGL Full Sphere Solid Viewer Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testSphereSolidViewer(context);
    testAffineSphereSolidInstance(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}
