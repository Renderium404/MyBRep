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
#include "MyBRep/Geometry/Surface/Geometry_CylindricalSurface.h"
#include "MyBRep/Mesh/FaceMesher.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Modeling/Shell/ShellModeling.h"
#include "MyBRep/Modeling/Solid/SolidModeling.h"
#include "MyBRep/Topology/Topology_Builder.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

#include "MyBRepOpenGL/Builder/BRepSolidBuilder.h"
#include "MyBRepOpenGL/Display/BRepDisplayObject.h"
#include "MyBRepOpenGL/Viewer/BRepViewerWidget.h"
#include "MyOpenGL/Item/RenderItem.h"
#include "MyOpenGL/Item/RenderPart.h"
#include "MyOpenGL/Resource/Geometry.h"

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

void testMixedSolidViewer(TestContext& context)
{
    MyBRep::Display::BRepViewerWidget viewer;
    const CylinderSolidFixture fixture = createCylinderSolid(5.0, 8.0);

    MyBRep::Display::BRepDisplayStyle style;
    style.surface.meshing.chordTolerance = 0.03;
    style.surface.cylindricalMeshing.boundaryChordTolerance = 0.03;
    style.surface.cylindricalMeshing.surfaceChordTolerance = 0.03;
    style.wireframe.chordTolerance = 0.03;

    MyBRep::Display::BRepSolidBuildOptions buildOptions;
    buildOptions.surface = style.surface;
    buildOptions.wireframe = style.wireframe;

    BufferGeometry* expectedSurface = MyBRep::Display::BRepSolidBuilder::buildSurface(fixture.solid, "ExpectedSurface", buildOptions);
    BufferGeometry* expectedBoundary = MyBRep::Display::BRepSolidBuilder::buildBoundary(fixture.solid, "ExpectedBoundary", buildOptions);

    const int expectedSurfaceIndices = expectedSurface != 0 ? expectedSurface->indexCount() : 0;
    const int expectedBoundaryIndices = expectedBoundary != 0 ? expectedBoundary->indexCount() : 0;

    delete expectedSurface;
    delete expectedBoundary;

    const std::size_t baseResourceCount = viewer.resourceManager().count();
    const std::size_t baseMaterialCount = viewer.materialManager().count();
    const std::size_t baseItemCount = viewer.itemManager().count();

    const MyBRep::Display::BRepDisplayId id = viewer.addSolid(fixture.solid, "MixedCylinderSolid", style);
    const MyBRep::Display::BRepDisplayObject object = viewer.display(id);

    context.expect(id != MyBRep::Display::InvalidBRepDisplayId, "Mixed cylinder Solid Viewer display registered");
    context.expect(object.isValid() && object.hasSurface() && object.hasWireframe(), "Mixed cylinder Viewer display object owns Surface and Boundary");
    context.expect(viewer.resourceManager().count() == baseResourceCount + 2, "Mixed cylinder Viewer registers two Geometry resources");
    context.expect(viewer.materialManager().count() == baseMaterialCount + 2, "Mixed cylinder Viewer registers two Materials");
    context.expect(viewer.itemManager().count() == baseItemCount + 1, "Mixed cylinder Viewer registers one RenderItem");

    RenderItem* item = viewer.itemManager().get(object.itemId);
    const RenderPart* boundaryPart = item != 0 ? item->partAt(0) : 0;
    const RenderPart* surfacePart = item != 0 ? item->partAt(1) : 0;

    context.expect(item != 0 && item->partCount() == 2, "Mixed cylinder Viewer uses two RenderParts");
    context.expect(boundaryPart != 0 && boundaryPart->geometry() != 0 && boundaryPart->geometry()->renderType() == RenderType::Lines, "Mixed cylinder Boundary Part is first");
    context.expect(surfacePart != 0 && surfacePart->geometry() != 0 && surfacePart->geometry()->renderType() == RenderType::Triangles, "Mixed cylinder Surface Part is second");
    context.expect(boundaryPart != 0 && boundaryPart->geometry() != 0 && boundaryPart->geometry()->indexCount() == expectedBoundaryIndices, "Viewer preserves mixed cylinder unique boundary");
    context.expect(surfacePart != 0 && surfacePart->geometry() != 0 && surfacePart->geometry()->indexCount() == expectedSurfaceIndices, "Viewer preserves mixed cylinder surface mesh");

    context.expect(viewer.removeDisplay(id), "Mixed cylinder Viewer display removal succeeds");
    const bool resourcesReleased = viewer.resourceManager().count() == baseResourceCount && viewer.materialManager().count() == baseMaterialCount && viewer.itemManager().count() == baseItemCount;
    context.expect(resourcesReleased, "Mixed cylinder Viewer resources return to baseline");
}

void testAffineSolidInstance(TestContext& context)
{
    MyBRep::Display::BRepViewerWidget viewer;
    const CylinderSolidFixture fixture = createCylinderSolid(5.0, 8.0);

    MyMath::Matrix4 transform = MyMath::Matrix4::identity();
    transform(0, 1) = 0.25; // XY剪切验证Solid实例的一般仿射显示。
    transform(2, 0) = 0.15; // Z随X变化验证圆柱侧面和端盖共同变换。
    transform(0, 3) = 10.0; // X方向平移10个模型单位。

    const MyBRep::Solid solidInstance(fixture.solid, transform);

    MyBRep::Display::BRepDisplayStyle style;
    style.surface.cylindricalMeshing.surfaceChordTolerance = 0.03;

    const MyBRep::Display::BRepDisplayId id = viewer.addSolid(solidInstance, "AffineMixedCylinder", style);

    context.expect(id != MyBRep::Display::InvalidBRepDisplayId, "Affine mixed cylinder Solid Instance display registered");
    context.expect(viewer.display(id).isValid(), "Affine mixed cylinder display object valid");
    context.expect(viewer.removeDisplay(id), "Affine mixed cylinder display removal succeeds");
}

}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep MyOpenGL Mixed Cylinder Viewer Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testMixedSolidViewer(context);
    testAffineSolidInstance(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}
