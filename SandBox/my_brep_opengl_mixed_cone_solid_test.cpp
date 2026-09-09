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
#include "MyBRep/Mesh/FaceMesher.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Modeling/Shell/ShellModeling.h"
#include "MyBRep/Modeling/Solid/SolidModeling.h"
#include "MyBRep/Topology/Topology_Builder.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

#include "MyBRepOpenGL/Builder/BRepSolidBuilder.h"
#include "MyBRepOpenGL/Builder/BRepWireframeBuilder.h"
#include "MyOpenGL/Resource/BufferGeometry.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 完整圆锥实体测试统一使用的圆周率。
const double TwoPi = Pi * 2.0;                        // 圆锥侧面的完整U参数周期。
const double TestTolerance = 1.0e-8;                 // Topology和Curve-on-Surface连接统一使用的三维容差。
const double PositionTolerance = 1.0e-4;             // GLfloat显示顶点与双精度仿射结果比较允许的位置误差。

struct ConeSolidFixture
{
    ConeSolidFixture()
    {
    }

    MyBRep::Topology_Solid solid;
    MyBRep::Topology_Edge baseEdge;
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

void addLineCurveOnSurface(MyBRep::Topology_Edge& edge, const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>& surface, const MyMath::Vector2& firstUV, const MyMath::Vector2& lastUV)
{
    const MyMath::Vector2 direction = lastUV - firstUV;
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> curve(new MyBRep::Geometry_Line2D(firstUV, direction));
    MyBRep::Topology_Builder::addCurveOnSurface(edge, surface, curve, 0.0, direction.length(), TestTolerance);
}

ConeSolidFixture createConeSolid(double semiAngle, double height)
{
    ConeSolidFixture fixture;

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> sideSurface(new MyBRep::Geometry_ConicalSurface(MyMath::CoordinateSystem::identity(), semiAngle));
    const MyBRep::Geometry_ConicalSurface& cone = static_cast<const MyBRep::Geometry_ConicalSurface&>(*sideSurface);

    const MyBRep::Topology_Vertex apexVertex(cone.pointAt(0.0, 0.0));
    const MyBRep::Topology_Vertex baseVertex(cone.pointAt(0.0, height));
    const double baseRadius = height * cone.radialSlope(); // 圆锥V=height截面的真实底圆半径。
    const MyMath::Vector3 baseCenter = cone.apex() + cone.axisDir() * height;

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> baseCircle(new MyBRep::Geometry_Circle(baseCenter, baseRadius, cone.xDir(), cone.yDir()));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> seamLine(new MyBRep::Geometry_Line(apexVertex.point(), baseVertex.point() - apexVertex.point()));

    fixture.baseEdge = MyBRep::Topology_Edge(baseVertex, baseVertex, baseCircle, 0.0, TwoPi, TestTolerance);
    fixture.seamEdge = MyBRep::Topology_Edge(apexVertex, baseVertex, seamLine, 0.0, (baseVertex.point() - apexVertex.point()).length(), TestTolerance);

    addLineCurveOnSurface(fixture.baseEdge, sideSurface, MyMath::Vector2(0.0, height), MyMath::Vector2(TwoPi, height));

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> firstSeamCurve(new MyBRep::Geometry_Line2D(MyMath::Vector2(0.0, 0.0), MyMath::Vector2(0.0, 1.0)));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> secondSeamCurve(new MyBRep::Geometry_Line2D(MyMath::Vector2(TwoPi, 0.0), MyMath::Vector2(0.0, 1.0)));
    MyBRep::Topology_Builder::addCurveOnClosedSurface(fixture.seamEdge, sideSurface, firstSeamCurve, 0.0, height, secondSeamCurve, 0.0, height, TestTolerance);

    std::vector<MyBRep::Topology_Edge> sideEdges;
    sideEdges.push_back(fixture.seamEdge);
    sideEdges.push_back(fixture.baseEdge);
    sideEdges.push_back(fixture.seamEdge.reversed());

    const MyBRep::Topology_Face sideFace = MyBRep::Modeling::createFace(sideSurface, std::vector<MyBRep::Topology_Wire>(1, MyBRep::Topology_Wire(sideEdges)));

    const MyMath::CoordinateSystem baseSystem = MyMath::CoordinateSystem::fromAxes(baseCenter, cone.xDir(), cone.yDir(), cone.axisDir());
    const MyBRep::Topology_Face baseFace = MyBRep::Modeling::createPlanarFace(baseSystem, MyBRep::Topology_Wire(std::vector<MyBRep::Topology_Edge>(1, fixture.baseEdge.reversed())), TestTolerance);

    std::vector<MyBRep::Topology_Face> faces;
    faces.push_back(baseFace);
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

MyBRep::FaceMeshOptions faceMeshOptions()
{
    MyBRep::FaceMeshOptions options;
    options.planar.chordTolerance = 0.02;
    options.conical.boundaryChordTolerance = 0.03;
    options.conical.surfaceChordTolerance = 0.02;
    options.conical.minimumBoundarySubdivisionDepth = 1;
    return options;
}

MyBRep::Display::BRepSolidBuildOptions solidBuildOptions()
{
    MyBRep::Display::BRepSolidBuildOptions options;
    options.surface.meshing.chordTolerance = 0.02;
    options.surface.conicalMeshing.boundaryChordTolerance = 0.03;
    options.surface.conicalMeshing.surfaceChordTolerance = 0.02;
    options.surface.conicalMeshing.minimumBoundarySubdivisionDepth = 1;
    options.wireframe.chordTolerance = 0.03;
    return options;
}

MyMath::Vector3 geometryPosition(const BufferGeometry& geometry, unsigned int vertexIndex)
{
    const std::size_t offset = static_cast<std::size_t>(vertexIndex) * 6;
    const std::vector<GLfloat>& data = geometry.vertexData();
    return MyMath::Vector3(data[offset], data[offset + 1], data[offset + 2]);
}

bool geometryPositionsFollowTransform(const BufferGeometry& localGeometry, const BufferGeometry& transformedGeometry, const MyMath::Matrix4& transform)
{
    if (localGeometry.vertexCount() != transformedGeometry.vertexCount() || localGeometry.valuesPerVertex() != 6 || transformedGeometry.valuesPerVertex() != 6)
    {
        return false;
    }

    for (unsigned int index = 0; index < localGeometry.vertexCount(); ++index)
    {
        const MyMath::Vector3 expected = transform.transformPoint(geometryPosition(localGeometry, index));

        if (!geometryPosition(transformedGeometry, index).isEqualTo(expected, PositionTolerance))
        {
            return false;
        }
    }

    return true;
}

void testTopology(TestContext& context)
{
    const ConeSolidFixture fixture = createConeSolid(Pi / 6.0, 8.0);
    const MyBRep::Topology_Shell shell = fixture.solid.shell(0);
    const std::vector<MyBRep::Topology_Edge> edges = uniqueEdges(shell);

    context.expect(fixture.solid.isValid(), "Mixed cone Solid valid");
    context.expect(fixture.solid.shellCount() == 1, "Mixed cone Solid has one Shell");
    context.expect(shell.isValid() && shell.isClosed(), "Mixed cone Shell valid and closed");
    context.expect(shell.faceCount() == 2, "Mixed cone Shell has Plane base and Conical side Faces");
    context.expect(edges.size() == 2, "Mixed cone Shell has two unique Topology_Edges");
    context.expect(fixture.seamEdge.isSeamOnSurface(shell.face(1).geometry()), "Cone side seam keeps two P-Curves");
}

void testFaceDispatchInsideSolid(TestContext& context)
{
    const ConeSolidFixture fixture = createConeSolid(Pi / 6.0, 8.0);
    const MyBRep::Topology_Shell shell = fixture.solid.shell(0);
    const MyBRep::FaceMeshOptions options = faceMeshOptions();

    bool allValid = true;
    bool hasConicalSubdivision = false;
    bool hasApexVertex = false;
    std::size_t totalVertices = 0;
    std::size_t totalTriangles = 0;

    for (std::size_t index = 0; index < shell.faceCount(); ++index)
    {
        const MyBRep::Topology_Face face = shell.face(index);
        const MyBRep::FaceMesh mesh = MyBRep::FaceMesher::mesh(face, options);

        allValid = allValid && mesh.isValid();
        totalVertices += mesh.vertexCount();
        totalTriangles += mesh.triangleCount();

        if (face.geometry().kind() == MyBRep::SurfaceKind::Conical)
        {
            hasConicalSubdivision = mesh.triangleCount() > 2;
            const MyBRep::Geometry_ConicalSurface& cone = static_cast<const MyBRep::Geometry_ConicalSurface&>(face.geometry());

            for (std::size_t vertexIndex = 0; vertexIndex < mesh.vertexCount(); ++vertexIndex)
            {
                hasApexVertex = hasApexVertex || mesh.vertices()[vertexIndex].position.isEqualTo(cone.apex(), TestTolerance);
            }
        }
    }

    context.expect(allValid, "Plane and Conical Faces all mesh through FaceMesher");
    context.expect(hasConicalSubdivision, "Cone side receives curvature subdivision");
    context.expect(hasApexVertex, "Cone side FaceMesh contains apex position");
    context.expect(totalVertices > 0 && totalTriangles > 0, "Mixed cone Face meshes produce aggregate surface data");
}

void testSolidBuilder(TestContext& context)
{
    const ConeSolidFixture fixture = createConeSolid(Pi / 6.0, 8.0);
    const MyBRep::Topology_Shell shell = fixture.solid.shell(0);
    const MyBRep::FaceMeshOptions faceOptions = faceMeshOptions();
    const MyBRep::Display::BRepSolidBuildOptions options = solidBuildOptions();

    std::size_t expectedVertices = 0;
    std::size_t expectedTriangles = 0;

    for (std::size_t index = 0; index < shell.faceCount(); ++index)
    {
        const MyBRep::FaceMesh mesh = MyBRep::FaceMesher::mesh(shell.face(index), faceOptions);
        expectedVertices += mesh.vertexCount();
        expectedTriangles += mesh.triangleCount();
    }

    BufferGeometry* surface = MyBRep::Display::BRepSolidBuilder::buildSurface(fixture.solid, "MixedConeSurface", options);
    BufferGeometry* boundary = MyBRep::Display::BRepSolidBuilder::buildBoundary(fixture.solid, "MixedConeBoundary", options);
    BufferGeometry* baseBoundary = MyBRep::Display::BRepWireframeBuilder::build(fixture.baseEdge, "ConeBaseBoundary", options.wireframe);
    BufferGeometry* seamBoundary = MyBRep::Display::BRepWireframeBuilder::build(fixture.seamEdge, "ConeSeamBoundary", options.wireframe);

    context.expect(surface != 0, "BRepSolidBuilder creates mixed Plane Conical surface");
    context.expect(surface != 0 && surface->renderType() == RenderType::Triangles, "Mixed cone surface uses Triangles");
    context.expect(surface != 0 && surface->valuesPerVertex() == 6, "Mixed cone surface uses Position Normal layout");
    context.expect(surface != 0 && surface->vertexCount() == static_cast<int>(expectedVertices), "Solid surface preserves sum of Face-local vertices");
    context.expect(surface != 0 && surface->indexCount() == static_cast<int>(expectedTriangles * 3), "Solid surface preserves sum of Face-local triangles");

    context.expect(boundary != 0, "BRepSolidBuilder creates mixed cone Boundary");
    context.expect(boundary != 0 && boundary->renderType() == RenderType::Lines, "Mixed cone Boundary uses Lines");
    const bool uniqueBoundaryCount = boundary != 0 && baseBoundary != 0 && seamBoundary != 0 && boundary->indexCount() == baseBoundary->indexCount() + seamBoundary->indexCount();
    context.expect(uniqueBoundaryCount, "Mixed cone Boundary globally deduplicates base Edge and repeated seam Edge");

    delete surface;
    delete boundary;
    delete baseBoundary;
    delete seamBoundary;
}

void testAffineSolidBuilder(TestContext& context)
{
    const ConeSolidFixture fixture = createConeSolid(Pi / 6.0, 8.0);
    const MyBRep::Display::BRepSolidBuildOptions options = solidBuildOptions();

    MyMath::Matrix4 transform = MyMath::Matrix4::identity();
    transform(0, 1) = 0.25; // XY剪切验证完整圆锥Solid的一般仿射顶点烘焙。
    transform(2, 0) = 0.15; // Z随X变化，使圆锥产生明显非TRS形变。
    transform(0, 3) = 10.0; // X方向平移10个模型单位。

    BufferGeometry* localSurface = MyBRep::Display::BRepSolidBuilder::buildSurface(fixture.solid, "LocalConeSurface", options);
    BufferGeometry* affineSurface = MyBRep::Display::BRepSolidBuilder::buildSurface(fixture.solid, transform, "AffineConeSurface", options);
    BufferGeometry* affineBoundary = MyBRep::Display::BRepSolidBuilder::buildBoundary(fixture.solid, transform, "AffineConeBoundary", options);

    context.expect(localSurface != 0 && affineSurface != 0, "Local and affine mixed cone Surfaces build");
    context.expect(affineBoundary != 0, "Affine mixed cone Boundary builds");

    const bool positionsCorrect = localSurface != 0 && affineSurface != 0 && geometryPositionsFollowTransform(*localSurface, *affineSurface, transform);
    context.expect(positionsCorrect, "Affine mixed cone Surface positions follow Matrix4");

    delete localSurface;
    delete affineSurface;
    delete affineBoundary;
}

}

int main()
{
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep Mixed Plane Conical Solid Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testTopology(context);
    testFaceDispatchInsideSolid(context);
    testSolidBuilder(context);
    testAffineSolidBuilder(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}
