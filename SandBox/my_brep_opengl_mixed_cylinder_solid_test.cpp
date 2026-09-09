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
#include "MyBRepOpenGL/Builder/BRepWireframeBuilder.h"
#include "MyOpenGL/Resource/BufferGeometry.h"

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

MyMath::Vector3 geometryPosition(const BufferGeometry& geometry, unsigned int vertexIndex)
{
    const std::size_t offset = static_cast<std::size_t>(vertexIndex) * 6;
    const std::vector<GLfloat>& data = geometry.vertexData();
    return MyMath::Vector3(data[offset], data[offset + 1], data[offset + 2]);
}

void testTopology(TestContext& context)
{
    const CylinderSolidFixture fixture = createCylinderSolid(5.0, 8.0);
    const MyBRep::Topology_Shell shell = fixture.solid.shell(0);
    const std::vector<MyBRep::Topology_Edge> edges = uniqueEdges(shell);

    context.expect(fixture.solid.isValid(), "Mixed cylinder Solid valid");
    context.expect(fixture.solid.shellCount() == 1, "Mixed cylinder Solid has one Shell");
    context.expect(shell.isValid() && shell.isClosed(), "Mixed cylinder Shell valid and closed");
    context.expect(shell.faceCount() == 3, "Mixed cylinder Shell has three Faces");
    context.expect(edges.size() == 3, "Mixed cylinder Shell has three unique Topology_Edges");
    context.expect(fixture.seamEdge.isSeamOnSurface(shell.face(2).geometry()), "Cylinder side seam keeps two P-Curves");
}

void testFaceDispatchInsideSolid(TestContext& context)
{
    const CylinderSolidFixture fixture = createCylinderSolid(5.0, 8.0);
    const MyBRep::Topology_Shell shell = fixture.solid.shell(0);

    MyBRep::FaceMeshOptions options;
    options.planar.chordTolerance = 0.02;
    options.cylindrical.boundaryChordTolerance = 0.02;
    options.cylindrical.surfaceChordTolerance = 0.02;
    options.cylindrical.minimumBoundarySubdivisionDepth = 1;

    std::size_t totalVertices = 0;
    std::size_t totalTriangles = 0;
    bool allValid = true;
    bool hasCurvedFace = false;

    for (std::size_t index = 0; index < shell.faceCount(); ++index)
    {
        const MyBRep::Topology_Face face = shell.face(index);
        const MyBRep::FaceMesh mesh = MyBRep::FaceMesher::mesh(face, options);

        allValid = allValid && mesh.isValid();
        totalVertices += mesh.vertexCount();
        totalTriangles += mesh.triangleCount();

        if (face.geometry().kind() == MyBRep::SurfaceKind::Cylindrical)
        {
            hasCurvedFace = mesh.triangleCount() > 2;
        }
    }

    context.expect(allValid, "Plane and Cylindrical Faces all mesh through FaceMesher");
    context.expect(hasCurvedFace, "Cylinder side receives curvature subdivision");
    context.expect(totalVertices > 0 && totalTriangles > 0, "Mixed Face meshes produce aggregate surface data");
}

void testSolidSurfaceAggregation(TestContext& context)
{
    const CylinderSolidFixture fixture = createCylinderSolid(5.0, 8.0);
    const MyBRep::Topology_Shell shell = fixture.solid.shell(0);

    MyBRep::Display::BRepSolidBuildOptions options;
    options.surface.meshing.chordTolerance = 0.02;
    options.surface.cylindricalMeshing.boundaryChordTolerance = 0.02;
    options.surface.cylindricalMeshing.surfaceChordTolerance = 0.02;
    options.surface.cylindricalMeshing.minimumBoundarySubdivisionDepth = 1;
    options.wireframe.chordTolerance = 0.02;

    MyBRep::FaceMeshOptions faceOptions;
    faceOptions.planar = options.surface.meshing;
    faceOptions.cylindrical = options.surface.cylindricalMeshing;

    std::size_t expectedVertices = 0;
    std::size_t expectedTriangles = 0;

    for (std::size_t index = 0; index < shell.faceCount(); ++index)
    {
        const MyBRep::FaceMesh mesh = MyBRep::FaceMesher::mesh(shell.face(index), faceOptions);
        expectedVertices += mesh.vertexCount();
        expectedTriangles += mesh.triangleCount();
    }

    BufferGeometry* surface = MyBRep::Display::BRepSolidBuilder::buildSurface(fixture.solid, "MixedCylinderSurface", options);

    context.expect(surface != 0, "BRepSolidBuilder creates mixed Plane Cylinder surface");
    context.expect(surface != 0 && surface->renderType() == RenderType::Triangles, "Mixed cylinder surface uses Triangles");
    context.expect(surface != 0 && surface->vertexCount() == static_cast<int>(expectedVertices), "Solid surface preserves sum of Face-local vertices");
    context.expect(surface != 0 && surface->indexCount() == static_cast<int>(expectedTriangles * 3), "Solid surface preserves sum of Face triangles");

    delete surface;
}

void testSolidBoundaryDeduplication(TestContext& context)
{
    const CylinderSolidFixture fixture = createCylinderSolid(5.0, 8.0);
    const MyBRep::Topology_Shell shell = fixture.solid.shell(0);
    const std::vector<MyBRep::Topology_Edge> edges = uniqueEdges(shell);

    MyBRep::Display::BRepSolidBuildOptions options;
    options.wireframe.chordTolerance = 0.02;

    int expectedIndexCount = 0;

    for (std::size_t index = 0; index < edges.size(); ++index)
    {
        BufferGeometry* edgeGeometry = MyBRep::Display::BRepWireframeBuilder::build(edges[index], "UniqueEdge", options.wireframe);

        if (edgeGeometry != 0)
        {
            expectedIndexCount += edgeGeometry->indexCount();
        }

        delete edgeGeometry;
    }

    BufferGeometry* boundary = MyBRep::Display::BRepSolidBuilder::buildBoundary(fixture.solid, "MixedCylinderBoundary", options);

    context.expect(boundary != 0, "BRepSolidBuilder creates mixed cylinder boundary");
    context.expect(boundary != 0 && boundary->renderType() == RenderType::Lines, "Mixed cylinder boundary uses Lines");
    context.expect(edges.size() == 3 && expectedIndexCount > 0, "Independent unique Edge boundary reference valid");
    context.expect(boundary != 0 && boundary->indexCount() == expectedIndexCount, "Solid boundary emits each shared circle and seam Edge once");

    delete boundary;
}

void testGeneralAffinePlacement(TestContext& context)
{
    const CylinderSolidFixture fixture = createCylinderSolid(5.0, 8.0);

    MyBRep::Display::BRepSolidBuildOptions options;
    options.surface.meshing.chordTolerance = 0.03;
    options.surface.cylindricalMeshing.boundaryChordTolerance = 0.03;
    options.surface.cylindricalMeshing.surfaceChordTolerance = 0.03;
    options.wireframe.chordTolerance = 0.03;

    MyMath::Matrix4 transform = MyMath::Matrix4::identity();
    transform(0, 1) = 0.30; // XY剪切验证圆柱和端盖统一的一般仿射位置烘焙。
    transform(2, 0) = 0.18; // Z随X变化，使圆柱轴和圆截面同时发生一般仿射变化。
    transform(0, 3) = 11.0; // X方向平移11个模型单位。

    BufferGeometry* localSurface = MyBRep::Display::BRepSolidBuilder::buildSurface(fixture.solid, "LocalMixedCylinder", options);
    BufferGeometry* transformedSurface = MyBRep::Display::BRepSolidBuilder::buildSurface(fixture.solid, transform, "AffineMixedCylinder", options);
    BufferGeometry* transformedBoundary = MyBRep::Display::BRepSolidBuilder::buildBoundary(fixture.solid, transform, "AffineMixedCylinderBoundary", options);

    const bool created = localSurface != 0 && transformedSurface != 0 && transformedBoundary != 0 && localSurface->vertexCount() == transformedSurface->vertexCount();
    bool positionsCorrect = created;

    if (created)
    {
        for (int index = 0; index < localSurface->vertexCount(); ++index)
        {
            const MyMath::Vector3 expected = transform.transformPoint(geometryPosition(*localSurface, static_cast<unsigned int>(index)));
            const MyMath::Vector3 actual = geometryPosition(*transformedSurface, static_cast<unsigned int>(index));

            if (!actual.isEqualTo(expected, 1.0e-5))
            {
                positionsCorrect = false;
                break;
            }
        }
    }

    context.expect(created, "General affine mixed cylinder Surface and Boundary created");
    context.expect(positionsCorrect, "General affine transform is baked into all mixed cylinder surface vertices");

    delete localSurface;
    delete transformedSurface;
    delete transformedBoundary;
}

}

int main()
{
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep Mixed-Surface Cylinder Solid Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testTopology(context);
    testFaceDispatchInsideSolid(context);
    testSolidSurfaceAggregation(context);
    testSolidBoundaryDeduplication(context);
    testGeneralAffinePlacement(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}
