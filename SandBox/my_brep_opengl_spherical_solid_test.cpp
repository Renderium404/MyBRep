#include <cmath>
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

const double Pi = 3.1415926535897932384626433832795; // 完整球形Solid测试统一使用的圆周率。
const double HalfPi = Pi * 0.5;                       // 球面南北极对应的V参数绝对值。
const double TwoPi = Pi * 2.0;                        // 球面U参数完整周期。
const double TestTolerance = 1.0e-8;                 // Topology和Curve-on-Surface连接统一使用的三维容差。
const double PositionTolerance = 1.0e-4;             // GLfloat显示Geometry与双精度仿射结果比较允许的三维位置误差。

struct SphereSolidFixture
{
    SphereSolidFixture()
    {
    }

    MyBRep::Topology_Solid solid;
    MyBRep::Topology_Face face;
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

    fixture.face = MyBRep::Modeling::createFace(surface, std::vector<MyBRep::Topology_Wire>(1, MyBRep::Topology_Wire(edges)));

    std::vector<MyBRep::Topology_Face> faces;
    faces.push_back(fixture.face);

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
    options.spherical.boundaryChordTolerance = 0.03;
    options.spherical.surfaceChordTolerance = 0.02;
    options.spherical.minimumBoundarySubdivisionDepth = 1;
    return options;
}

MyBRep::Display::BRepSolidBuildOptions solidBuildOptions()
{
    MyBRep::Display::BRepSolidBuildOptions options;
    options.surface.sphericalMeshing.boundaryChordTolerance = 0.03;
    options.surface.sphericalMeshing.surfaceChordTolerance = 0.02;
    options.surface.sphericalMeshing.minimumBoundarySubdivisionDepth = 1;
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
    const SphereSolidFixture fixture = createSphereSolid(5.0);
    const MyBRep::Topology_Shell shell = fixture.solid.shell(0);
    const std::vector<MyBRep::Topology_Edge> edges = uniqueEdges(shell);

    context.expect(fixture.face.isValid(), "Full sphere Face valid");
    context.expect(fixture.seamEdge.isSeamOnSurface(fixture.face.geometry()), "Full sphere seam keeps two P-Curves");
    context.expect(fixture.solid.isValid(), "Full sphere Solid valid");
    context.expect(fixture.solid.shellCount() == 1, "Full sphere Solid has one Shell");
    context.expect(shell.isValid() && shell.isClosed(), "Full sphere Shell valid and closed");
    context.expect(shell.faceCount() == 1, "Full sphere Shell has one Spherical Face");
    context.expect(edges.size() == 1, "Full sphere Shell has one unique Topology_Edge");
    context.expect(shell.face(0).wireCount() == 1 && shell.face(0).wire(0).edgeCount() == 2, "Full sphere Face uses seam Forward and Reversed");
}

void testFaceMesherInsideSolid(TestContext& context)
{
    const SphereSolidFixture fixture = createSphereSolid(5.0);
    const MyBRep::FaceMesh mesh = MyBRep::FaceMesher::mesh(fixture.solid.shell(0).face(0), faceMeshOptions());

    context.expect(MyBRep::FaceMesher::canMesh(fixture.solid.shell(0).face(0)), "Full sphere Solid Face dispatch supported");
    context.expect(mesh.isValid(), "Full sphere Solid Face mesh valid");
    context.expect(mesh.triangleCount() > 8, "Full sphere Solid Face receives spherical curvature subdivision");
}

void testSolidBuilder(TestContext& context)
{
    const SphereSolidFixture fixture = createSphereSolid(5.0);
    const MyBRep::FaceMesh faceMesh = MyBRep::FaceMesher::mesh(fixture.face, faceMeshOptions());
    const MyBRep::Display::BRepSolidBuildOptions options = solidBuildOptions();

    BufferGeometry* surface = MyBRep::Display::BRepSolidBuilder::buildSurface(fixture.solid, "FullSphereSurface", options);
    BufferGeometry* boundary = MyBRep::Display::BRepSolidBuilder::buildBoundary(fixture.solid, "FullSphereBoundary", options);
    BufferGeometry* seam = MyBRep::Display::BRepWireframeBuilder::build(fixture.seamEdge, "FullSphereSeam", options.wireframe);

    context.expect(surface != 0, "BRepSolidBuilder creates full sphere Surface");
    context.expect(surface != 0 && surface->renderType() == RenderType::Triangles, "Full sphere Surface uses Triangles");
    context.expect(surface != 0 && surface->valuesPerVertex() == 6, "Full sphere Surface uses Position Normal layout");
    context.expect(surface != 0 && surface->vertexCount() == faceMesh.vertexCount(), "Full sphere Solid Surface keeps FaceMesh vertex count");
    context.expect(surface != 0 && surface->indexCount() == static_cast<int>(faceMesh.indices().size()), "Full sphere Solid Surface keeps FaceMesh index count");

    context.expect(boundary != 0, "BRepSolidBuilder creates full sphere Boundary");
    context.expect(boundary != 0 && boundary->renderType() == RenderType::Lines, "Full sphere Boundary uses Lines");
    context.expect(seam != 0 && boundary != 0 && boundary->indexCount() == seam->indexCount(), "Full sphere Boundary globally deduplicates repeated seam Edge");

    delete surface;
    delete boundary;
    delete seam;
}

void testAffineSolidBuilder(TestContext& context)
{
    const SphereSolidFixture fixture = createSphereSolid(5.0);
    const MyBRep::Display::BRepSolidBuildOptions options = solidBuildOptions();

    MyMath::Matrix4 transform = MyMath::Matrix4::identity();
    transform(0, 1) = 0.28; // XY剪切验证完整球形Solid的一般仿射顶点烘焙。
    transform(2, 0) = 0.16; // Z随X变化，使球面成为明显的非TRS仿射形变。
    transform(0, 3) = 12.0; // X方向平移12个模型单位。

    BufferGeometry* localSurface = MyBRep::Display::BRepSolidBuilder::buildSurface(fixture.solid, "LocalSphereSurface", options);
    BufferGeometry* transformedSurface = MyBRep::Display::BRepSolidBuilder::buildSurface(fixture.solid, transform, "AffineSphereSurface", options);
    BufferGeometry* transformedBoundary = MyBRep::Display::BRepSolidBuilder::buildBoundary(fixture.solid, transform, "AffineSphereBoundary", options);

    context.expect(localSurface != 0 && transformedSurface != 0, "Local and affine full sphere Surfaces build");
    context.expect(transformedBoundary != 0, "Affine full sphere Boundary builds");
    const bool affinePositionsCorrect = localSurface != 0 && transformedSurface != 0 && geometryPositionsFollowTransform(*localSurface, *transformedSurface, transform);
    context.expect(affinePositionsCorrect, "Affine full sphere Surface positions follow Matrix4");

    delete localSurface;
    delete transformedSurface;
    delete transformedBoundary;
}

}

int main()
{
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep Full Sphere Solid Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testTopology(context);
    testFaceMesherInsideSolid(context);
    testSolidBuilder(context);
    testAffineSolidBuilder(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}
