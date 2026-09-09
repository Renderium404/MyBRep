#include <iostream>
#include <string>

#include "GeneratedSurfaceSolidFixtures.h"

#include "MyBRep/Mesh/FaceMesher.h"
#include "MyBRepOpenGL/Builder/BRepSolidBuilder.h"
#include "MyOpenGL/Resource/BufferGeometry.h"

namespace
{

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

void testTopology(TestContext& context, const GeneratedSurfaceSolidFixtures::ExtrusionSolidFixture& fixture)
{
    context.expect(fixture.sideFace.isValid(), "Extrusion Solid side Face valid");
    context.expect(fixture.bottomFace.isValid() && fixture.topFace.isValid(), "Extrusion Solid planar cap Faces valid");
    context.expect(fixture.sideFace.geometry().kind() == MyBRep::SurfaceKind::Extrusion, "Extrusion Solid side uses Extrusion Surface");
    context.expect(fixture.bottomFace.geometry().kind() == MyBRep::SurfaceKind::Plane &&
                   fixture.topFace.geometry().kind() == MyBRep::SurfaceKind::Plane, "Extrusion Solid caps use Plane Surfaces");

    context.expect(fixture.sideFace.wire(0).edge(0).isSame(fixture.bottomEdge), "Extrusion side shares bottom TEdge");
    context.expect(fixture.bottomFace.wire(0).edge(0).isSame(fixture.bottomEdge), "Bottom cap shares side bottom TEdge");
    context.expect(fixture.sideFace.wire(0).edge(2).isSame(fixture.topEdge), "Extrusion side shares top TEdge");
    context.expect(fixture.topFace.wire(0).edge(0).isSame(fixture.topEdge), "Top cap shares side top TEdge");

    context.expect(fixture.shell.isValid(), "Extrusion Solid Shell valid");
    context.expect(fixture.shell.faceCount() == 3, "Extrusion Solid Shell has three Faces");
    context.expect(fixture.shell.isClosed(), "Extrusion Solid Shell closed");
    context.expect(fixture.solid.isValid(), "Extrusion Topology_Solid valid");
}

void testMeshing(TestContext& context, const GeneratedSurfaceSolidFixtures::ExtrusionSolidFixture& fixture)
{
    MyBRep::FaceMeshOptions options;
    options.extruded.boundaryChordTolerance = 0.03;
    options.extruded.surfaceChordTolerance = 0.02;
    options.extruded.minimumBoundarySubdivisionDepth = 1;

    const MyBRep::FaceMesh sideMesh = MyBRep::FaceMesher::mesh(fixture.sideFace, options);
    const MyBRep::FaceMesh bottomMesh = MyBRep::FaceMesher::mesh(fixture.bottomFace, options);
    const MyBRep::FaceMesh topMesh = MyBRep::FaceMesher::mesh(fixture.topFace, options);

    context.expect(sideMesh.isValid(), "Extrusion Solid side Face meshes through FaceMesher");
    context.expect(bottomMesh.isValid() && topMesh.isValid(), "Extrusion Solid cap Faces mesh through FaceMesher");
    context.expect(sideMesh.triangleCount() > 2, "Extrusion Solid side preserves curved periodic subdivision");
}

void testBuilder(TestContext& context, const GeneratedSurfaceSolidFixtures::ExtrusionSolidFixture& fixture)
{
    MyBRep::Display::BRepSolidBuildOptions options;
    options.surface.extrudedMeshing.boundaryChordTolerance = 0.03;
    options.surface.extrudedMeshing.surfaceChordTolerance = 0.02;
    options.surface.extrudedMeshing.minimumBoundarySubdivisionDepth = 1;

    BufferGeometry* surface = MyBRep::Display::BRepSolidBuilder::buildSurface(
        fixture.solid, MyMath::Matrix4::identity(), "ExtrusionSolidSurface", options);
    BufferGeometry* boundary = MyBRep::Display::BRepSolidBuilder::buildBoundary(
        fixture.solid, MyMath::Matrix4::identity(), "ExtrusionSolidBoundary", options);

    context.expect(surface != 0, "BRepSolidBuilder creates Extrusion Solid surface Geometry");
    context.expect(boundary != 0, "BRepSolidBuilder creates Extrusion Solid boundary Geometry");
    context.expect(surface != 0 && surface->renderType() == RenderType::Triangles, "Extrusion Solid surface uses Triangles");
    context.expect(surface != 0 && surface->valuesPerVertex() == 6, "Extrusion Solid surface uses Position Normal layout");
    context.expect(surface != 0 && surface->hasAttribute(GeometryAttribute::Normal, 3), "Extrusion Solid surface exposes Normal");
    context.expect(boundary != 0 && boundary->renderType() == RenderType::Lines, "Extrusion Solid boundary uses Lines");
    context.expect(boundary != 0 && boundary->valuesPerVertex() == 3, "Extrusion Solid boundary uses Position layout");
    context.expect(surface != 0 && surface->indexCount() > 12, "Extrusion Solid surface contains side and both caps");
    context.expect(boundary != 0 && boundary->indexCount() > 6, "Extrusion Solid boundary contains three unique curved/seam Edges");

    delete surface;
    delete boundary;
}

}

int main()
{
    TestContext context;
    const GeneratedSurfaceSolidFixtures::ExtrusionSolidFixture fixture =
        GeneratedSurfaceSolidFixtures::createExtrusionSolid(3.0, -2.0, 2.0);

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep Extrusion Solid Integration Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testTopology(context, fixture);
    testMeshing(context, fixture);
    testBuilder(context, fixture);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;
    return context.failed() == 0 ? 0 : 1;
}
