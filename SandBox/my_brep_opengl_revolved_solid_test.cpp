#include <cmath>
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

double parameterSpan(const MyBRep::FaceMesh& mesh, int axis)
{
    if (mesh.vertices().empty())
    {
        return 0.0;
    }

    double minimum = axis == 0 ? mesh.vertices()[0].parameter.x() : mesh.vertices()[0].parameter.y();
    double maximum = minimum;

    for (std::size_t index = 1; index < mesh.vertices().size(); ++index)
    {
        const double value = axis == 0 ? mesh.vertices()[index].parameter.x() : mesh.vertices()[index].parameter.y();
        minimum = (std::min)(minimum, value);
        maximum = (std::max)(maximum, value);
    }

    return maximum - minimum;
}

void testTopology(TestContext& context, const GeneratedSurfaceSolidFixtures::RevolutionSolidFixture& fixture)
{
    context.expect(fixture.face.isValid(), "Revolution torus Face valid");
    context.expect(fixture.face.geometry().kind() == MyBRep::SurfaceKind::Revolution, "Revolution torus uses Revolution Surface");
    context.expect(fixture.face.wireCount() == 1 && fixture.face.wire(0).edgeCount() == 4, "Revolution torus Face has four seam uses");

    context.expect(fixture.face.wire(0).edge(0).isSame(fixture.vSeamEdge) &&
                   fixture.face.wire(0).edge(2).isSame(fixture.vSeamEdge), "Revolution torus reuses one V seam TEdge");
    context.expect(fixture.face.wire(0).edge(1).isSame(fixture.uSeamEdge) &&
                   fixture.face.wire(0).edge(3).isSame(fixture.uSeamEdge), "Revolution torus reuses one U seam TEdge");

    context.expect(fixture.shell.isValid(), "Revolution torus Shell valid");
    context.expect(fixture.shell.faceCount() == 1, "Revolution torus Shell has one doubly-periodic Face");
    context.expect(fixture.shell.isClosed(), "Revolution torus single-Face Shell closed");
    context.expect(fixture.solid.isValid(), "Revolution torus Topology_Solid valid");
}

void testMeshing(TestContext& context, const GeneratedSurfaceSolidFixtures::RevolutionSolidFixture& fixture)
{
    MyBRep::FaceMeshOptions options;
    options.revolved.boundaryChordTolerance = 0.04;
    options.revolved.surfaceChordTolerance = 0.03;
    options.revolved.minimumBoundarySubdivisionDepth = 1;

    const MyBRep::FaceMesh mesh = MyBRep::FaceMesher::mesh(fixture.face, options);

    context.expect(MyBRep::FaceMesher::canMesh(fixture.face), "FaceMesher accepts full doubly-periodic Revolution Face");
    context.expect(mesh.isValid(), "Full doubly-periodic Revolution Face mesh valid");
    context.expect(mesh.triangleCount() > 8, "Full Revolution torus receives two-direction curvature subdivision");
    context.expect(std::fabs(parameterSpan(mesh, 0) - GeneratedSurfaceSolidFixtures::TwoPi) <= 1.0e-8,
                   "Full Revolution torus keeps one complete U period");
    context.expect(std::fabs(parameterSpan(mesh, 1) - GeneratedSurfaceSolidFixtures::TwoPi) <= 1.0e-8,
                   "Full Revolution torus keeps one complete V period");
}

void testBuilder(TestContext& context, const GeneratedSurfaceSolidFixtures::RevolutionSolidFixture& fixture)
{
    MyBRep::Display::BRepSolidBuildOptions options;
    options.surface.revolvedMeshing.boundaryChordTolerance = 0.04;
    options.surface.revolvedMeshing.surfaceChordTolerance = 0.03;
    options.surface.revolvedMeshing.minimumBoundarySubdivisionDepth = 1;

    BufferGeometry* surface = MyBRep::Display::BRepSolidBuilder::buildSurface(
        fixture.solid, MyMath::Matrix4::identity(), "RevolutionSolidSurface", options);
    BufferGeometry* boundary = MyBRep::Display::BRepSolidBuilder::buildBoundary(
        fixture.solid, MyMath::Matrix4::identity(), "RevolutionSolidBoundary", options);

    context.expect(surface != 0, "BRepSolidBuilder creates Revolution torus surface Geometry");
    context.expect(boundary != 0, "BRepSolidBuilder creates Revolution torus boundary Geometry");
    context.expect(surface != 0 && surface->renderType() == RenderType::Triangles, "Revolution torus surface uses Triangles");
    context.expect(surface != 0 && surface->valuesPerVertex() == 6, "Revolution torus surface uses Position Normal layout");
    context.expect(surface != 0 && surface->hasAttribute(GeometryAttribute::Normal, 3), "Revolution torus surface exposes Normal");
    context.expect(boundary != 0 && boundary->renderType() == RenderType::Lines, "Revolution torus boundary uses Lines");
    context.expect(boundary != 0 && boundary->valuesPerVertex() == 3, "Revolution torus boundary uses Position layout");
    context.expect(surface != 0 && surface->indexCount() > 24, "Revolution torus surface preserves full curvature subdivision");
    context.expect(boundary != 0 && boundary->indexCount() > 4, "Revolution torus boundary contains both unique seam Edges");

    delete surface;
    delete boundary;
}

}

int main()
{
    TestContext context;
    const GeneratedSurfaceSolidFixtures::RevolutionSolidFixture fixture =
        GeneratedSurfaceSolidFixtures::createRevolutionTorusSolid(5.0, 1.5);

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep Revolution Torus Solid Integration Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testTopology(context, fixture);
    testMeshing(context, fixture);
    testBuilder(context, fixture);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;
    return context.failed() == 0 ? 0 : 1;
}
