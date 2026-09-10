#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

#include "RevolutionAxisSingularitySolidFixture.h"

#include "MyBRep/Mesh/FaceMesher.h"
#include "MyBRepOpenGL/Builder/BRepSolidBuilder.h"
#include "MyOpenGL/Resource/BufferGeometry.h"

namespace
{

const double AreaTolerance = 0.9;      // 与v2 FaceMesher完整双pole面积测试保持一致。
const double PositionTolerance = 1e-6; // pole三维位置比较容差。

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

double meshArea(const MyBRep::FaceMesh& mesh)
{
    double area = 0.0;

    for (std::size_t index = 0; index < mesh.indices().size(); index += 3)
    {
        const MyMath::Vector3& first =
            mesh.vertices()[mesh.indices()[index]].position;
        const MyMath::Vector3& second =
            mesh.vertices()[mesh.indices()[index + 1]].position;
        const MyMath::Vector3& third =
            mesh.vertices()[mesh.indices()[index + 2]].position;

        area += MyMath::Vector3::cross(
            second - first,
            third - first).length() * 0.5;
    }

    return area;
}

std::size_t countVerticesAtPosition(
    const MyBRep::FaceMesh& mesh,
    const MyMath::Vector3& position)
{
    std::size_t count = 0;

    for (std::size_t index = 0; index < mesh.vertices().size(); ++index)
    {
        if (mesh.vertices()[index].position.isEqualTo(
                position,
                PositionTolerance))
        {
            ++count;
        }
    }

    return count;
}

double parameterUSpan(const MyBRep::FaceMesh& mesh)
{
    if (mesh.vertices().empty())
    {
        return 0.0;
    }

    double minimum = mesh.vertices()[0].parameter.x();
    double maximum = minimum;

    for (std::size_t index = 1; index < mesh.vertices().size(); ++index)
    {
        minimum = (std::min)(
            minimum,
            mesh.vertices()[index].parameter.x());
        maximum = (std::max)(
            maximum,
            mesh.vertices()[index].parameter.x());
    }

    return maximum - minimum;
}

bool triangleWindingMatchesNormals(const MyBRep::FaceMesh& mesh)
{
    if (!mesh.isValid())
    {
        return false;
    }

    for (std::size_t index = 0; index < mesh.indices().size(); index += 3)
    {
        const unsigned int firstIndex = mesh.indices()[index];
        const unsigned int secondIndex = mesh.indices()[index + 1];
        const unsigned int thirdIndex = mesh.indices()[index + 2];

        const MyMath::Vector3& first =
            mesh.vertices()[firstIndex].position;
        const MyMath::Vector3& second =
            mesh.vertices()[secondIndex].position;
        const MyMath::Vector3& third =
            mesh.vertices()[thirdIndex].position;

        const MyMath::Vector3 triangleNormal =
            MyMath::Vector3::cross(
                second - first,
                third - first);

        if (!triangleNormal.isVector(0.0))
        {
            return false;
        }

        const MyMath::Vector3 averageNormal =
            (mesh.vertices()[firstIndex].normal +
             mesh.vertices()[secondIndex].normal +
             mesh.vertices()[thirdIndex].normal).normalized(0.0);

        if (MyMath::Vector3::dot(
                triangleNormal,
                averageNormal) <= 0.0)
        {
            return false;
        }
    }

    return true;
}

void testTopology(
    TestContext& context,
    const RevolutionAxisSingularitySolidFixture::Fixture& fixture)
{
    context.expect(
        fixture.face.isValid(),
        "Axis-singularity Revolution Face valid");
    context.expect(
        fixture.face.geometry().kind() == MyBRep::SurfaceKind::Revolution,
        "Axis-singularity Solid uses Revolution Surface");
    context.expect(
        fixture.face.wireCount() == 1 &&
        fixture.face.wire(0).edgeCount() == 2,
        "Axis-singularity Face uses one seam TEdge twice");
    context.expect(
        fixture.face.wire(0).edge(0).isSame(fixture.seamEdge) &&
        fixture.face.wire(0).edge(1).isSame(fixture.seamEdge),
        "Axis-singularity Face reuses the same seam TEdge");
    context.expect(
        fixture.shell.isValid(),
        "Axis-singularity Revolution Shell valid");
    context.expect(
        fixture.shell.faceCount() == 1,
        "Axis-singularity Revolution Shell has one Face");
    context.expect(
        fixture.shell.isClosed(),
        "Axis-singularity single-Face Revolution Shell closed");
    context.expect(
        fixture.solid.isValid(),
        "Axis-singularity Revolution Topology_Solid valid");
}

void testMeshing(
    TestContext& context,
    const RevolutionAxisSingularitySolidFixture::Fixture& fixture,
    double radius)
{
    MyBRep::FaceMeshOptions options;
    options.revolved.boundaryChordTolerance = 0.05;
    options.revolved.surfaceChordTolerance = 0.04;
    options.revolved.geometricTolerance = 1.0e-10;
    options.revolved.minimumBoundarySubdivisionDepth = 1;
    options.revolved.maximumBoundarySubdivisionDepth = 12;
    options.revolved.maximumSurfaceSubdivisionRounds = 12;

    const MyBRep::FaceMesh mesh =
        MyBRep::FaceMesher::mesh(fixture.face, options);

    const double expectedArea =
        4.0 *
        RevolutionAxisSingularitySolidFixture::Pi *
        radius *
        radius;

    context.expect(
        MyBRep::FaceMesher::canMesh(fixture.face),
        "FaceMesher accepts axis-singularity Revolution Solid Face");
    context.expect(
        mesh.isValid(),
        "Axis-singularity Revolution Solid Face mesh valid");
    context.expect(
        mesh.triangleCount() > 8,
        "Axis-singularity Revolution Solid receives curvature subdivision");
    context.expect(
        std::fabs(
            parameterUSpan(mesh) -
            RevolutionAxisSingularitySolidFixture::TwoPi) <= 1.0e-8,
        "Axis-singularity Revolution Solid keeps one complete U period");
    context.expect(
        countVerticesAtPosition(
            mesh,
            fixture.northVertex.point()) >= 2,
        "Axis-singularity Solid keeps north-pole limit vertices");
    context.expect(
        countVerticesAtPosition(
            mesh,
            fixture.southVertex.point()) >= 2,
        "Axis-singularity Solid keeps south-pole limit vertices");
    context.expect(
        std::fabs(meshArea(mesh) - expectedArea) <= AreaTolerance,
        "Axis-singularity Revolution Solid Face mesh area");
    context.expect(
        triangleWindingMatchesNormals(mesh),
        "Axis-singularity Revolution Solid Face winding");
}

void testBuilder(
    TestContext& context,
    const RevolutionAxisSingularitySolidFixture::Fixture& fixture)
{
    MyBRep::Display::BRepSolidBuildOptions options;
    options.surface.revolvedMeshing.boundaryChordTolerance = 0.05;
    options.surface.revolvedMeshing.surfaceChordTolerance = 0.04;
    options.surface.revolvedMeshing.geometricTolerance = 1.0e-10;
    options.surface.revolvedMeshing.minimumBoundarySubdivisionDepth = 1;
    options.surface.revolvedMeshing.maximumBoundarySubdivisionDepth = 12;
    options.surface.revolvedMeshing.maximumSurfaceSubdivisionRounds = 12;

    BufferGeometry* surface =
        MyBRep::Display::BRepSolidBuilder::buildSurface(
            fixture.solid,
            MyMath::Matrix4::identity(),
            "AxisSingularityRevolutionSurface",
            options);
    BufferGeometry* boundary =
        MyBRep::Display::BRepSolidBuilder::buildBoundary(
            fixture.solid,
            MyMath::Matrix4::identity(),
            "AxisSingularityRevolutionBoundary",
            options);

    context.expect(
        surface != 0,
        "BRepSolidBuilder creates axis-singularity Revolution surface");
    context.expect(
        boundary != 0,
        "BRepSolidBuilder creates axis-singularity Revolution boundary");
    context.expect(
        surface != 0 &&
        surface->renderType() == RenderType::Triangles,
        "Axis-singularity Revolution surface uses Triangles");
    context.expect(
        surface != 0 &&
        surface->valuesPerVertex() == 6,
        "Axis-singularity Revolution surface uses Position Normal layout");
    context.expect(
        surface != 0 &&
        surface->hasAttribute(GeometryAttribute::Normal, 3),
        "Axis-singularity Revolution surface exposes Normal");
    context.expect(
        boundary != 0 &&
        boundary->renderType() == RenderType::Lines,
        "Axis-singularity Revolution boundary uses Lines");
    context.expect(
        boundary != 0 &&
        boundary->valuesPerVertex() == 3,
        "Axis-singularity Revolution boundary uses Position layout");
    context.expect(
        surface != 0 &&
        surface->indexCount() > 24,
        "Axis-singularity Revolution surface preserves full subdivision");
    context.expect(
        boundary != 0 &&
        boundary->indexCount() > 2,
        "Axis-singularity Revolution boundary contains seam geometry");

    delete surface;
    delete boundary;
}

}

int main()
{
    TestContext context;
    const double radius = 4.0;

    const RevolutionAxisSingularitySolidFixture::Fixture fixture =
        RevolutionAxisSingularitySolidFixture::create(radius);

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep Revolution Axis Singularity Solid Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testTopology(context, fixture);
    testMeshing(context, fixture, radius);
    testBuilder(context, fixture);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed()
              << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}