#include <cmath>
#include <iostream>
#include <string>

#include "FreeformSurfaceTestFixtures.h"

#include "MyBRep/Geometry/Surface/Geometry_BSplineSurface.h"
#include "MyBRep/Mesh/BSplineFaceMesher.h"

namespace
{

const double NormalTolerance = 1.0e-6; // FaceMesh顶点法向比较容差。

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
        const MyMath::Vector3& first = mesh.vertices()[mesh.indices()[index]].position;
        const MyMath::Vector3& second = mesh.vertices()[mesh.indices()[index + 1]].position;
        const MyMath::Vector3& third = mesh.vertices()[mesh.indices()[index + 2]].position;
        area += MyMath::Vector3::cross(second - first, third - first).length() * 0.5;
    }

    return area;
}

double referenceSurfaceArea(const MyBRep::Geometry_Surface& surface, int divisions)
{
    double area = 0.0;
    const double u0 = surface.uDomainStart();
    const double u1 = surface.uDomainEnd();
    const double v0 = surface.vDomainStart();
    const double v1 = surface.vDomainEnd();
    const double du = (u1 - u0) / static_cast<double>(divisions);
    const double dv = (v1 - v0) / static_cast<double>(divisions);

    for (int vIndex = 0; vIndex < divisions; ++vIndex)
    {
        const double v = v0 + (static_cast<double>(vIndex) + 0.5) * dv;

        for (int uIndex = 0; uIndex < divisions; ++uIndex)
        {
            const double u = u0 + (static_cast<double>(uIndex) + 0.5) * du;
            const MyMath::Vector3 derivativeU = surface.firstDerivativeUAt(u, v);
            const MyMath::Vector3 derivativeV = surface.firstDerivativeVAt(u, v);
            area += MyMath::Vector3::cross(derivativeU, derivativeV).length() * du * dv;
        }
    }

    return area;
}

bool vertexNormalsMatchFace(const MyBRep::Topology_Face& face, const MyBRep::FaceMesh& mesh)
{
    if (!mesh.isValid())
    {
        return false;
    }

    for (std::size_t index = 0; index < mesh.vertices().size(); ++index)
    {
        const MyBRep::FaceMeshVertex& vertex = mesh.vertices()[index];
        const MyMath::Vector3 expected = face.normalAt(vertex.parameter.x(), vertex.parameter.y());

        if (!vertex.normal.isEqualTo(expected, NormalTolerance))
        {
            return false;
        }
    }

    return true;
}

bool triangleWindingMatchesNormals(const MyBRep::FaceMesh& mesh)
{
    if (!mesh.isValid())
    {
        return false;
    }

    for (std::size_t index = 0; index < mesh.indices().size(); index += 3)
    {
        const MyBRep::FaceMeshVertex& first = mesh.vertices()[mesh.indices()[index]];
        const MyBRep::FaceMeshVertex& second = mesh.vertices()[mesh.indices()[index + 1]];
        const MyBRep::FaceMeshVertex& third = mesh.vertices()[mesh.indices()[index + 2]];
        const MyMath::Vector3 triangleNormal = MyMath::Vector3::cross(second.position - first.position, third.position - first.position);
        const MyMath::Vector3 averageNormal = first.normal + second.normal + third.normal;

        if (!triangleNormal.isVector(0.0) || MyMath::Vector3::dot(triangleNormal, averageNormal) <= 0.0)
        {
            return false;
        }
    }

    return true;
}

MyBRep::BSplineFaceMeshOptions testOptions()
{
    MyBRep::BSplineFaceMeshOptions options;
    options.boundaryChordTolerance = 0.02;
    options.surfaceChordTolerance = 0.02;
    options.geometricTolerance = 1.0e-10;
    options.minimumBoundarySubdivisionDepth = 1;
    options.maximumBoundarySubdivisionDepth = 12;
    options.maximumSurfaceSubdivisionRounds = 12;
    return options;
}

void testGeometryContract(TestContext& context, const MyBRep::Topology_Face& face)
{
    const MyBRep::Geometry_BSplineSurface& surface =
        static_cast<const MyBRep::Geometry_BSplineSurface&>(face.geometry());

    context.expect(surface.kind() == MyBRep::SurfaceKind::BSpline, "B-Spline test Face uses B-Spline Surface");
    context.expect(surface.uDegree() == 2 && surface.vDegree() == 2, "B-Spline test Surface is quadratic in U and V");
    context.expect(surface.uControlPointCount() == 4 && surface.vControlPointCount() == 4, "B-Spline test Surface uses 4x4 control net");
    context.expect(surface.uKnotCount() == 7 && surface.vKnotCount() == 7, "B-Spline test Surface uses seven knots per direction");
    context.expect(surface.uDomainStart() == 0.0 && surface.uDomainEnd() == 1.0, "B-Spline U domain is [0,1]");
    context.expect(surface.vDomainStart() == 0.0 && surface.vDomainEnd() == 1.0, "B-Spline V domain is [0,1]");
    context.expect(surface.uKnot(3) == 0.5 && surface.vKnot(3) == 0.5, "B-Spline test Surface crosses simple internal U/V knots");
    context.expect(!surface.isUPeriodic() && !surface.isVPeriodic(), "B-Spline test Surface is non-periodic");
    context.expect(surface.pointAt(0.5, 0.5).z() > 0.5, "B-Spline control net produces interior surface curvature");
    context.expect(surface.normalAt(0.5, 0.5).isUnit(), "B-Spline simple internal knot keeps unique first-order normal");
}

void testMeshing(TestContext& context, const MyBRep::Topology_Face& face)
{
    const MyBRep::BSplineFaceMeshOptions options = testOptions();
    const MyBRep::FaceMesh mesh = MyBRep::BSplineFaceMesher::mesh(face, options);
    const double referenceArea = referenceSurfaceArea(face.geometry(), 240);

    context.expect(MyBRep::BSplineFaceMesher::canMesh(face), "B-Spline Face can mesh");
    context.expect(mesh.isValid(), "B-Spline Face mesh valid");
    context.expect(mesh.triangleCount() > 2, "B-Spline Face receives multi-span interior curvature subdivision");
    context.expect(meshArea(mesh) > 16.0, "B-Spline curved mesh area exceeds planar footprint");
    context.expect(std::fabs(meshArea(mesh) - referenceArea) <= 0.20, "B-Spline mesh area matches differential reference area");
    context.expect(vertexNormalsMatchFace(face, mesh), "B-Spline mesh vertex normals match Face");
    context.expect(triangleWindingMatchesNormals(mesh), "B-Spline mesh winding matches normals");
}

void testTolerance(TestContext& context, const MyBRep::Topology_Face& face)
{
    MyBRep::BSplineFaceMeshOptions loose = testOptions();
    MyBRep::BSplineFaceMeshOptions tight = testOptions();
    loose.surfaceChordTolerance = 0.20;
    tight.surfaceChordTolerance = 0.01;

    const MyBRep::FaceMesh looseMesh = MyBRep::BSplineFaceMesher::mesh(face, loose);
    const MyBRep::FaceMesh tightMesh = MyBRep::BSplineFaceMesher::mesh(face, tight);

    context.expect(looseMesh.isValid() && tightMesh.isValid(), "Loose and tight B-Spline meshes valid");
    context.expect(tightMesh.triangleCount() > looseMesh.triangleCount(), "Tighter B-Spline surface tolerance increases triangle count");
}

void testReversed(TestContext& context, const MyBRep::Topology_Face& face)
{
    const MyBRep::Topology_Face reversedFace = face.reversed();
    const MyBRep::FaceMesh forwardMesh = MyBRep::BSplineFaceMesher::mesh(face, testOptions());
    const MyBRep::FaceMesh reversedMesh = MyBRep::BSplineFaceMesher::mesh(reversedFace, testOptions());

    context.expect(forwardMesh.isValid() && reversedMesh.isValid(), "Forward and Reversed B-Spline meshes valid");
    context.expect(std::fabs(meshArea(forwardMesh) - meshArea(reversedMesh)) <= 1.0e-8, "Reversed B-Spline preserves area");
    context.expect(vertexNormalsMatchFace(reversedFace, reversedMesh), "Reversed B-Spline normals match reversed Face");
    context.expect(triangleWindingMatchesNormals(reversedMesh), "Reversed B-Spline winding follows reversed normal");
}

}

int main()
{
    TestContext context;
    const MyBRep::Topology_Face face = FreeformSurfaceTestFixtures::createBSplineFace();

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep B-Spline Face Mesher Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    context.expect(testOptions().isValid(), "Default BSplineFaceMeshOptions valid");
    testGeometryContract(context, face);
    testMeshing(context, face);
    testTolerance(context, face);
    testReversed(context, face);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}