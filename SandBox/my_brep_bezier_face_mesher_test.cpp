#include <cmath>
#include <iostream>
#include <string>

#include "FreeformSurfaceTestFixtures.h"

#include "MyBRep/Geometry/Surface/Geometry_BezierSurface.h"
#include "MyBRep/Mesh/BezierFaceMesher.h"

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

MyBRep::BezierFaceMeshOptions testOptions()
{
    MyBRep::BezierFaceMeshOptions options;
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
    const MyBRep::Geometry_BezierSurface& surface =
        static_cast<const MyBRep::Geometry_BezierSurface&>(face.geometry());

    context.expect(surface.kind() == MyBRep::SurfaceKind::Bezier, "Bezier test Face uses Bezier Surface");
    context.expect(surface.uControlPointCount() == 4 && surface.vControlPointCount() == 4, "Bezier test Surface uses 4x4 control net");
    context.expect(surface.uDegree() == 3 && surface.vDegree() == 3, "Bezier test Surface is cubic in U and V");
    context.expect(surface.uDomainStart() == 0.0 && surface.uDomainEnd() == 1.0, "Bezier U domain is [0,1]");
    context.expect(surface.vDomainStart() == 0.0 && surface.vDomainEnd() == 1.0, "Bezier V domain is [0,1]");
    context.expect(!surface.isUPeriodic() && !surface.isVPeriodic(), "Bezier test Surface is non-periodic");
    context.expect(surface.pointAt(0.5, 0.5).z() > 0.5, "Bezier control net produces interior surface curvature");
}

void testMeshing(TestContext& context, const MyBRep::Topology_Face& face)
{
    const MyBRep::BezierFaceMeshOptions options = testOptions();
    const MyBRep::FaceMesh mesh = MyBRep::BezierFaceMesher::mesh(face, options);
    const double referenceArea = referenceSurfaceArea(face.geometry(), 240);

    context.expect(MyBRep::BezierFaceMesher::canMesh(face), "Bezier Face can mesh");
    context.expect(mesh.isValid(), "Bezier Face mesh valid");
    context.expect(mesh.triangleCount() > 2, "Bezier Face receives interior curvature subdivision");
    context.expect(meshArea(mesh) > 16.0, "Bezier curved mesh area exceeds planar footprint");
    context.expect(std::fabs(meshArea(mesh) - referenceArea) <= 0.18, "Bezier mesh area matches differential reference area");
    context.expect(vertexNormalsMatchFace(face, mesh), "Bezier mesh vertex normals match Face");
    context.expect(triangleWindingMatchesNormals(mesh), "Bezier mesh winding matches normals");
}

void testTolerance(TestContext& context, const MyBRep::Topology_Face& face)
{
    MyBRep::BezierFaceMeshOptions loose = testOptions();
    MyBRep::BezierFaceMeshOptions tight = testOptions();
    loose.surfaceChordTolerance = 0.20;
    tight.surfaceChordTolerance = 0.01;

    const MyBRep::FaceMesh looseMesh = MyBRep::BezierFaceMesher::mesh(face, loose);
    const MyBRep::FaceMesh tightMesh = MyBRep::BezierFaceMesher::mesh(face, tight);

    context.expect(looseMesh.isValid() && tightMesh.isValid(), "Loose and tight Bezier meshes valid");
    context.expect(tightMesh.triangleCount() > looseMesh.triangleCount(), "Tighter Bezier surface tolerance increases triangle count");
}

void testReversed(TestContext& context, const MyBRep::Topology_Face& face)
{
    const MyBRep::Topology_Face reversedFace = face.reversed();
    const MyBRep::FaceMesh forwardMesh = MyBRep::BezierFaceMesher::mesh(face, testOptions());
    const MyBRep::FaceMesh reversedMesh = MyBRep::BezierFaceMesher::mesh(reversedFace, testOptions());

    context.expect(forwardMesh.isValid() && reversedMesh.isValid(), "Forward and Reversed Bezier meshes valid");
    context.expect(std::fabs(meshArea(forwardMesh) - meshArea(reversedMesh)) <= 1.0e-8, "Reversed Bezier preserves area");
    context.expect(vertexNormalsMatchFace(reversedFace, reversedMesh), "Reversed Bezier normals match reversed Face");
    context.expect(triangleWindingMatchesNormals(reversedMesh), "Reversed Bezier winding follows reversed normal");
}

}

int main()
{
    TestContext context;
    const MyBRep::Topology_Face face = FreeformSurfaceTestFixtures::createBezierFace();

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep Bezier Face Mesher Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    context.expect(testOptions().isValid(), "Default BezierFaceMeshOptions valid");
    testGeometryContract(context, face);
    testMeshing(context, face);
    testTolerance(context, face);
    testReversed(context, face);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}