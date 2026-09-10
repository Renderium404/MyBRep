#include <iostream>
#include <string>

#include "FreeformSurfaceTestFixtures.h"

#include "MyBRep/Mesh/BSplineFaceMesher.h"
#include "MyBRep/Mesh/BezierFaceMesher.h"
#include "MyBRep/Mesh/FaceMesher.h"

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

bool sameMeshShape(const MyBRep::FaceMesh& first, const MyBRep::FaceMesh& second)
{
    return first.isValid() &&
           second.isValid() &&
           first.vertexCount() == second.vertexCount() &&
           first.triangleCount() == second.triangleCount() &&
           first.indices() == second.indices();
}

void testBezierDispatch(TestContext& context)
{
    const MyBRep::Topology_Face face = FreeformSurfaceTestFixtures::createBezierFace();

    MyBRep::FaceMeshOptions options;
    options.bezier.boundaryChordTolerance = 0.02;
    options.bezier.surfaceChordTolerance = 0.02;
    options.bezier.minimumBoundarySubdivisionDepth = 1;

    const MyBRep::FaceMesh direct = MyBRep::BezierFaceMesher::mesh(face, options.bezier);
    const MyBRep::FaceMesh dispatched = MyBRep::FaceMesher::mesh(face, options);

    context.expect(MyBRep::FaceMesher::canMesh(face), "FaceMesher recognizes Bezier Face");
    context.expect(dispatched.isValid(), "FaceMesher dispatches Bezier Face");
    context.expect(sameMeshShape(direct, dispatched), "Bezier dispatch preserves BezierFaceMesher result");
    context.expect(dispatched.triangleCount() > 2, "Bezier dispatch preserves interior curvature subdivision");
}

void testBezierTolerance(TestContext& context)
{
    const MyBRep::Topology_Face face = FreeformSurfaceTestFixtures::createBezierFace();
    MyBRep::FaceMeshOptions loose;
    MyBRep::FaceMeshOptions tight;

    loose.bezier.surfaceChordTolerance = 0.20;
    tight.bezier.surfaceChordTolerance = 0.01;

    const MyBRep::FaceMesh looseMesh = MyBRep::FaceMesher::mesh(face, loose);
    const MyBRep::FaceMesh tightMesh = MyBRep::FaceMesher::mesh(face, tight);

    context.expect(looseMesh.isValid() && tightMesh.isValid(), "FaceMesher forwards valid Bezier options");
    context.expect(tightMesh.triangleCount() > looseMesh.triangleCount(), "FaceMesher forwards Bezier surface tolerance");
}

void testBSplineDispatch(TestContext& context)
{
    const MyBRep::Topology_Face face = FreeformSurfaceTestFixtures::createBSplineFace();

    MyBRep::FaceMeshOptions options;
    options.bspline.boundaryChordTolerance = 0.02;
    options.bspline.surfaceChordTolerance = 0.02;
    options.bspline.minimumBoundarySubdivisionDepth = 1;

    const MyBRep::FaceMesh direct = MyBRep::BSplineFaceMesher::mesh(face, options.bspline);
    const MyBRep::FaceMesh dispatched = MyBRep::FaceMesher::mesh(face, options);

    context.expect(MyBRep::FaceMesher::canMesh(face), "FaceMesher recognizes B-Spline Face");
    context.expect(dispatched.isValid(), "FaceMesher dispatches B-Spline Face");
    context.expect(sameMeshShape(direct, dispatched), "B-Spline dispatch preserves BSplineFaceMesher result");
    context.expect(dispatched.triangleCount() > 2, "B-Spline dispatch preserves multi-span curvature subdivision");
}

void testBSplineTolerance(TestContext& context)
{
    const MyBRep::Topology_Face face = FreeformSurfaceTestFixtures::createBSplineFace();
    MyBRep::FaceMeshOptions loose;
    MyBRep::FaceMeshOptions tight;

    loose.bspline.surfaceChordTolerance = 0.20;
    tight.bspline.surfaceChordTolerance = 0.01;

    const MyBRep::FaceMesh looseMesh = MyBRep::FaceMesher::mesh(face, loose);
    const MyBRep::FaceMesh tightMesh = MyBRep::FaceMesher::mesh(face, tight);

    context.expect(looseMesh.isValid() && tightMesh.isValid(), "FaceMesher forwards valid B-Spline options");
    context.expect(tightMesh.triangleCount() > looseMesh.triangleCount(), "FaceMesher forwards B-Spline surface tolerance");
}

}

int main()
{
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep Freeform Surface FaceMesher Dispatch Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    context.expect(MyBRep::FaceMeshOptions().isValid(), "Default eight-surface FaceMeshOptions valid");
    testBezierDispatch(context);
    testBezierTolerance(context);
    testBSplineDispatch(context);
    testBSplineTolerance(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}