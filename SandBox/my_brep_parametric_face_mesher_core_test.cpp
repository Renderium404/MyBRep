#include <iostream>
#include <string>
#include <vector>

#include "MyBRep/Mesh/ParametricFaceMesherCore.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Modeling/Wire/WireModeling.h"

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

void testDefaultObjects(TestContext& context)
{
    const MyBRep::ParametricFaceMeshOptions options;
    const MyBRep::ParametricFaceMeshPolicy policy;

    context.expect(options.isValid(), "Default parametric mesh options valid");
    context.expect(!policy.periodicU && !policy.periodicV, "Default parametric policy non-periodic");
    context.expect(policy.rejectSingularParameters, "Default parametric policy rejects singular parameters");
}

void testPlanarCore(TestContext& context)
{
    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createRectangle(10.0, 8.0), 1.0e-8);

    MyBRep::ParametricFaceMeshOptions options;
    options.minimumBoundarySubdivisionDepth = 0;
    options.boundaryChordTolerance = 0.01;
    options.surfaceChordTolerance = 0.01;

    MyBRep::ParametricFaceMeshPolicy policy;
    const MyBRep::FaceMesh mesh = MyBRep::ParametricFaceMesherCore::mesh(face, options, policy);

    context.expect(MyBRep::ParametricFaceMesherCore::canMesh(face, policy), "Generic core accepts non-periodic planar Face");
    context.expect(mesh.isValid(), "Generic core meshes non-periodic planar Face");
    context.expect(mesh.triangleCount() == 2, "Generic core keeps planar rectangle at two triangles");
}

void testPlanarHoleCore(TestContext& context)
{
    std::vector<MyBRep::Topology_Wire> wires;
    wires.push_back(MyBRep::Modeling::createRectangle(12.0, 10.0));
    wires.push_back(MyBRep::Modeling::createCircle(2.0));

    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(wires, 1.0e-8);

    MyBRep::ParametricFaceMeshOptions options;
    options.boundaryChordTolerance = 0.02;
    options.surfaceChordTolerance = 0.02;
    options.minimumBoundarySubdivisionDepth = 1;

    MyBRep::ParametricFaceMeshPolicy policy;
    const MyBRep::FaceMesh mesh = MyBRep::ParametricFaceMesherCore::mesh(face, options, policy);

    context.expect(mesh.isValid(), "Generic core meshes planar Face with hole");
    context.expect(mesh.triangleCount() > 2, "Generic core preserves even-odd hole triangulation");
}

}

int main()
{
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep Parametric Face Mesher Core Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testDefaultObjects(context);
    testPlanarCore(context);
    testPlanarHoleCore(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}
