#include <iostream>
#include <string>

#include "MyBRep/Mesh/ExtrudedFaceMesher.h"
#include "MyBRep/Mesh/RevolvedFaceMesher.h"
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

}

int main()
{
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep Extruded / Revolved Face Mesher Contract Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    const MyBRep::ExtrudedFaceMeshOptions extrudedOptions;
    const MyBRep::RevolvedFaceMeshOptions revolvedOptions;
    const MyBRep::Topology_Face plane = MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createRectangle(8.0, 6.0), 1.0e-8);

    context.expect(extrudedOptions.isValid(), "Default ExtrudedFaceMeshOptions valid");
    context.expect(revolvedOptions.isValid(), "Default RevolvedFaceMeshOptions valid");
    context.expect(!MyBRep::ExtrudedFaceMesher::canMesh(plane), "ExtrudedFaceMesher rejects Plane Face");
    context.expect(MyBRep::ExtrudedFaceMesher::mesh(plane).isEmpty(), "ExtrudedFaceMesher returns empty mesh for Plane Face");
    context.expect(!MyBRep::RevolvedFaceMesher::canMesh(plane), "RevolvedFaceMesher rejects Plane Face");
    context.expect(MyBRep::RevolvedFaceMesher::mesh(plane).isEmpty(), "RevolvedFaceMesher returns empty mesh for Plane Face");

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}
