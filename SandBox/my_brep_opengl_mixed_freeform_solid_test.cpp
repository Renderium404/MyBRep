#include <iostream>
#include <string>
#include <vector>

#include "MixedFreeformSolidFixture.h"
#include "MyBRep/Mesh/FaceMesher.h"
#include "MyBRepOpenGL/Builder/BRepSolidBuilder.h"
#include "MyOpenGL/Resource/BufferGeometry.h"

namespace
{
class Ctx
{
public:
    Ctx() : p(0), f(0) {}
    void expect(bool ok, const std::string& name) { if (ok) { ++p; std::cout << "[PASS] " << name << std::endl; } else { ++f; std::cout << "[FAIL] " << name << std::endl; } }
    int passed() const { return p; }
    int failed() const { return f; }
private:
    int p, f;
};

bool contains(const MyBRep::Topology_Face& face, const MyBRep::Topology_Edge& edge)
{
    for (std::size_t w = 0; w < face.wireCount(); ++w)
        for (std::size_t e = 0; e < face.wire(w).edgeCount(); ++e)
            if (face.wire(w).edge(e).isSame(edge)) return true;
    return false;
}
}

int main()
{
    Ctx c;
    const MixedFreeformSolidFixture::Fixture x = MixedFreeformSolidFixture::create();
    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep Mixed Freeform Solid Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    c.expect(x.topFace.isValid() && x.topFace.geometry().kind() == MyBRep::SurfaceKind::Bezier, "Bezier top Face valid");
    c.expect(x.bottomFace.isValid() && x.bottomFace.geometry().kind() == MyBRep::SurfaceKind::BSpline, "B-Spline bottom Face valid");
    c.expect(x.sides.size() == 4, "Four side Faces created");
    bool sidesOk = true;
    for (std::size_t i = 0; i < x.sides.size(); ++i)
        sidesOk = sidesOk && x.sides[i].isValid() && x.sides[i].geometry().kind() == MyBRep::SurfaceKind::Extrusion;
    c.expect(sidesOk, "All side Faces use Extrusion Surface");

    bool shared = true;
    for (std::size_t i = 0; i < 4; ++i)
    {
        const std::size_t previous = (i + 3) % 4;
        shared = shared && contains(x.topFace, x.topEdges[i]) && contains(x.sides[i], x.topEdges[i]);
        shared = shared && contains(x.bottomFace, x.bottomEdges[i]) && contains(x.sides[i], x.bottomEdges[i]);
        shared = shared && contains(x.sides[previous], x.verticalEdges[i]) && contains(x.sides[i], x.verticalEdges[i]);
    }
    c.expect(shared, "All twelve TEdges are shared by adjacent Faces");
    c.expect(x.shell.isValid(), "Mixed Shell valid");
    c.expect(x.shell.faceCount() == 6, "Mixed Shell has six Faces");
    c.expect(x.shell.isClosed(), "Mixed Shell closed");
    c.expect(x.solid.isValid(), "Mixed Topology_Solid valid");

    MyBRep::FaceMeshOptions meshOptions;
    meshOptions.bezier.surfaceChordTolerance = 0.02;
    meshOptions.bspline.surfaceChordTolerance = 0.02;
    meshOptions.extruded.surfaceChordTolerance = 0.02;
    std::vector<MyBRep::Topology_Face> faces;
    faces.push_back(x.topFace);
    faces.push_back(x.bottomFace);
    faces.insert(faces.end(), x.sides.begin(), x.sides.end());
    bool meshesOk = true;
    std::size_t triangles = 0;
    for (std::size_t i = 0; i < faces.size(); ++i)
    {
        const MyBRep::FaceMesh mesh = MyBRep::FaceMesher::mesh(faces[i], meshOptions);
        meshesOk = meshesOk && MyBRep::FaceMesher::canMesh(faces[i]) && mesh.isValid();
        triangles += mesh.triangleCount();
    }
    c.expect(meshesOk, "All six mixed Faces mesh through FaceMesher");
    c.expect(triangles > 20, "Mixed Solid preserves freeform subdivision");

    MyBRep::Display::BRepSolidBuildOptions options;
    options.surface.bezierMeshing.surfaceChordTolerance = 0.02;
    options.surface.bsplineMeshing.surfaceChordTolerance = 0.02;
    options.surface.extrudedMeshing.surfaceChordTolerance = 0.02;
    BufferGeometry* surface = MyBRep::Display::BRepSolidBuilder::buildSurface(x.solid, MyMath::Matrix4::identity(), "MixedSurface", options);
    BufferGeometry* boundary = MyBRep::Display::BRepSolidBuilder::buildBoundary(x.solid, MyMath::Matrix4::identity(), "MixedBoundary", options);
    c.expect(surface != 0, "BRepSolidBuilder creates mixed surface Geometry");
    c.expect(boundary != 0, "BRepSolidBuilder creates mixed boundary Geometry");
    c.expect(surface != 0 && surface->renderType() == RenderType::Triangles && surface->valuesPerVertex() == 6, "Mixed surface Geometry layout valid");
    c.expect(boundary != 0 && boundary->renderType() == RenderType::Lines && boundary->valuesPerVertex() == 3, "Mixed boundary Geometry layout valid");
    delete surface;
    delete boundary;

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << c.passed() << " | Failed: " << c.failed() << std::endl;
    std::cout << "============================================================" << std::endl;
    return c.failed() == 0 ? 0 : 1;
}