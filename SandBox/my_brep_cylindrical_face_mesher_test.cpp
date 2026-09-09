#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "MyMath/CoordinateSystem.h"
#include "MyMath/Vector2.h"
#include "MyMath/Vector3.h"

#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Curve/Geometry_Circle.h"
#include "MyBRep/Geometry/Curve/Geometry_Curve.h"
#include "MyBRep/Geometry/Curve/Geometry_Line.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Curve2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Line2D.h"
#include "MyBRep/Geometry/Surface/Geometry_CylindricalSurface.h"
#include "MyBRep/Geometry/Surface/Geometry_PlaneSurface.h"
#include "MyBRep/Mesh/CylindricalFaceMesher.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Topology/Topology_Builder.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 圆柱测试统一使用的圆周率。
const double TwoPi = Pi * 2.0;                        // 完整圆柱U参数周期。
const double TestTolerance = 1.0e-8;                 // Topology与Curve-on-Surface连接验证统一使用的三维容差。
const double AreaTolerance = 0.6;                    // 离散三角形面积相对解析圆柱面积允许的绝对误差。
const double NormalTolerance = 1.0e-6;               // FaceMesh顶点法向和解析Surface法向比较容差。

class TestContext
{
public:
    TestContext()
        : m_passed(0)
        , m_failed(0)
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

MyBRep::CylindricalFaceMeshOptions testOptions()
{
    MyBRep::CylindricalFaceMeshOptions options;
    options.boundaryChordTolerance = 0.02;
    options.surfaceChordTolerance = 0.02;
    options.geometricTolerance = 1.0e-10;
    options.minimumBoundarySubdivisionDepth = 1;
    options.maximumBoundarySubdivisionDepth = 12;
    options.maximumSurfaceSubdivisionRounds = 12;
    return options;
}

MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>
createCylinderSurface(const MyMath::CoordinateSystem& coordinateSystem,
                      double radius)
{
    return MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>(
        new MyBRep::Geometry_CylindricalSurface(coordinateSystem, radius));
}

void addLineCurveOnSurface(
    MyBRep::Topology_Edge& edge,
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>& surface,
    const MyMath::Vector2& firstUV,
    const MyMath::Vector2& lastUV)
{
    const MyMath::Vector2 direction = lastUV - firstUV;
    const double length = direction.length();

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> curve(
        new MyBRep::Geometry_Line2D(firstUV, direction));

    MyBRep::Topology_Builder::addCurveOnSurface(
        edge,
        surface,
        curve,
        0.0,
        length,
        TestTolerance);
}

MyBRep::Topology_Wire createCylinderPatchWire(
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>& surface,
    double u0,
    double u1,
    double v0,
    double v1)
{
    // 四个角点必须使用共享Topology_Vertex身份，保证Topology_Wire按严格拓扑连接闭合。
    const MyBRep::Geometry_CylindricalSurface& cylinder =
        static_cast<const MyBRep::Geometry_CylindricalSurface&>(*surface);

    const MyBRep::Topology_Vertex v00(cylinder.pointAt(u0, v0));
    const MyBRep::Topology_Vertex v10(cylinder.pointAt(u1, v0));
    const MyBRep::Topology_Vertex v11(cylinder.pointAt(u1, v1));
    const MyBRep::Topology_Vertex v01(cylinder.pointAt(u0, v1));

    const MyMath::Vector3 bottomCenter =
        cylinder.axisOrigin() + cylinder.axisDir() * v0;

    const MyMath::Vector3 topCenter =
        cylinder.axisOrigin() + cylinder.axisDir() * v1;

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> bottomGeometry(
        new MyBRep::Geometry_Circle(
            bottomCenter,
            cylinder.radius(),
            cylinder.xDir(),
            cylinder.yDir()));

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> topGeometry(
        new MyBRep::Geometry_Circle(
            topCenter,
            cylinder.radius(),
            cylinder.xDir(),
            cylinder.yDir()));

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> rightGeometry(
        new MyBRep::Geometry_Line(v10.point(), v11.point() - v10.point()));

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> leftGeometry(
        new MyBRep::Geometry_Line(v00.point(), v01.point() - v00.point()));

    MyBRep::Topology_Edge bottom(
        v00, v10, bottomGeometry, u0, u1, TestTolerance);

    MyBRep::Topology_Edge right(
        v10, v11, rightGeometry, 0.0, (v11.point() - v10.point()).length(), TestTolerance);

    MyBRep::Topology_Edge top(
        v01, v11, topGeometry, u0, u1, TestTolerance);

    MyBRep::Topology_Edge left(
        v00, v01, leftGeometry, 0.0, (v01.point() - v00.point()).length(), TestTolerance);

    addLineCurveOnSurface(
        bottom, surface,
        MyMath::Vector2(u0, v0),
        MyMath::Vector2(u1, v0));

    addLineCurveOnSurface(
        right, surface,
        MyMath::Vector2(u1, v0),
        MyMath::Vector2(u1, v1));

    addLineCurveOnSurface(
        top, surface,
        MyMath::Vector2(u0, v1),
        MyMath::Vector2(u1, v1));

    addLineCurveOnSurface(
        left, surface,
        MyMath::Vector2(u0, v0),
        MyMath::Vector2(u0, v1));

    std::vector<MyBRep::Topology_Edge> edges;
    edges.push_back(bottom);
    edges.push_back(right);
    edges.push_back(top.reversed());
    edges.push_back(left.reversed());

    return MyBRep::Topology_Wire(edges);
}

MyBRep::Topology_Face createCylinderPatchFace(
    const MyMath::CoordinateSystem& coordinateSystem,
    double radius,
    double u0,
    double u1,
    double v0,
    double v1)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface =
        createCylinderSurface(coordinateSystem, radius);

    return MyBRep::Modeling::createFace(
        surface,
        std::vector<MyBRep::Topology_Wire>(
            1,
            createCylinderPatchWire(surface, u0, u1, v0, v1)));
}

MyBRep::Topology_Face createCylinderFaceWithHole(
    const MyMath::CoordinateSystem& coordinateSystem,
    double radius)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface =
        createCylinderSurface(coordinateSystem, radius);

    std::vector<MyBRep::Topology_Wire> wires;

    wires.push_back(
        createCylinderPatchWire(surface,
                                0.0, 2.0,
                                -2.0, 2.0));

    wires.push_back(
        createCylinderPatchWire(surface,
                                0.7, 1.3,
                                -0.8, 0.8));

    return MyBRep::Modeling::createFace(surface, wires);
}

MyBRep::Topology_Face createFullCylinderBandFace(
    const MyMath::CoordinateSystem& coordinateSystem,
    double radius,
    double v0,
    double v1)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface =
        createCylinderSurface(coordinateSystem, radius);

    const MyBRep::Geometry_CylindricalSurface& cylinder =
        static_cast<const MyBRep::Geometry_CylindricalSurface&>(*surface);

    const MyBRep::Topology_Vertex bottomVertex(
        cylinder.pointAt(0.0, v0));

    const MyBRep::Topology_Vertex topVertex(
        cylinder.pointAt(0.0, v1));

    const MyMath::Vector3 bottomCenter =
        cylinder.axisOrigin() + cylinder.axisDir() * v0;

    const MyMath::Vector3 topCenter =
        cylinder.axisOrigin() + cylinder.axisDir() * v1;

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> bottomGeometry(
        new MyBRep::Geometry_Circle(
            bottomCenter,
            cylinder.radius(),
            cylinder.xDir(),
            cylinder.yDir()));

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> topGeometry(
        new MyBRep::Geometry_Circle(
            topCenter,
            cylinder.radius(),
            cylinder.xDir(),
            cylinder.yDir()));

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> seamGeometry(
        new MyBRep::Geometry_Line(
            bottomVertex.point(),
            topVertex.point() - bottomVertex.point()));

    MyBRep::Topology_Edge bottom(
        bottomVertex,
        bottomVertex,
        bottomGeometry,
        0.0,
        TwoPi,
        TestTolerance);

    MyBRep::Topology_Edge top(
        topVertex,
        topVertex,
        topGeometry,
        0.0,
        TwoPi,
        TestTolerance);

    MyBRep::Topology_Edge seam(
        bottomVertex,
        topVertex,
        seamGeometry,
        0.0,
        (topVertex.point() - bottomVertex.point()).length(),
        TestTolerance);

    addLineCurveOnSurface(
        bottom,
        surface,
        MyMath::Vector2(0.0, v0),
        MyMath::Vector2(TwoPi, v0));

    addLineCurveOnSurface(
        top,
        surface,
        MyMath::Vector2(0.0, v1),
        MyMath::Vector2(TwoPi, v1));

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> firstSeamCurve(
        new MyBRep::Geometry_Line2D(
            MyMath::Vector2(0.0, v0),
            MyMath::Vector2(0.0, 1.0)));

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> secondSeamCurve(
        new MyBRep::Geometry_Line2D(
            MyMath::Vector2(TwoPi, v0),
            MyMath::Vector2(0.0, 1.0)));

    const double height = v1 - v0;

    MyBRep::Topology_Builder::addCurveOnClosedSurface(
        seam,
        surface,
        firstSeamCurve,
        0.0,
        height,
        secondSeamCurve,
        0.0,
        height,
        TestTolerance);

    std::vector<MyBRep::Topology_Edge> edges;

    // Bottom reversed使用2π→0；随后Forward seam使用U=0；
    // Top Forward使用0→2π；最后Reversed seam自动选择第二条U=2π P-Curve并返回到底部。
    edges.push_back(bottom.reversed());
    edges.push_back(seam);
    edges.push_back(top);
    edges.push_back(seam.reversed());

    const MyBRep::Topology_Wire wire(edges);

    return MyBRep::Modeling::createFace(
        surface,
        std::vector<MyBRep::Topology_Wire>(1, wire));
}

double meshArea(const MyBRep::FaceMesh& mesh)
{
    double area = 0.0;

    for (std::size_t index = 0;
         index < mesh.indices().size();
         index += 3)
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

bool triangleWindingMatchesNormals(const MyBRep::FaceMesh& mesh)
{
    if (!mesh.isValid())
    {
        return false;
    }

    for (std::size_t index = 0;
         index < mesh.indices().size();
         index += 3)
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

bool vertexNormalsMatchFace(const MyBRep::Topology_Face& face,
                            const MyBRep::FaceMesh& mesh)
{
    for (std::size_t index = 0; index < mesh.vertices().size(); ++index)
    {
        const MyBRep::FaceMeshVertex& vertex =
            mesh.vertices()[index];

        const MyMath::Vector3 expected =
            face.normalAt(vertex.parameter.x(),
                          vertex.parameter.y());

        if (!vertex.normal.isEqualTo(expected,
                                     NormalTolerance))
        {
            return false;
        }
    }

    return true;
}

void testDefaultOptions(TestContext& context)
{
    const MyBRep::CylindricalFaceMeshOptions options;
    context.expect(options.isValid(), "Default cylindrical mesh options valid");
}

void testQuarterCylinder(TestContext& context)
{
    const double radius = 5.0;
    const double u0 = 0.0;
    const double u1 = Pi * 0.5;
    const double v0 = -2.0;
    const double v1 = 2.0;

    const MyMath::CoordinateSystem coordinateSystem =
        MyMath::CoordinateSystem::identity();

    const MyBRep::Topology_Face face =
        createCylinderPatchFace(
            coordinateSystem,
            radius,
            u0, u1,
            v0, v1);

    const MyBRep::FaceMesh mesh =
        MyBRep::CylindricalFaceMesher::mesh(
            face,
            testOptions());

    const double expectedArea =
        radius * (u1 - u0) * (v1 - v0);

    context.expect(MyBRep::CylindricalFaceMesher::canMesh(face),
                   "Quarter cylinder Face can mesh");

    context.expect(mesh.isValid(),
                   "Quarter cylinder mesh valid");

    context.expect(mesh.triangleCount() > 2,
                   "Quarter cylinder surface receives curvature subdivision");

    context.expect(std::fabs(meshArea(mesh) - expectedArea) <= AreaTolerance,
                   "Quarter cylinder mesh area");

    context.expect(triangleWindingMatchesNormals(mesh),
                   "Quarter cylinder triangle winding matches normals");

    context.expect(vertexNormalsMatchFace(face, mesh),
                   "Quarter cylinder vertex normals match Face normals");
}

void testCylinderHole(TestContext& context)
{
    const double radius = 4.0;

    const MyBRep::Topology_Face face =
        createCylinderFaceWithHole(
            MyMath::CoordinateSystem::identity(),
            radius);

    const MyBRep::FaceMesh mesh =
        MyBRep::CylindricalFaceMesher::mesh(
            face,
            testOptions());

    const double outerUVArea =
        (2.0 - 0.0) * (2.0 - (-2.0));

    const double holeUVArea =
        (1.3 - 0.7) * (0.8 - (-0.8));

    const double expectedArea =
        radius * (outerUVArea - holeUVArea);

    context.expect(mesh.isValid(),
                   "Cylinder Face with hole mesh valid");

    context.expect(std::fabs(meshArea(mesh) - expectedArea) <= AreaTolerance,
                   "Cylinder Face with hole area");

    context.expect(triangleWindingMatchesNormals(mesh),
                   "Cylinder Face with hole winding");
}

void testSeamCrossingPatch(TestContext& context)
{
    const double radius = 3.5;
    const double u0 = 5.7;
    const double u1 = 6.7;

    const MyBRep::Topology_Face face =
        createCylinderPatchFace(
            MyMath::CoordinateSystem::identity(),
            radius,
            u0,
            u1,
            -1.5,
            1.5);

    const MyBRep::FaceMesh mesh =
        MyBRep::CylindricalFaceMesher::mesh(
            face,
            testOptions());

    bool containsBeyondTwoPi = false;

    for (std::size_t index = 0;
         index < mesh.vertices().size();
         ++index)
    {
        if (mesh.vertices()[index].parameter.x() > TwoPi)
        {
            containsBeyondTwoPi = true;
            break;
        }
    }

    context.expect(mesh.isValid(),
                   "Cylinder seam-crossing patch mesh valid");

    context.expect(containsBeyondTwoPi,
                   "Cylinder seam-crossing patch preserves continuous unwrapped U");

    context.expect(triangleWindingMatchesNormals(mesh),
                   "Cylinder seam-crossing patch winding");
}

void testFullCylinderBand(TestContext& context)
{
    const double radius = 3.0;
    const double v0 = -2.0;
    const double v1 = 2.0;

    const MyBRep::Topology_Face face =
        createFullCylinderBandFace(
            MyMath::CoordinateSystem::identity(),
            radius,
            v0,
            v1);

    MyBRep::CylindricalFaceMeshOptions options =
        testOptions();

    options.boundaryChordTolerance = 0.05;
    options.surfaceChordTolerance = 0.05;

    const MyBRep::FaceMesh mesh =
        MyBRep::CylindricalFaceMesher::mesh(
            face,
            options);

    const double expectedArea =
        TwoPi * radius * (v1 - v0);

    double minimumU = 0.0;
    double maximumU = 0.0;

    if (!mesh.vertices().empty())
    {
        minimumU = mesh.vertices()[0].parameter.x();
        maximumU = mesh.vertices()[0].parameter.x();

        for (std::size_t index = 1;
             index < mesh.vertices().size();
             ++index)
        {
            minimumU =
                (std::min)(minimumU,
                           mesh.vertices()[index].parameter.x());

            maximumU =
                (std::max)(maximumU,
                           mesh.vertices()[index].parameter.x());
        }
    }

    context.expect(MyBRep::CylindricalFaceMesher::canMesh(face),
                   "Full cylinder seam Face can mesh");

    context.expect(mesh.isValid(),
                   "Full cylinder band mesh valid");

    context.expect(maximumU - minimumU >= TwoPi - 1.0e-8,
                   "Full cylinder band keeps one complete U period");

    context.expect(std::fabs(meshArea(mesh) - expectedArea) <= AreaTolerance,
                   "Full cylinder band mesh area");

    context.expect(triangleWindingMatchesNormals(mesh),
                   "Full cylinder band winding");
}

void testSurfaceToleranceRefinement(TestContext& context)
{
    const MyBRep::Topology_Face face =
        createCylinderPatchFace(
            MyMath::CoordinateSystem::identity(),
            6.0,
            0.0,
            Pi,
            -2.0,
            2.0);

    MyBRep::CylindricalFaceMeshOptions loose =
        testOptions();

    MyBRep::CylindricalFaceMeshOptions tight =
        testOptions();

    loose.boundaryChordTolerance = 0.05;
    tight.boundaryChordTolerance = 0.05;

    loose.surfaceChordTolerance = 0.20;
    tight.surfaceChordTolerance = 0.01;

    const MyBRep::FaceMesh looseMesh =
        MyBRep::CylindricalFaceMesher::mesh(face, loose);

    const MyBRep::FaceMesh tightMesh =
        MyBRep::CylindricalFaceMesher::mesh(face, tight);

    context.expect(looseMesh.isValid() && tightMesh.isValid(),
                   "Loose and tight cylinder meshes valid");

    context.expect(tightMesh.triangleCount() > looseMesh.triangleCount(),
                   "Tighter cylinder surface tolerance increases triangle count");
}

void testReversedFace(TestContext& context)
{
    const MyBRep::Topology_Face forwardFace =
        createCylinderPatchFace(
            MyMath::CoordinateSystem::identity(),
            5.0,
            0.0,
            Pi * 0.75,
            -2.0,
            2.0);

    const MyBRep::Topology_Face reversedFace =
        forwardFace.reversed();

    const MyBRep::FaceMesh forwardMesh =
        MyBRep::CylindricalFaceMesher::mesh(
            forwardFace,
            testOptions());

    const MyBRep::FaceMesh reversedMesh =
        MyBRep::CylindricalFaceMesher::mesh(
            reversedFace,
            testOptions());

    context.expect(forwardMesh.isValid() && reversedMesh.isValid(),
                   "Forward and Reversed cylinder Face meshes valid");

    context.expect(std::fabs(meshArea(forwardMesh) -
                             meshArea(reversedMesh)) <= 1.0e-8,
                   "Reversed cylinder Face preserves mesh area");

    bool normalsOpposite = false;

    if (!forwardMesh.vertices().empty() &&
        !reversedMesh.vertices().empty())
    {
        normalsOpposite =
            MyMath::Vector3::dot(
                forwardMesh.vertices()[0].normal,
                reversedMesh.vertices()[0].normal) < -0.999999;
    }

    context.expect(normalsOpposite,
                   "Reversed cylinder Face normals are opposite");

    context.expect(triangleWindingMatchesNormals(reversedMesh),
                   "Reversed cylinder Face winding follows reversed normal");
}

void testOrientedCylinder(TestContext& context)
{
    const MyMath::CoordinateSystem coordinateSystem =
        MyMath::CoordinateSystem::fromAxes(
            MyMath::Vector3(3.0, 4.0, 5.0),
            MyMath::Vector3::unitY(),
            MyMath::Vector3::unitZ(),
            MyMath::Vector3::unitX());

    const MyBRep::Topology_Face face =
        createCylinderPatchFace(
            coordinateSystem,
            4.0,
            0.2,
            1.4,
            -1.0,
            3.0);

    const MyBRep::FaceMesh mesh =
        MyBRep::CylindricalFaceMesher::mesh(
            face,
            testOptions());

    context.expect(mesh.isValid(),
                   "Oriented cylinder Face mesh valid");

    context.expect(vertexNormalsMatchFace(face, mesh),
                   "Oriented cylinder normals match Face");

    context.expect(triangleWindingMatchesNormals(mesh),
                   "Oriented cylinder winding matches normal");
}

void testUnsupportedPlane(TestContext& context)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> surface(
        new MyBRep::Geometry_PlaneSurface(
            MyMath::CoordinateSystem::identity()));

    const MyBRep::Topology_Face face(surface);

    context.expect(!MyBRep::CylindricalFaceMesher::canMesh(face),
                   "Plane Face cannot use cylindrical mesher");

    context.expect(MyBRep::CylindricalFaceMesher::mesh(face).isEmpty(),
                   "Plane Face returns empty cylindrical mesh");
}

}

int main()
{
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep Cylindrical Face Mesher Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testDefaultOptions(context);
    testQuarterCylinder(context);
    testCylinderHole(context);
    testSeamCrossingPatch(context);
    testFullCylinderBand(context);
    testSurfaceToleranceRefinement(context);
    testReversedFace(context);
    testOrientedCylinder(context);
    testUnsupportedPlane(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed()
              << " | Failed: " << context.failed()
              << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}
