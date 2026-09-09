#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"

#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Modeling/Wire/WireModeling.h"

#include "MyBRepOpenGL/Builder/BRepFaceBuilder.h"
#include "MyOpenGL/Resource/BufferGeometry.h"

namespace
{

const double TestTolerance = 1.0e-8; // Face构造与测试数值比较统一使用的几何容差。
const double ValueTolerance = 1.0e-5; // double转换为GLfloat后的显示数据比较容差。

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

bool nearEqual(double first, double second, double tolerance = ValueTolerance)
{
    return std::fabs(first - second) <= tolerance;
}

MyMath::Vector3 geometryPosition(const BufferGeometry& geometry, unsigned int vertexIndex)
{
    const std::size_t offset = static_cast<std::size_t>(vertexIndex) * 6;
    const std::vector<GLfloat>& data = geometry.vertexData();

    return MyMath::Vector3(data[offset], data[offset + 1], data[offset + 2]);
}

MyMath::Vector3 geometryNormal(const BufferGeometry& geometry, unsigned int vertexIndex)
{
    const std::size_t offset = static_cast<std::size_t>(vertexIndex) * 6 + 3;
    const std::vector<GLfloat>& data = geometry.vertexData();

    return MyMath::Vector3(data[offset], data[offset + 1], data[offset + 2]);
}

bool firstTriangleWindingMatchesStoredNormal(const BufferGeometry& geometry)
{
    if (geometry.indexData().size() < 3)
    {
        return false;
    }

    const unsigned int firstIndex = geometry.indexData()[0];
    const unsigned int secondIndex = geometry.indexData()[1];
    const unsigned int thirdIndex = geometry.indexData()[2];

    const MyMath::Vector3 first = geometryPosition(geometry, firstIndex);
    const MyMath::Vector3 second = geometryPosition(geometry, secondIndex);
    const MyMath::Vector3 third = geometryPosition(geometry, thirdIndex);
    const MyMath::Vector3 normal = geometryNormal(geometry, firstIndex);
    const MyMath::Vector3 triangleNormal = MyMath::Vector3::cross(second - first, third - first);

    return MyMath::Vector3::dot(triangleNormal, normal) > 0.0;
}

void testDefaultOptions(TestContext& context)
{
    const MyBRep::Display::BRepFaceBuildOptions options;
    context.expect(options.isValid(), "Default Face build options valid");
}

void testRectangle(TestContext& context)
{
    const MyBRep::Topology_Face face =
        MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createRectangle(10.0, 8.0), TestTolerance);

    BufferGeometry* geometry = MyBRep::Display::BRepFaceBuilder::build(face, "RectangleSurface");

    context.expect(geometry != 0, "Rectangle surface Geometry created");
    context.expect(geometry != 0 && geometry->renderType() == RenderType::Triangles, "Rectangle surface uses Triangles render type");
    context.expect(geometry != 0 && geometry->valuesPerVertex() == 6, "Rectangle surface uses Position Normal interleaved layout");
    context.expect(geometry != 0 && geometry->hasAttribute(GeometryAttribute::Position, 3), "Rectangle surface exposes Position attribute");
    context.expect(geometry != 0 && geometry->hasAttribute(GeometryAttribute::Normal, 3), "Rectangle surface exposes Normal attribute");
    context.expect(geometry != 0 && geometry->vertexCount() == 4, "Rectangle surface keeps four mesh vertices");
    context.expect(geometry != 0 && geometry->indexCount() == 6, "Rectangle surface contains two triangles");
    context.expect(geometry != 0 && firstTriangleWindingMatchesStoredNormal(*geometry), "Rectangle triangle winding matches stored normal");

    delete geometry;
}

void testFaceWithHole(TestContext& context)
{
    std::vector<MyBRep::Topology_Wire> wires;
    wires.push_back(MyBRep::Modeling::createRectangle(10.0, 8.0));
    wires.push_back(MyBRep::Modeling::createCircle(2.0));

    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(wires, TestTolerance);

    MyBRep::Display::BRepFaceBuildOptions options;
    options.meshing.chordTolerance = 0.01;

    BufferGeometry* geometry = MyBRep::Display::BRepFaceBuilder::build(face, "FaceWithHoleSurface", options);

    context.expect(geometry != 0, "Face with hole surface Geometry created");
    context.expect(geometry != 0 && geometry->indexCount() > 6, "Face with hole contains triangulated curved boundary");
    context.expect(geometry != 0 && firstTriangleWindingMatchesStoredNormal(*geometry), "Face with hole triangle winding matches stored normal");

    delete geometry;
}

void testTranslationBakedIntoVertices(TestContext& context)
{
    const MyBRep::Topology_Face face =
        MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createRectangle(10.0, 8.0), TestTolerance);

    const MyMath::Vector3 translation(10.0, 20.0, 30.0); // 固定平移量，用于逐顶点验证Face显示Geometry的仿射位置烘焙。
    const MyMath::Matrix4 transform = MyMath::Matrix4::fromTranslation(translation);

    BufferGeometry* localGeometry = MyBRep::Display::BRepFaceBuilder::build(face, "LocalFace");
    BufferGeometry* translatedGeometry = MyBRep::Display::BRepFaceBuilder::build(face, transform, "TranslatedFace");

    const bool created =
        localGeometry != 0 &&
        translatedGeometry != 0 &&
        localGeometry->vertexCount() > 0 &&
        localGeometry->vertexCount() == translatedGeometry->vertexCount();

    context.expect(created, "Translated Face surface Geometry created");

    bool translated = created;

    if (created)
    {
        for (int index = 0; index < localGeometry->vertexCount(); ++index)
        {
            const MyMath::Vector3 localPosition = geometryPosition(*localGeometry, static_cast<unsigned int>(index));
            const MyMath::Vector3 translatedPosition = geometryPosition(*translatedGeometry, static_cast<unsigned int>(index));
            const MyMath::Vector3 expectedPosition = localPosition + translation;

            if (!translatedPosition.isEqualTo(expectedPosition, ValueTolerance))
            {
                translated = false;
                break;
            }
        }
    }

    context.expect(translated, "Face translation is baked into generated vertices");

    delete localGeometry;
    delete translatedGeometry;
}

void testGeneralAffineNormalTransform(TestContext& context)
{
    const MyBRep::Topology_Face face =
        MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createRectangle(4.0, 4.0), TestTolerance);

    MyMath::Matrix4 transform = MyMath::Matrix4::identity();
    transform(2, 0) = 0.5; // 将局部XY平面剪切为z=0.5x倾斜平面，用于验证逆转置法向变换。

    BufferGeometry* geometry = MyBRep::Display::BRepFaceBuilder::build(face, transform, "ShearedFace");

    context.expect(geometry != 0, "General affine Face surface Geometry created");

    bool normalCorrect = false;

    if (geometry != 0 && geometry->vertexCount() > 0)
    {
        const MyMath::Vector3 normal = geometryNormal(*geometry, 0);
        const MyMath::Vector3 expected(-0.5, 0.0, 1.0);
        const MyMath::Vector3 expectedUnit = expected.normalized(0.0);

        normalCorrect = normal.isEqualTo(expectedUnit, ValueTolerance);
    }

    context.expect(normalCorrect, "General affine Face uses inverse transpose normal transform");
    context.expect(geometry != 0 && firstTriangleWindingMatchesStoredNormal(*geometry), "General affine triangle winding matches transformed normal");

    delete geometry;
}

void testOrientationReversingAffine(TestContext& context)
{
    const MyBRep::Topology_Face face =
        MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createRectangle(4.0, 4.0), TestTolerance);

    const MyMath::Matrix4 transform = MyMath::Matrix4::fromScale(MyMath::Vector3(-1.0, 1.0, 1.0));
    BufferGeometry* geometry = MyBRep::Display::BRepFaceBuilder::build(face, transform, "MirroredFace");

    context.expect(geometry != 0, "Orientation reversing affine Face Geometry created");

    bool normalReversed = false;

    if (geometry != 0 && geometry->vertexCount() > 0)
    {
        const MyMath::Vector3 normal = geometryNormal(*geometry, 0);
        normalReversed = nearEqual(normal.x(), 0.0) && nearEqual(normal.y(), 0.0) && nearEqual(normal.z(), -1.0);
    }

    context.expect(normalReversed, "Orientation reversing affine also reverses Surface normal");
    context.expect(geometry != 0 && firstTriangleWindingMatchesStoredNormal(*geometry), "Mirrored triangle winding matches stored normal");

    delete geometry;
}

void testReversedFace(TestContext& context)
{
    const MyBRep::Topology_Face face =
        MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createRectangle(4.0, 4.0), TestTolerance).reversed();

    BufferGeometry* geometry = MyBRep::Display::BRepFaceBuilder::build(face, "ReversedFace");

    context.expect(geometry != 0, "Reversed Face surface Geometry created");

    bool normalReversed = false;

    if (geometry != 0 && geometry->vertexCount() > 0)
    {
        const MyMath::Vector3 normal = geometryNormal(*geometry, 0);
        normalReversed = nearEqual(normal.x(), 0.0) && nearEqual(normal.y(), 0.0) && nearEqual(normal.z(), -1.0);
    }

    context.expect(normalReversed, "Reversed Face stores reversed normal");
    context.expect(geometry != 0 && firstTriangleWindingMatchesStoredNormal(*geometry), "Reversed Face triangle winding matches stored normal");

    delete geometry;
}

void testUntrimmedPlaneRejected(TestContext& context)
{
    const MyBRep::Topology_Face trimmed =
        MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createRectangle(4.0, 4.0), TestTolerance);
    const MyBRep::Topology_Face untrimmed(trimmed.geometryResource());

    BufferGeometry* geometry = MyBRep::Display::BRepFaceBuilder::build(untrimmed, "UntrimmedPlane");

    context.expect(geometry == 0, "Untrimmed infinite Plane surface is rejected");

    delete geometry;
}

}

int main()
{
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep MyOpenGL Face Builder Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testDefaultOptions(context);
    testRectangle(context);
    testFaceWithHole(context);
    testTranslationBakedIntoVertices(context);
    testGeneralAffineNormalTransform(context);
    testOrientationReversingAffine(context);
    testReversedFace(context);
    testUntrimmedPlaneRejected(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}
