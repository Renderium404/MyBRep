#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Modeling/Wire/WireModeling.h"
#include "MyBRepOpenGL/Builder/BRepWireframeBuilder.h"
#include "MyBRepOpenGL/Display/BRepDisplayObject.h"
#include "MyBRepOpenGL/Display/BRepDisplayStyle.h"
#include "MyOpenGL/Resource/BufferGeometry.h"

namespace
{

class TestContext
{
public:
    TestContext() : m_passed(0), m_failed(0) {}

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

    int passed() const { return m_passed; }
    int failed() const { return m_failed; }

private:
    int m_passed;
    int m_failed;
};

bool nearEqual(double first, double second, double tolerance = 1.0e-5)
{
    return std::fabs(first - second) <= tolerance;
}

void testDefaultObjects(TestContext& context)
{
    const MyBRep::Display::BRepWireframeBuildOptions options;
    const MyBRep::Display::BRepDisplayStyle style;
    const MyBRep::Display::BRepDisplayObject object;

    context.expect(options.isValid(), "Default wireframe options valid");
    context.expect(style.isValid(), "Default display style valid");
    context.expect(!object.isValid(), "Default display object invalid");
}

void testRectangle(TestContext& context)
{
    const MyBRep::Topology_Wire rectangle = MyBRep::Modeling::createRectangle(10.0, 8.0);
    BufferGeometry* geometry = MyBRep::Display::BRepWireframeBuilder::build(rectangle, "RectangleWireframe");

    context.expect(geometry != 0, "Rectangle wireframe Geometry created");
    context.expect(geometry != 0 && geometry->renderType() == RenderType::Lines, "Rectangle uses Lines render type");
    context.expect(geometry != 0 && geometry->valuesPerVertex() == 3, "Rectangle uses position-only vertex layout");
    context.expect(geometry != 0 && geometry->hasAttribute(GeometryAttribute::Position, 3), "Rectangle exposes Position attribute");
    context.expect(geometry != 0 && geometry->indexCount() == 8, "Rectangle four linear Edges remain four line segments");
    context.expect(geometry != 0 && nearEqual(geometry->lineWidth(), 1.5), "Rectangle default line width");

    delete geometry;
}

void testCircleAdaptiveSubdivision(TestContext& context)
{
    const MyBRep::Topology_Wire circle = MyBRep::Modeling::createCircle(5.0);
    MyBRep::Display::BRepWireframeBuildOptions options;
    options.chordTolerance = 1.0e-2;

    BufferGeometry* geometry = MyBRep::Display::BRepWireframeBuilder::build(circle, "CircleWireframe", options);

    context.expect(geometry != 0, "Circle wireframe Geometry created");
    context.expect(geometry != 0 && geometry->indexCount() >= 32, "Circle adaptively subdivides curved Edge");
    context.expect(geometry != 0 && geometry->indexCount() % 2 == 0, "Circle Lines index count remains even");

    delete geometry;
}

void testFaceBoundary(TestContext& context)
{
    std::vector<MyBRep::Topology_Wire> wires;
    wires.push_back(MyBRep::Modeling::createRectangle(10.0, 8.0));
    wires.push_back(MyBRep::Modeling::createCircle(2.0));

    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(wires, 1.0e-8);
    BufferGeometry* geometry = MyBRep::Display::BRepWireframeBuilder::build(face, "FaceBoundary");

    context.expect(geometry != 0, "Face boundary Geometry created");
    context.expect(geometry != 0 && geometry->indexCount() > 8, "Face includes all trimming Wire boundaries");

    delete geometry;
}

void testAffinePlacementBakedIntoVertices(TestContext& context)
{
    const MyBRep::Topology_Wire rectangle = MyBRep::Modeling::createRectangle(10.0, 8.0);
    const MyMath::Matrix4 transform = MyMath::Matrix4::fromTranslation(MyMath::Vector3(10.0, 20.0, 30.0));
    BufferGeometry* geometry = MyBRep::Display::BRepWireframeBuilder::build(rectangle, transform, "TranslatedRectangle");

    context.expect(geometry != 0 && geometry->vertexData().size() >= 3, "Affine placement Geometry created");

    const bool placementCorrect = geometry != 0 && geometry->vertexData().size() >= 3 &&
                                  nearEqual(geometry->vertexData()[0], 5.0) && nearEqual(geometry->vertexData()[1], 16.0) && nearEqual(geometry->vertexData()[2], 30.0);

    context.expect(placementCorrect, "Affine placement is baked into generated vertices");

    delete geometry;
}

void testGeneralAffinePlacement(TestContext& context)
{
    const MyBRep::Topology_Wire rectangle = MyBRep::Modeling::createRectangle(2.0, 2.0);
    MyMath::Matrix4 transform = MyMath::Matrix4::identity();

    transform(0, 1) = 0.5; // XY剪切，用于验证显示层不依赖MyOpenGL仅支持TRS的Transform。
    transform(0, 3) = 3.0;

    BufferGeometry* geometry = MyBRep::Display::BRepWireframeBuilder::build(rectangle, transform, "ShearedRectangle");

    context.expect(geometry != 0, "General affine placement Geometry created");

    const bool transformed = geometry != 0 && geometry->vertexData().size() >= 3 &&
                             nearEqual(geometry->vertexData()[0], 1.5) && nearEqual(geometry->vertexData()[1], -1.0) && nearEqual(geometry->vertexData()[2], 0.0);

    context.expect(transformed, "General affine shear is baked into vertices");

    delete geometry;
}

void testReversedWire(TestContext& context)
{
    const MyBRep::Topology_Wire forward = MyBRep::Modeling::createRectangle(10.0, 8.0);
    const MyBRep::Topology_Wire reversed = forward.reversed();

    BufferGeometry* forwardGeometry = MyBRep::Display::BRepWireframeBuilder::build(forward, "ForwardRectangle");
    BufferGeometry* reversedGeometry = MyBRep::Display::BRepWireframeBuilder::build(reversed, "ReversedRectangle");

    context.expect(forwardGeometry != 0 && reversedGeometry != 0, "Forward and Reversed Wire Geometry created");
    context.expect(forwardGeometry != 0 && reversedGeometry != 0 && forwardGeometry->indexCount() == reversedGeometry->indexCount(),
                   "Reversed Wire preserves wireframe segment count");

    delete forwardGeometry;
    delete reversedGeometry;
}

void testCustomStyle(TestContext& context)
{
    MyBRep::Display::BRepDisplayStyle style;
    style.wireColor = QVector4D(0.2f, 0.3f, 0.4f, 0.8f);
    style.wireframe.lineWidth = 2.0f;
    style.wireframe.chordTolerance = 5.0e-3;

    context.expect(style.isValid(), "Custom display style valid");

    const MyBRep::Topology_Wire rectangle = MyBRep::Modeling::createRectangle(4.0, 3.0);
    BufferGeometry* geometry = MyBRep::Display::BRepWireframeBuilder::build(rectangle, "StyledRectangle", style.wireframe);

    context.expect(geometry != 0 && nearEqual(geometry->lineWidth(), 2.0), "Custom line width reaches Geometry");

    delete geometry;
}

}

int main()
{
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep MyOpenGL Wireframe Builder Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testDefaultObjects(context);
    testRectangle(context);
    testCircleAdaptiveSubdivision(context);
    testFaceBoundary(context);
    testAffinePlacementBakedIntoVertices(context);
    testGeneralAffinePlacement(context);
    testReversedWire(context);
    testCustomStyle(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}
