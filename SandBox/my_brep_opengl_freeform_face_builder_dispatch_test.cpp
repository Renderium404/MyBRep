#include <iostream>
#include <string>

#include "FreeformSurfaceTestFixtures.h"

#include "MyBRepOpenGL/Builder/BRepFaceBuilder.h"
#include "MyOpenGL/Resource/BufferGeometry.h"

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

void testBezierBuilder(TestContext& context)
{
    const MyBRep::Topology_Face face = FreeformSurfaceTestFixtures::createBezierFace();

    MyBRep::Display::BRepFaceBuildOptions options;
    options.bezierMeshing.boundaryChordTolerance = 0.02;
    options.bezierMeshing.surfaceChordTolerance = 0.02;
    options.bezierMeshing.minimumBoundarySubdivisionDepth = 1;

    BufferGeometry* geometry =
        MyBRep::Display::BRepFaceBuilder::build(face, "BezierFreeformFace", options);

    context.expect(geometry != 0, "BRepFaceBuilder creates Bezier Face Geometry");
    context.expect(geometry != 0 && geometry->renderType() == RenderType::Triangles, "Bezier Geometry uses Triangles");
    context.expect(geometry != 0 && geometry->valuesPerVertex() == 6, "Bezier Geometry uses Position Normal layout");
    context.expect(geometry != 0 && geometry->hasAttribute(GeometryAttribute::Position, 3), "Bezier Geometry exposes Position");
    context.expect(geometry != 0 && geometry->hasAttribute(GeometryAttribute::Normal, 3), "Bezier Geometry exposes Normal");
    context.expect(geometry != 0 && geometry->indexCount() > 6, "Bezier Geometry preserves interior curvature subdivision");

    delete geometry;
}

void testBezierTolerance(TestContext& context)
{
    const MyBRep::Topology_Face face = FreeformSurfaceTestFixtures::createBezierFace();
    MyBRep::Display::BRepFaceBuildOptions loose;
    MyBRep::Display::BRepFaceBuildOptions tight;

    loose.bezierMeshing.surfaceChordTolerance = 0.20;
    tight.bezierMeshing.surfaceChordTolerance = 0.01;

    BufferGeometry* looseGeometry = MyBRep::Display::BRepFaceBuilder::build(face, "LooseBezier", loose);
    BufferGeometry* tightGeometry = MyBRep::Display::BRepFaceBuilder::build(face, "TightBezier", tight);

    context.expect(looseGeometry != 0 && tightGeometry != 0, "BRepFaceBuilder creates loose and tight Bezier Geometry");
    context.expect(looseGeometry != 0 && tightGeometry != 0 && tightGeometry->indexCount() > looseGeometry->indexCount(),
                   "BRepFaceBuilder forwards Bezier surface tolerance");

    delete looseGeometry;
    delete tightGeometry;
}

void testBSplineBuilder(TestContext& context)
{
    const MyBRep::Topology_Face face = FreeformSurfaceTestFixtures::createBSplineFace();

    MyBRep::Display::BRepFaceBuildOptions options;
    options.bsplineMeshing.boundaryChordTolerance = 0.02;
    options.bsplineMeshing.surfaceChordTolerance = 0.02;
    options.bsplineMeshing.minimumBoundarySubdivisionDepth = 1;

    BufferGeometry* geometry =
        MyBRep::Display::BRepFaceBuilder::build(face, "BSplineFreeformFace", options);

    context.expect(geometry != 0, "BRepFaceBuilder creates B-Spline Face Geometry");
    context.expect(geometry != 0 && geometry->renderType() == RenderType::Triangles, "B-Spline Geometry uses Triangles");
    context.expect(geometry != 0 && geometry->valuesPerVertex() == 6, "B-Spline Geometry uses Position Normal layout");
    context.expect(geometry != 0 && geometry->hasAttribute(GeometryAttribute::Position, 3), "B-Spline Geometry exposes Position");
    context.expect(geometry != 0 && geometry->hasAttribute(GeometryAttribute::Normal, 3), "B-Spline Geometry exposes Normal");
    context.expect(geometry != 0 && geometry->indexCount() > 6, "B-Spline Geometry preserves multi-span curvature subdivision");

    delete geometry;
}

void testBSplineTolerance(TestContext& context)
{
    const MyBRep::Topology_Face face = FreeformSurfaceTestFixtures::createBSplineFace();
    MyBRep::Display::BRepFaceBuildOptions loose;
    MyBRep::Display::BRepFaceBuildOptions tight;

    loose.bsplineMeshing.surfaceChordTolerance = 0.20;
    tight.bsplineMeshing.surfaceChordTolerance = 0.01;

    BufferGeometry* looseGeometry = MyBRep::Display::BRepFaceBuilder::build(face, "LooseBSpline", loose);
    BufferGeometry* tightGeometry = MyBRep::Display::BRepFaceBuilder::build(face, "TightBSpline", tight);

    context.expect(looseGeometry != 0 && tightGeometry != 0, "BRepFaceBuilder creates loose and tight B-Spline Geometry");
    context.expect(looseGeometry != 0 && tightGeometry != 0 && tightGeometry->indexCount() > looseGeometry->indexCount(),
                   "BRepFaceBuilder forwards B-Spline surface tolerance");

    delete looseGeometry;
    delete tightGeometry;
}

}

int main()
{
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep OpenGL Freeform Face Builder Dispatch Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    context.expect(MyBRep::Display::BRepFaceBuildOptions().isValid(), "Default eight-surface BRepFaceBuildOptions valid");
    testBezierBuilder(context);
    testBezierTolerance(context);
    testBSplineBuilder(context);
    testBSplineTolerance(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}