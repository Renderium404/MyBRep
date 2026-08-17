#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "MyMath/Vector2.h"
#include "MyMath/Vector3.h"
#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Curve2D/Geometry_BSpline2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Bezier2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Line2D.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Modeling/Wire/WireModeling.h"
#include "MyBRep/Operation/Intersection/WireCurve2DIntersection.h"
#include "MyBRep/Operation/Split/WireSplit.h"

namespace
{

const double TestTolerance = 1.0e-8; // Wire-P-Curve求交和通用二维曲线求交统一使用1e-8测试容差。

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

bool nearEqual(double first, double second, double tolerance = 1.0e-6)
{
    return std::fabs(first - second) <= tolerance;
}

bool pointEqual(const MyMath::Vector2& first, const MyMath::Vector2& second, double tolerance = 1.0e-6)
{
    return first.isEqualTo(second, tolerance);
}

// 测试标准矩形边界与水平cutting P-Curve的两个内部Edge命中。
void testRectangleCrossing(TestContext& context)
{
    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createRectangle(10.0, 8.0), TestTolerance);
    const MyBRep::Topology_Wire wire = face.wire(0);
    const MyBRep::Geometry_Line2D cuttingCurve(MyMath::Vector2(-10.0, 0.0), MyMath::Vector2::unitX());

    const MyBRep::Operation::Intersection::WireCurve2DIntersectionResult result =
        MyBRep::Operation::Intersection::intersectWireCurve2D(wire, face.geometry(), cuttingCurve, 0.0, 20.0, TestTolerance);

    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Points && result.points.size() == 2,
                   "Rectangle crossing count");
    context.expect(pointEqual(result.points[0].point, MyMath::Vector2(-5.0, 0.0)) && pointEqual(result.points[1].point, MyMath::Vector2(5.0, 0.0)),
                   "Rectangle crossing point order");
    context.expect(result.points[0].edgeIndex == 3 && result.points[1].edgeIndex == 1, "Rectangle crossing Edge indices");
    context.expect(nearEqual(result.points[0].edgeParameter, 0.5) && nearEqual(result.points[1].edgeParameter, 0.5),
                   "Rectangle crossing Edge normalized parameters");
    context.expect(nearEqual(result.points[0].curveNormalizedParameter, 0.25) && nearEqual(result.points[1].curveNormalizedParameter, 0.75),
                   "Rectangle crossing cutting normalized parameters");

    const MyBRep::Operation::Split::WireBoundarySplit split = MyBRep::Operation::Split::splitClosedWireBoundary(
        wire,
        MyBRep::Operation::Split::WireSplitLocation(result.points[0].edgeIndex, result.points[0].edgeParameter),
        MyBRep::Operation::Split::WireSplitLocation(result.points[1].edgeIndex, result.points[1].edgeParameter));

    context.expect(split.wire.isClosed() && split.wire.edgeCount() == 6, "Intersection locations feed WireSplit directly");
    context.expect(split.firstVertex.point().isEqualTo(MyMath::Vector3(-5.0, 0.0, 0.0), 1.0e-6) &&
                   split.secondVertex.point().isEqualTo(MyMath::Vector3(5.0, 0.0, 0.0), 1.0e-6),
                   "Intersection-WireSplit resolved boundary vertices");
}

// 测试cutting有限区间反向时结果仍按cutting当前使用方向排列。
void testReversedCuttingCurve(TestContext& context)
{
    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createRectangle(10.0, 8.0), TestTolerance);
    const MyBRep::Geometry_Line2D cuttingCurve(MyMath::Vector2(-10.0, 0.0), MyMath::Vector2::unitX());

    const MyBRep::Operation::Intersection::WireCurve2DIntersectionResult result =
        MyBRep::Operation::Intersection::intersectWireCurve2D(face.wire(0), face.geometry(), cuttingCurve, 20.0, 0.0, TestTolerance);

    context.expect(result.points.size() == 2 && pointEqual(result.points[0].point, MyMath::Vector2(5.0, 0.0)) &&
                   pointEqual(result.points[1].point, MyMath::Vector2(-5.0, 0.0)), "Reversed cutting curve result order");
    context.expect(result.points.size() == 2 && nearEqual(result.points[0].curveNormalizedParameter, 0.25) &&
                   nearEqual(result.points[1].curveNormalizedParameter, 0.75), "Reversed cutting curve normalized parameters");
}

// 测试Reversed Wire索引和Edge规范化参数使用当前Wire遍历方向。
void testReversedWire(TestContext& context)
{
    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createRectangle(10.0, 8.0), TestTolerance);
    const MyBRep::Topology_Wire wire = face.wire(0).reversed();
    const MyBRep::Geometry_Line2D cuttingCurve(MyMath::Vector2(-10.0, 0.0), MyMath::Vector2::unitX());

    const MyBRep::Operation::Intersection::WireCurve2DIntersectionResult result =
        MyBRep::Operation::Intersection::intersectWireCurve2D(wire, face.geometry(), cuttingCurve, 0.0, 20.0, TestTolerance);

    context.expect(result.points.size() == 2 && result.points[0].edgeIndex == 0 && result.points[1].edgeIndex == 2,
                   "Reversed Wire current-direction Edge indices");
    context.expect(result.points.size() == 2 && nearEqual(result.points[0].edgeParameter, 0.5) && nearEqual(result.points[1].edgeParameter, 0.5),
                   "Reversed Wire current-direction Edge parameters");
}

// 测试共享Wire顶点被相邻两条Edge同时命中时只返回一个规范化边界位置。
void testVertexDeduplication(TestContext& context)
{
    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createRectangle(10.0, 8.0), TestTolerance);
    const MyMath::Vector2 direction(10.0, 8.0);
    const MyBRep::Geometry_Line2D cuttingCurve(MyMath::Vector2(-5.0, -4.0), direction);
    const double length = direction.length();

    const MyBRep::Operation::Intersection::WireCurve2DIntersectionResult result =
        MyBRep::Operation::Intersection::intersectWireCurve2D(face.wire(0), face.geometry(), cuttingCurve, 0.0, length, TestTolerance);

    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Points && result.points.size() == 2,
                   "Shared boundary Vertex duplicate hits collapse");
    context.expect(result.points.size() == 2 && result.points[0].edgeIndex == 0 && nearEqual(result.points[0].edgeParameter, 0.0) &&
                   result.points[1].edgeIndex == 2 && nearEqual(result.points[1].edgeParameter, 0.0),
                   "Shared boundary Vertex canonical WireSplit locations");
    context.expect(result.points.size() == 2 && pointEqual(result.points[0].point, MyMath::Vector2(-5.0, -4.0)) &&
                   pointEqual(result.points[1].point, MyMath::Vector2(5.0, 4.0)), "Shared boundary Vertex coordinates");
}

// 测试cutting曲线与完整边界Edge连续重合时提升为Wire级Overlap。
void testBoundaryOverlap(TestContext& context)
{
    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createRectangle(10.0, 8.0), TestTolerance);
    const MyBRep::Geometry_Line2D cuttingCurve(MyMath::Vector2(-5.0, -4.0), MyMath::Vector2::unitX());

    const MyBRep::Operation::Intersection::WireCurve2DIntersectionResult result =
        MyBRep::Operation::Intersection::intersectWireCurve2D(face.wire(0), face.geometry(), cuttingCurve, 0.0, 10.0, TestTolerance);

    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Overlap && result.points.empty(),
                   "Boundary Edge overlap propagates to Wire result");
}

// 测试完全位于边界之外的cutting曲线无命中。
void testNoIntersection(TestContext& context)
{
    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createRectangle(10.0, 8.0), TestTolerance);
    const MyBRep::Geometry_Line2D cuttingCurve(MyMath::Vector2(-10.0, 10.0), MyMath::Vector2::unitX());

    const MyBRep::Operation::Intersection::WireCurve2DIntersectionResult result =
        MyBRep::Operation::Intersection::intersectWireCurve2D(face.wire(0), face.geometry(), cuttingCurve, 0.0, 20.0, TestTolerance);

    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::None && result.points.empty(), "Wire-Curve outside none");
}

// 测试单Edge圆Wire边界能够在同一Edge-use上返回两个命中。
void testCircleBoundary(TestContext& context)
{
    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createCircle(5.0), TestTolerance);
    const MyBRep::Geometry_Line2D cuttingCurve(MyMath::Vector2(-10.0, 0.0), MyMath::Vector2::unitX());

    const MyBRep::Operation::Intersection::WireCurve2DIntersectionResult result =
        MyBRep::Operation::Intersection::intersectWireCurve2D(face.wire(0), face.geometry(), cuttingCurve, 0.0, 20.0, TestTolerance);

    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Points && result.points.size() == 2,
                   "Circle Wire two boundary hits");
    context.expect(result.points.size() == 2 && result.points[0].edgeIndex == 0 && result.points[1].edgeIndex == 0,
                   "Circle Wire hits remain on single Edge-use");
    context.expect(result.points.size() == 2 && pointEqual(result.points[0].point, MyMath::Vector2(-5.0, 0.0)) &&
                   pointEqual(result.points[1].point, MyMath::Vector2(5.0, 0.0)), "Circle Wire hit coordinates");
}

// 测试Bezier cutting曲线通过已验证的General Curve-Curve路径与矩形P-Curve边界求交。
void testBezierCuttingCurve(TestContext& context)
{
    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createRectangle(10.0, 8.0), TestTolerance);
    std::vector<MyMath::Vector2> controlPoints;
    controlPoints.push_back(MyMath::Vector2(-10.0, 0.0));
    controlPoints.push_back(MyMath::Vector2(0.0, 0.0));
    controlPoints.push_back(MyMath::Vector2(10.0, 0.0));
    const MyBRep::Geometry_Bezier2D cuttingCurve(controlPoints);

    const MyBRep::Operation::Intersection::WireCurve2DIntersectionResult result =
        MyBRep::Operation::Intersection::intersectWireCurve2D(face.wire(0), face.geometry(), cuttingCurve, 0.0, 1.0, TestTolerance);

    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Points && result.points.size() == 2,
                   "Bezier cutting curve boundary count");
    context.expect(result.points.size() == 2 && pointEqual(result.points[0].point, MyMath::Vector2(-5.0, 0.0)) &&
                   pointEqual(result.points[1].point, MyMath::Vector2(5.0, 0.0)), "Bezier cutting curve boundary coordinates");
}

// 测试BSpline cutting曲线通过已验证的General Curve-Curve路径与矩形P-Curve边界求交。
void testBSplineCuttingCurve(TestContext& context)
{
    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createRectangle(10.0, 8.0), TestTolerance);
    std::vector<MyMath::Vector2> controlPoints;
    controlPoints.push_back(MyMath::Vector2(-10.0, 0.0));
    controlPoints.push_back(MyMath::Vector2(0.0, 0.0));
    controlPoints.push_back(MyMath::Vector2(10.0, 0.0));
    std::vector<double> knots;
    knots.push_back(0.0);
    knots.push_back(0.0);
    knots.push_back(0.0);
    knots.push_back(1.0);
    knots.push_back(1.0);
    knots.push_back(1.0);
    const MyBRep::Geometry_BSpline2D cuttingCurve(2, controlPoints, knots);

    const MyBRep::Operation::Intersection::WireCurve2DIntersectionResult result =
        MyBRep::Operation::Intersection::intersectWireCurve2D(face.wire(0), face.geometry(), cuttingCurve, 0.0, 1.0, TestTolerance);

    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Points && result.points.size() == 2,
                   "BSpline cutting curve boundary count");
    context.expect(result.points.size() == 2 && pointEqual(result.points[0].point, MyMath::Vector2(-5.0, 0.0)) &&
                   pointEqual(result.points[1].point, MyMath::Vector2(5.0, 0.0)), "BSpline cutting curve boundary coordinates");
}

}

int main()
{
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep Wire-Curve 2D Intersection Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testRectangleCrossing(context);
    testReversedCuttingCurve(context);
    testReversedWire(context);
    testVertexDeduplication(context);
    testBoundaryOverlap(context);
    testNoIntersection(context);
    testCircleBoundary(context);
    testBezierCuttingCurve(context);
    testBSplineCuttingCurve(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}