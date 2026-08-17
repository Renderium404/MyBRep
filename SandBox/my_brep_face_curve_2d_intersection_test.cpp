#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "MyMath/Vector2.h"
#include "MyMath/Vector3.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Bezier2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Line2D.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Modeling/Wire/WireModeling.h"
#include "MyBRep/Operation/Intersection/FaceCurve2DIntersection.h"
#include "MyBRep/Operation/Split/WireSplit.h"

namespace
{

const double TestTolerance = 1.0e-8; // Face-P-Curve聚合测试沿用当前二维求交的1e-8几何容差。

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

// 创建包含外矩形与中心内矩形两个trimming Wire的Planar Face；这里只验证Wire聚合，不赋予Outer/Hole语义。
MyBRep::Topology_Face createTwoWireFace()
{
    std::vector<MyBRep::Topology_Wire> wires;
    wires.push_back(MyBRep::Modeling::createRectangle(10.0, 8.0));
    wires.push_back(MyBRep::Modeling::createRectangle(4.0, 2.0));
    return MyBRep::Modeling::createPlanarFace(wires, TestTolerance);
}

// 测试单Wire Face结果与已验证Wire-Curve层保持一致。
void testSingleWireFace(TestContext& context)
{
    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createRectangle(10.0, 8.0), TestTolerance);
    const MyBRep::Geometry_Line2D cuttingCurve(MyMath::Vector2(-10.0, 0.0), MyMath::Vector2::unitX());

    const MyBRep::Operation::Intersection::FaceCurve2DIntersectionResult result =
        MyBRep::Operation::Intersection::intersectFaceCurve2D(face, cuttingCurve, 0.0, 20.0, TestTolerance);

    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Points && result.points.size() == 2,
                   "Single-Wire Face crossing count");
    context.expect(result.points.size() == 2 && result.points[0].wireIndex == 0 && result.points[1].wireIndex == 0,
                   "Single-Wire Face wire indices");
    context.expect(result.points.size() == 2 && result.points[0].edgeIndex == 3 && result.points[1].edgeIndex == 1,
                   "Single-Wire Face Edge indices");
    context.expect(result.points.size() == 2 && nearEqual(result.points[0].edgeParameter, 0.5) && nearEqual(result.points[1].edgeParameter, 0.5),
                   "Single-Wire Face Edge parameters");
    context.expect(result.points.size() == 2 && pointEqual(result.points[0].point, MyMath::Vector2(-5.0, 0.0)) &&
                   pointEqual(result.points[1].point, MyMath::Vector2(5.0, 0.0)), "Single-Wire Face crossing order");
}

// 测试多Wire Face聚合全部边界命中并按cutting曲线方向统一排序。
void testMultipleWires(TestContext& context)
{
    const MyBRep::Topology_Face face = createTwoWireFace();
    const MyBRep::Geometry_Line2D cuttingCurve(MyMath::Vector2(-10.0, 0.0), MyMath::Vector2::unitX());

    const MyBRep::Operation::Intersection::FaceCurve2DIntersectionResult result =
        MyBRep::Operation::Intersection::intersectFaceCurve2D(face, cuttingCurve, 0.0, 20.0, TestTolerance);

    context.expect(face.wireCount() == 2, "Two-Wire Face setup");
    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Points && result.points.size() == 4,
                   "Two-Wire Face crossing count");
    context.expect(result.points.size() == 4 && pointEqual(result.points[0].point, MyMath::Vector2(-5.0, 0.0)) &&
                   pointEqual(result.points[1].point, MyMath::Vector2(-2.0, 0.0)) &&
                   pointEqual(result.points[2].point, MyMath::Vector2(2.0, 0.0)) &&
                   pointEqual(result.points[3].point, MyMath::Vector2(5.0, 0.0)), "Two-Wire Face global cutting order");
    context.expect(result.points.size() == 4 && result.points[0].wireIndex == 0 && result.points[1].wireIndex == 1 &&
                   result.points[2].wireIndex == 1 && result.points[3].wireIndex == 0, "Two-Wire Face wire index attribution");
    context.expect(result.points.size() == 4 && result.points[0].edgeIndex == 3 && result.points[1].edgeIndex == 3 &&
                   result.points[2].edgeIndex == 1 && result.points[3].edgeIndex == 1, "Two-Wire Face Edge index attribution");
    context.expect(result.points.size() == 4 && nearEqual(result.points[0].edgeParameter, 0.5) && nearEqual(result.points[1].edgeParameter, 0.5) &&
                   nearEqual(result.points[2].edgeParameter, 0.5) && nearEqual(result.points[3].edgeParameter, 0.5),
                   "Two-Wire Face Edge normalized parameters");

    const MyBRep::Operation::Split::WireBoundarySplit outerSplit = MyBRep::Operation::Split::splitClosedWireBoundary(
        face.wire(result.points[0].wireIndex),
        MyBRep::Operation::Split::WireSplitLocation(result.points[0].edgeIndex, result.points[0].edgeParameter),
        MyBRep::Operation::Split::WireSplitLocation(result.points[3].edgeIndex, result.points[3].edgeParameter));
    const MyBRep::Operation::Split::WireBoundarySplit innerSplit = MyBRep::Operation::Split::splitClosedWireBoundary(
        face.wire(result.points[1].wireIndex),
        MyBRep::Operation::Split::WireSplitLocation(result.points[1].edgeIndex, result.points[1].edgeParameter),
        MyBRep::Operation::Split::WireSplitLocation(result.points[2].edgeIndex, result.points[2].edgeParameter));

    context.expect(outerSplit.wire.edgeCount() == 6 && innerSplit.wire.edgeCount() == 6,
                   "Face intersection locations feed corresponding WireSplit operations");
}

// 测试cutting有限区间反向时跨多个Wire的全局结果顺序随cutting方向反转。
void testReversedCuttingCurve(TestContext& context)
{
    const MyBRep::Topology_Face face = createTwoWireFace();
    const MyBRep::Geometry_Line2D cuttingCurve(MyMath::Vector2(-10.0, 0.0), MyMath::Vector2::unitX());

    const MyBRep::Operation::Intersection::FaceCurve2DIntersectionResult result =
        MyBRep::Operation::Intersection::intersectFaceCurve2D(face, cuttingCurve, 20.0, 0.0, TestTolerance);

    context.expect(result.points.size() == 4 && pointEqual(result.points[0].point, MyMath::Vector2(5.0, 0.0)) &&
                   pointEqual(result.points[1].point, MyMath::Vector2(2.0, 0.0)) &&
                   pointEqual(result.points[2].point, MyMath::Vector2(-2.0, 0.0)) &&
                   pointEqual(result.points[3].point, MyMath::Vector2(-5.0, 0.0)), "Reversed cutting curve global Face order");
    context.expect(result.points.size() == 4 && nearEqual(result.points[0].curveNormalizedParameter, 0.25) &&
                   nearEqual(result.points[1].curveNormalizedParameter, 0.4) &&
                   nearEqual(result.points[2].curveNormalizedParameter, 0.6) &&
                   nearEqual(result.points[3].curveNormalizedParameter, 0.75), "Reversed cutting curve global normalized parameters");
}

// 测试Reversed Face直接依赖Face::wire()返回的当前方向Wire，不改变wireIndex，只改变Edge-use索引。
void testReversedFace(TestContext& context)
{
    const MyBRep::Topology_Face face = createTwoWireFace().reversed();
    const MyBRep::Geometry_Line2D cuttingCurve(MyMath::Vector2(-10.0, 0.0), MyMath::Vector2::unitX());

    const MyBRep::Operation::Intersection::FaceCurve2DIntersectionResult result =
        MyBRep::Operation::Intersection::intersectFaceCurve2D(face, cuttingCurve, 0.0, 20.0, TestTolerance);

    context.expect(result.points.size() == 4 && result.points[0].wireIndex == 0 && result.points[1].wireIndex == 1 &&
                   result.points[2].wireIndex == 1 && result.points[3].wireIndex == 0, "Reversed Face preserves trimming Wire indices");
    context.expect(result.points.size() == 4 && result.points[0].edgeIndex == 0 && result.points[1].edgeIndex == 0 &&
                   result.points[2].edgeIndex == 2 && result.points[3].edgeIndex == 2, "Reversed Face uses current-direction Edge indices");
    context.expect(result.points.size() == 4 && nearEqual(result.points[0].edgeParameter, 0.5) && nearEqual(result.points[1].edgeParameter, 0.5) &&
                   nearEqual(result.points[2].edgeParameter, 0.5) && nearEqual(result.points[3].edgeParameter, 0.5),
                   "Reversed Face uses current-direction Edge parameters");
}

// 测试某个Face trimming Wire与cutting曲线连续重合时记录具体overlap wire索引。
void testBoundaryOverlap(TestContext& context)
{
    const MyBRep::Topology_Face face = createTwoWireFace();
    const MyBRep::Geometry_Line2D cuttingCurve(MyMath::Vector2(-5.0, -4.0), MyMath::Vector2::unitX());

    const MyBRep::Operation::Intersection::FaceCurve2DIntersectionResult result =
        MyBRep::Operation::Intersection::intersectFaceCurve2D(face, cuttingCurve, 0.0, 10.0, TestTolerance);

    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Overlap && result.points.empty(),
                   "Face boundary overlap propagates");
    context.expect(result.overlapWireIndices.size() == 1 && result.overlapWireIndices[0] == 0,
                   "Face boundary overlap records Wire index");
}

// 测试完全位于Face全部边界之外的cutting曲线返回None。
void testNoIntersection(TestContext& context)
{
    const MyBRep::Topology_Face face = createTwoWireFace();
    const MyBRep::Geometry_Line2D cuttingCurve(MyMath::Vector2(-10.0, 10.0), MyMath::Vector2::unitX());

    const MyBRep::Operation::Intersection::FaceCurve2DIntersectionResult result =
        MyBRep::Operation::Intersection::intersectFaceCurve2D(face, cuttingCurve, 0.0, 20.0, TestTolerance);

    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::None &&
                   result.points.empty() && result.overlapWireIndices.empty(), "Face-Curve outside none");
}

// 测试没有显式trimming Wire的Face没有边界命中。
void testUntrimmedFace(TestContext& context)
{
    const MyBRep::Topology_Face trimmed = MyBRep::Modeling::createPlanarFace(MyBRep::Modeling::createRectangle(10.0, 8.0), TestTolerance);
    const MyBRep::Topology_Face face(trimmed.geometryResource());
    const MyBRep::Geometry_Line2D cuttingCurve(MyMath::Vector2(-10.0, 0.0), MyMath::Vector2::unitX());

    const MyBRep::Operation::Intersection::FaceCurve2DIntersectionResult result =
        MyBRep::Operation::Intersection::intersectFaceCurve2D(face, cuttingCurve, 0.0, 20.0, TestTolerance);

    context.expect(face.wireCount() == 0 && result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::None &&
                   result.points.empty(), "Untrimmed Face has no trimming boundary hits");
}

// 测试Bezier cutting曲线通过Face聚合层继续复用General Curve-Curve路径。
void testBezierCuttingCurve(TestContext& context)
{
    const MyBRep::Topology_Face face = createTwoWireFace();
    std::vector<MyMath::Vector2> controlPoints;
    controlPoints.push_back(MyMath::Vector2(-10.0, 0.0));
    controlPoints.push_back(MyMath::Vector2(0.0, 0.0));
    controlPoints.push_back(MyMath::Vector2(10.0, 0.0));
    const MyBRep::Geometry_Bezier2D cuttingCurve(controlPoints);

    const MyBRep::Operation::Intersection::FaceCurve2DIntersectionResult result =
        MyBRep::Operation::Intersection::intersectFaceCurve2D(face, cuttingCurve, 0.0, 1.0, TestTolerance);

    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Points && result.points.size() == 4,
                   "Bezier cutting curve Face boundary count");
    context.expect(result.points.size() == 4 && pointEqual(result.points[0].point, MyMath::Vector2(-5.0, 0.0)) &&
                   pointEqual(result.points[1].point, MyMath::Vector2(-2.0, 0.0)) &&
                   pointEqual(result.points[2].point, MyMath::Vector2(2.0, 0.0)) &&
                   pointEqual(result.points[3].point, MyMath::Vector2(5.0, 0.0)), "Bezier cutting curve Face boundary order");
}

}

int main()
{
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep Face-Curve 2D Intersection Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testSingleWireFace(context);
    testMultipleWires(context);
    testReversedCuttingCurve(context);
    testReversedFace(context);
    testBoundaryOverlap(context);
    testNoIntersection(context);
    testUntrimmedFace(context);
    testBezierCuttingCurve(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}