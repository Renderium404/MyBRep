#include <cmath>
#include <iostream>
#include <string>

#include "MyMath/Vector2.h"
#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Circle2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Curve2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Line2D.h"
#include "MyBRep/Operation/Intersection/CurveCurve2DIntersection.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 二维圆测试统一使用弧度制。
const double TwoPi = Pi * 2.0; // 完整二维圆测试周期。
const double TestTolerance = 1.0e-10; // 求交专项测试统一几何容差。

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

bool nearEqual(double first, double second)
{
    return std::fabs(first - second) <= TestTolerance;
}

bool pointEqual(const MyMath::Vector2& first, const MyMath::Vector2& second)
{
    return first.isEqualTo(second, TestTolerance);
}

void testLineLine(TestContext& context)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> horizontal(
        new MyBRep::Geometry_Line2D(MyMath::Vector2(-5.0, 0.0), MyMath::Vector2::unitX()));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> vertical(
        new MyBRep::Geometry_Line2D(MyMath::Vector2(0.0, -5.0), MyMath::Vector2::unitY()));

    MyBRep::Operation::Intersection::CurveCurve2DIntersectionResult result =
        MyBRep::Operation::Intersection::intersectCurveCurve2D(*horizontal, 0.0, 10.0, *vertical, 0.0, 10.0, TestTolerance);

    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Points && result.points.size() == 1,
                   "Line-Line crossing count");
    context.expect(pointEqual(result.points[0].point, MyMath::Vector2(0.0, 0.0)), "Line-Line crossing point");
    context.expect(nearEqual(result.points[0].firstParameter, 5.0) && nearEqual(result.points[0].secondParameter, 5.0),
                   "Line-Line curve parameters");
    context.expect(nearEqual(result.points[0].firstNormalizedParameter, 0.5) && nearEqual(result.points[0].secondNormalizedParameter, 0.5),
                   "Line-Line normalized parameters");

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> parallel(
        new MyBRep::Geometry_Line2D(MyMath::Vector2(-5.0, 2.0), MyMath::Vector2::unitX()));
    result = MyBRep::Operation::Intersection::intersectCurveCurve2D(*horizontal, 0.0, 10.0, *parallel, 0.0, 10.0, TestTolerance);
    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::None, "Line-Line parallel none");

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> collinear(
        new MyBRep::Geometry_Line2D(MyMath::Vector2(0.0, 0.0), MyMath::Vector2::unitX()));
    result = MyBRep::Operation::Intersection::intersectCurveCurve2D(*horizontal, 0.0, 10.0, *collinear, 0.0, 10.0, TestTolerance);
    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Overlap, "Line-Line collinear overlap");

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> touching(
        new MyBRep::Geometry_Line2D(MyMath::Vector2(5.0, 0.0), MyMath::Vector2::unitX()));
    result = MyBRep::Operation::Intersection::intersectCurveCurve2D(*horizontal, 0.0, 10.0, *touching, 0.0, 4.0, TestTolerance);
    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Points && result.points.size() == 1,
                   "Line-Line endpoint touch");
    context.expect(pointEqual(result.points[0].point, MyMath::Vector2(5.0, 0.0)), "Line-Line endpoint touch point");

    result = MyBRep::Operation::Intersection::intersectCurveCurve2D(*horizontal, 10.0, 0.0, *vertical, 0.0, 10.0, TestTolerance);
    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Points &&
                   nearEqual(result.points[0].firstNormalizedParameter, 0.5), "Line-Line reversed finite-use direction");
}

void testLineCircle(TestContext& context)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> line(
        new MyBRep::Geometry_Line2D(MyMath::Vector2(-10.0, 0.0), MyMath::Vector2::unitX()));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> circle(
        new MyBRep::Geometry_Circle2D(MyMath::Vector2::zero(), 5.0));

    MyBRep::Operation::Intersection::CurveCurve2DIntersectionResult result =
        MyBRep::Operation::Intersection::intersectCurveCurve2D(*line, 0.0, 20.0, *circle, 0.0, TwoPi, TestTolerance);

    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Points && result.points.size() == 2,
                   "Line-Circle secant count");
    context.expect(pointEqual(result.points[0].point, MyMath::Vector2(-5.0, 0.0)) &&
                   pointEqual(result.points[1].point, MyMath::Vector2(5.0, 0.0)), "Line-Circle secant points");
    context.expect(nearEqual(result.points[0].firstNormalizedParameter, 0.25) &&
                   nearEqual(result.points[1].firstNormalizedParameter, 0.75), "Line-Circle result ordering");

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> tangent(
        new MyBRep::Geometry_Line2D(MyMath::Vector2(-10.0, 5.0), MyMath::Vector2::unitX()));
    result = MyBRep::Operation::Intersection::intersectCurveCurve2D(*tangent, 0.0, 20.0, *circle, 0.0, TwoPi, TestTolerance);
    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Points && result.points.size() == 1,
                   "Line-Circle tangent count");
    context.expect(pointEqual(result.points[0].point, MyMath::Vector2(0.0, 5.0)), "Line-Circle tangent point");

    result = MyBRep::Operation::Intersection::intersectCurveCurve2D(*line, 0.0, 20.0, *circle, 0.0, Pi * 0.5, TestTolerance);
    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Points && result.points.size() == 1,
                   "Line-Circle finite arc trim");
    context.expect(pointEqual(result.points[0].point, MyMath::Vector2(5.0, 0.0)), "Line-Circle finite arc point");

    result = MyBRep::Operation::Intersection::intersectCurveCurve2D(*circle, 0.0, TwoPi, *line, 0.0, 20.0, TestTolerance);
    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Points && result.points.size() == 2,
                   "Circle-Line symmetric dispatch");
}

void testCircleCircle(TestContext& context)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> first(
        new MyBRep::Geometry_Circle2D(MyMath::Vector2(0.0, 0.0), 5.0));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> second(
        new MyBRep::Geometry_Circle2D(MyMath::Vector2(8.0, 0.0), 5.0));

    MyBRep::Operation::Intersection::CurveCurve2DIntersectionResult result =
        MyBRep::Operation::Intersection::intersectCurveCurve2D(*first, 0.0, TwoPi, *second, 0.0, TwoPi, TestTolerance);

    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Points && result.points.size() == 2,
                   "Circle-Circle two-point count");
    context.expect((pointEqual(result.points[0].point, MyMath::Vector2(4.0, 3.0)) ||
                    pointEqual(result.points[0].point, MyMath::Vector2(4.0, -3.0))) &&
                   (pointEqual(result.points[1].point, MyMath::Vector2(4.0, 3.0)) ||
                    pointEqual(result.points[1].point, MyMath::Vector2(4.0, -3.0))),
                   "Circle-Circle two-point coordinates");

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> tangent(
        new MyBRep::Geometry_Circle2D(MyMath::Vector2(10.0, 0.0), 5.0));
    result = MyBRep::Operation::Intersection::intersectCurveCurve2D(*first, 0.0, TwoPi, *tangent, 0.0, TwoPi, TestTolerance);
    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Points && result.points.size() == 1,
                   "Circle-Circle tangent count");
    context.expect(pointEqual(result.points[0].point, MyMath::Vector2(5.0, 0.0)), "Circle-Circle tangent point");

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> disjoint(
        new MyBRep::Geometry_Circle2D(MyMath::Vector2(20.0, 0.0), 5.0));
    result = MyBRep::Operation::Intersection::intersectCurveCurve2D(*first, 0.0, TwoPi, *disjoint, 0.0, TwoPi, TestTolerance);
    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::None, "Circle-Circle disjoint");

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> coincident(
        new MyBRep::Geometry_Circle2D(MyMath::Vector2(0.0, 0.0), 5.0, MyMath::Vector2::unitY(), MyMath::Vector2(-1.0, 0.0)));
    result = MyBRep::Operation::Intersection::intersectCurveCurve2D(*first, 0.0, TwoPi, *coincident, 0.0, TwoPi, TestTolerance);
    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Overlap, "Circle-Circle coincident full overlap");

    result = MyBRep::Operation::Intersection::intersectCurveCurve2D(*first, 0.0, Pi * 0.25, *coincident, Pi, Pi * 1.25, TestTolerance);
    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::None, "Circle-Circle coincident disjoint arcs");

    result = MyBRep::Operation::Intersection::intersectCurveCurve2D(*first, 0.0, Pi * 0.5, *first, Pi * 0.5, Pi, TestTolerance);
    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Points && result.points.size() == 1,
                   "Circle-Circle coincident endpoint touch");
    context.expect(pointEqual(result.points[0].point, MyMath::Vector2(0.0, 5.0)), "Circle-Circle coincident endpoint point");

    result = MyBRep::Operation::Intersection::intersectCurveCurve2D(*first, 0.0, Pi, *first, Pi * 0.5, Pi * 1.5, TestTolerance);
    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Overlap, "Circle-Circle coincident partial overlap");
}

}

int main()
{
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep Curve-Curve 2D Intersection Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testLineLine(context);
    testLineCircle(context);
    testCircleCircle(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}
