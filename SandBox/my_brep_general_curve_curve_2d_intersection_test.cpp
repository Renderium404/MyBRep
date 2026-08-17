#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "MyMath/Vector2.h"
#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Curve2D/Geometry_BSpline2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Bezier2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Circle2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Line2D.h"
#include "MyBRep/Operation/Intersection/CurveCurve2DIntersection.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 圆测试统一使用弧度制。
const double TestTolerance = 1.0e-8; // 通用数值求交专项测试使用1e-8几何容差。

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

MyBRep::Geometry_Bezier2D* makeParabolaBezier()
{
    std::vector<MyMath::Vector2> points;
    points.push_back(MyMath::Vector2(-5.0, -5.0));
    points.push_back(MyMath::Vector2(0.0, 5.0));
    points.push_back(MyMath::Vector2(5.0, -5.0));
    return new MyBRep::Geometry_Bezier2D(points);
}

MyBRep::Geometry_BSpline2D* makeParabolaBSpline()
{
    std::vector<MyMath::Vector2> points;
    points.push_back(MyMath::Vector2(-5.0, -5.0));
    points.push_back(MyMath::Vector2(0.0, 5.0));
    points.push_back(MyMath::Vector2(5.0, -5.0));

    std::vector<double> knots;
    knots.push_back(0.0);
    knots.push_back(0.0);
    knots.push_back(0.0);
    knots.push_back(1.0);
    knots.push_back(1.0);
    knots.push_back(1.0);

    return new MyBRep::Geometry_BSpline2D(2, points, knots);
}

void testBezierIntersections(TestContext& context)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> bezier(makeParabolaBezier());
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> line(
        new MyBRep::Geometry_Line2D(MyMath::Vector2(-10.0, -2.0), MyMath::Vector2::unitX()));

    MyBRep::Operation::Intersection::CurveCurve2DIntersectionResult result =
        MyBRep::Operation::Intersection::intersectCurveCurve2D(*line, 0.0, 20.0, *bezier, 0.0, 1.0, TestTolerance);

    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Points && result.points.size() == 2,
                   "Line-Bezier two-point count");
    context.expect(result.points.size() == 2 && result.points[0].point.x() < 0.0 && result.points[1].point.x() > 0.0,
                   "Line-Bezier point ordering");
    context.expect(result.points.size() == 2 && nearEqual(result.points[0].point.y(), -2.0) && nearEqual(result.points[1].point.y(), -2.0),
                   "Line-Bezier point coordinates");

    result = MyBRep::Operation::Intersection::intersectCurveCurve2D(*bezier, 1.0, 0.0, *line, 0.0, 20.0, TestTolerance);
    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Points && result.points.size() == 2 &&
                   result.points[0].point.x() > result.points[1].point.x(), "Bezier-Line reversed result ordering");

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> tangent(
        new MyBRep::Geometry_Line2D(MyMath::Vector2(-10.0, 0.0), MyMath::Vector2::unitX()));
    result = MyBRep::Operation::Intersection::intersectCurveCurve2D(*tangent, 0.0, 20.0, *bezier, 0.0, 1.0, TestTolerance);
    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Points && result.points.size() == 1,
                   "Line-Bezier tangent count");
    context.expect(result.points.size() == 1 && pointEqual(result.points[0].point, MyMath::Vector2(0.0, 0.0)),
                   "Line-Bezier tangent point");

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> circle(
        new MyBRep::Geometry_Circle2D(MyMath::Vector2::zero(), 5.0));
    std::vector<MyMath::Vector2> lineBezierPoints;
    lineBezierPoints.push_back(MyMath::Vector2(-10.0, 0.0));
    lineBezierPoints.push_back(MyMath::Vector2(0.0, 0.0));
    lineBezierPoints.push_back(MyMath::Vector2(10.0, 0.0));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> lineBezier(new MyBRep::Geometry_Bezier2D(lineBezierPoints));

    result = MyBRep::Operation::Intersection::intersectCurveCurve2D(*circle, 0.0, Pi * 2.0, *lineBezier, 0.0, 1.0, TestTolerance);
    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Points && result.points.size() == 2,
                   "Circle-Bezier two-point count");
    context.expect(result.points.size() == 2 &&
                   ((nearEqual(std::fabs(result.points[0].point.x()), 5.0) && nearEqual(std::fabs(result.points[1].point.x()), 5.0))),
                   "Circle-Bezier point coordinates");

    std::vector<MyMath::Vector2> diagonalPoints;
    diagonalPoints.push_back(MyMath::Vector2(-5.0, -5.0));
    diagonalPoints.push_back(MyMath::Vector2(0.0, 0.0));
    diagonalPoints.push_back(MyMath::Vector2(5.0, 5.0));
    std::vector<MyMath::Vector2> reverseDiagonalPoints;
    reverseDiagonalPoints.push_back(MyMath::Vector2(-5.0, 5.0));
    reverseDiagonalPoints.push_back(MyMath::Vector2(0.0, 0.0));
    reverseDiagonalPoints.push_back(MyMath::Vector2(5.0, -5.0));

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> firstBezier(new MyBRep::Geometry_Bezier2D(diagonalPoints));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> secondBezier(new MyBRep::Geometry_Bezier2D(reverseDiagonalPoints));

    result = MyBRep::Operation::Intersection::intersectCurveCurve2D(*firstBezier, 0.0, 1.0, *secondBezier, 0.0, 1.0, TestTolerance);
    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Points && result.points.size() == 1,
                   "Bezier-Bezier crossing count");
    context.expect(result.points.size() == 1 && pointEqual(result.points[0].point, MyMath::Vector2::zero()), "Bezier-Bezier crossing point");

    result = MyBRep::Operation::Intersection::intersectCurveCurve2D(*firstBezier, 0.0, 0.5, *firstBezier, 0.5, 1.0, TestTolerance);
    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Points && result.points.size() == 1,
                   "Same Bezier endpoint touch");

    result = MyBRep::Operation::Intersection::intersectCurveCurve2D(*firstBezier, 0.0, 0.75, *firstBezier, 0.25, 1.0, TestTolerance);
    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Overlap, "Same Bezier finite-use overlap");
}

void testBSplineIntersections(TestContext& context)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> spline(makeParabolaBSpline());
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> line(
        new MyBRep::Geometry_Line2D(MyMath::Vector2(-10.0, -2.0), MyMath::Vector2::unitX()));

    MyBRep::Operation::Intersection::CurveCurve2DIntersectionResult result =
        MyBRep::Operation::Intersection::intersectCurveCurve2D(*spline, 0.0, 1.0, *line, 0.0, 20.0, TestTolerance);

    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Points && result.points.size() == 2,
                   "BSpline-Line two-point count");
    context.expect(result.points.size() == 2 && result.points[0].point.x() < 0.0 && result.points[1].point.x() > 0.0,
                   "BSpline-Line result ordering");

    std::vector<MyMath::Vector2> lineBezierPoints;
    lineBezierPoints.push_back(MyMath::Vector2(-10.0, 0.0));
    lineBezierPoints.push_back(MyMath::Vector2(0.0, 0.0));
    lineBezierPoints.push_back(MyMath::Vector2(10.0, 0.0));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> lineBezier(new MyBRep::Geometry_Bezier2D(lineBezierPoints));

    result = MyBRep::Operation::Intersection::intersectCurveCurve2D(*spline, 0.0, 1.0, *lineBezier, 0.0, 1.0, TestTolerance);
    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Points && result.points.size() == 1,
                   "BSpline-Bezier tangent count");
    context.expect(result.points.size() == 1 && pointEqual(result.points[0].point, MyMath::Vector2::zero()), "BSpline-Bezier tangent point");

    std::vector<MyMath::Vector2> diagonalPoints;
    diagonalPoints.push_back(MyMath::Vector2(-5.0, -5.0));
    diagonalPoints.push_back(MyMath::Vector2(0.0, 0.0));
    diagonalPoints.push_back(MyMath::Vector2(5.0, 5.0));

    std::vector<double> knots;
    knots.push_back(0.0);
    knots.push_back(0.0);
    knots.push_back(0.0);
    knots.push_back(1.0);
    knots.push_back(1.0);
    knots.push_back(1.0);

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> diagonalSpline(new MyBRep::Geometry_BSpline2D(2, diagonalPoints, knots));
    std::vector<MyMath::Vector2> reverseDiagonalPoints;
    reverseDiagonalPoints.push_back(MyMath::Vector2(-5.0, 5.0));
    reverseDiagonalPoints.push_back(MyMath::Vector2(0.0, 0.0));
    reverseDiagonalPoints.push_back(MyMath::Vector2(5.0, -5.0));
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> reverseSpline(new MyBRep::Geometry_BSpline2D(2, reverseDiagonalPoints, knots));

    result = MyBRep::Operation::Intersection::intersectCurveCurve2D(*diagonalSpline, 0.0, 1.0, *reverseSpline, 0.0, 1.0, TestTolerance);
    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Points && result.points.size() == 1,
                   "BSpline-BSpline crossing count");
    context.expect(result.points.size() == 1 && pointEqual(result.points[0].point, MyMath::Vector2::zero()), "BSpline-BSpline crossing point");

    result = MyBRep::Operation::Intersection::intersectCurveCurve2D(*diagonalSpline, 0.0, 0.8, *diagonalSpline, 0.2, 1.0, TestTolerance);
    context.expect(result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Overlap, "Same BSpline finite-use overlap");
}

}

int main()
{
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep General Curve-Curve 2D Intersection Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testBezierIntersections(context);
    testBSplineIntersections(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}
