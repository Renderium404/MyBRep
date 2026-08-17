#include "CurveCurve2DIntersection.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "MyBRep/Foundation/Diagnostic.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Circle2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Line2D.h"
#include "GeneralCurveCurve2DIntersection.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 二维圆求交和周期参数统一使用弧度制。
const double TwoPi = Pi * 2.0; // 完整二维圆参数周期。
const double NumericalScale = 64.0; // 参数、方向和平方判别式判断覆盖浮点舍入误差使用的固定倍数。

// 判断标量是否为有限值。
bool isFiniteValue(double value)
{
    const double infinity = (std::numeric_limits<double>::infinity)();
    return value == value && value != infinity && value != -infinity;
}

// 判断标量是否为有限非负值。
bool isFiniteNonNegative(double value)
{
    return isFiniteValue(value) && value >= 0.0;
}

// 返回二维向量长度。
double vectorLength(const MyMath::Vector2& vector)
{
    return vector.length();
}

// 返回参数区间尺度使用的数值容差。
double numericalParameterTolerance(double firstParameter, double lastParameter)
{
    const double scale = (std::max)(1.0, (std::max)(std::fabs(firstParameter), std::fabs(lastParameter)));
    return scale * (std::numeric_limits<double>::epsilon)() * NumericalScale;
}

// 返回指定曲线把几何容差换算到参数空间后的有效参数容差。
double curveParameterTolerance(const MyBRep::Geometry_Curve2D& curve, double firstParameter, double lastParameter, double tolerance)
{
    double geometricParameterTolerance = tolerance;

    if (curve.kind() == MyBRep::CurveKind::Circle)
    {
        const MyBRep::Geometry_Circle2D& circle = static_cast<const MyBRep::Geometry_Circle2D&>(curve);
        geometricParameterTolerance = tolerance / circle.radius();
    }

    return (std::max)(geometricParameterTolerance, numericalParameterTolerance(firstParameter, lastParameter));
}

// 校验有限曲线使用参数区间。
void validateCurveUse(const MyBRep::Geometry_Curve2D& curve, double firstParameter, double lastParameter)
{
    MYBREP_ASSERT_MESSAGE(isFiniteValue(firstParameter) && isFiniteValue(lastParameter),
                          "CurveCurve2DIntersection finite-use parameters must be finite.");
    MYBREP_ASSERT_MESSAGE(firstParameter != lastParameter,
                          "CurveCurve2DIntersection finite-use parameter interval must be non-degenerate.");
    MYBREP_ASSERT_MESSAGE(curve.isParameterInDomain(firstParameter) && curve.isParameterInDomain(lastParameter),
                          "CurveCurve2DIntersection finite-use parameters must lie in the curve natural parameter domain.");

    if (curve.isPeriodic())
    {
        const double parameterTolerance = numericalParameterTolerance(firstParameter, lastParameter);
        MYBREP_ASSERT_MESSAGE(std::fabs(lastParameter - firstParameter) <= curve.period() + parameterTolerance,
                              "CurveCurve2DIntersection periodic finite-use interval must not exceed one curve period.");
    }
}

// 将参数换算到当前有限有向区间上的规范化[0,1]参数。
double normalizedParameter(double parameter, double firstParameter, double lastParameter)
{
    return (parameter - firstParameter) / (lastParameter - firstParameter);
}

// 将非周期参数解析到有限区间内，并允许参数在容差内落于端点外侧。
bool resolveLinearParameter(double parameter, double firstParameter, double lastParameter, double parameterTolerance, double& resolvedParameter)
{
    const double minimum = (std::min)(firstParameter, lastParameter);
    const double maximum = (std::max)(firstParameter, lastParameter);

    if (parameter < minimum - parameterTolerance || parameter > maximum + parameterTolerance)
    {
        return false;
    }

    resolvedParameter = parameter;

    if (resolvedParameter < minimum)
    {
        resolvedParameter = minimum;
    }
    else if (resolvedParameter > maximum)
    {
        resolvedParameter = maximum;
    }

    return true;
}

// 将标准周期参数平移整数个period后解析到指定有限周期区间。
bool resolvePeriodicParameter(double parameter, double period, double firstParameter, double lastParameter,
                              double parameterTolerance, double& resolvedParameter)
{
    const double minimum = (std::min)(firstParameter, lastParameter);
    const double maximum = (std::max)(firstParameter, lastParameter);
    const double shiftCount = std::floor((minimum - parameter) / period);
    double candidate = parameter + shiftCount * period;

    while (candidate < minimum - parameterTolerance)
    {
        candidate += period;
    }

    while (candidate > maximum + parameterTolerance)
    {
        candidate -= period;
    }

    if (candidate < minimum - parameterTolerance || candidate > maximum + parameterTolerance)
    {
        return false;
    }

    resolvedParameter = candidate;

    if (resolvedParameter < minimum)
    {
        resolvedParameter = minimum;
    }
    else if (resolvedParameter > maximum)
    {
        resolvedParameter = maximum;
    }

    return true;
}

// 返回点在二维直线完整参数化中的参数。
double lineParameterAtPoint(const MyBRep::Geometry_Line2D& line, const MyMath::Vector2& point)
{
    return MyMath::Vector2::dot(point - line.origin(), line.direction());
}

// 返回点在二维圆完整参数化中的标准周期参数。
double circleParameterAtPoint(const MyBRep::Geometry_Circle2D& circle, const MyMath::Vector2& point)
{
    const MyMath::Vector2 relative = point - circle.center();
    return std::atan2(MyMath::Vector2::dot(relative, circle.yDir()), MyMath::Vector2::dot(relative, circle.xDir()));
}

// 返回规范化到[0,2π)的世界极角。
double normalizeAngle(double angle)
{
    double result = std::fmod(angle, TwoPi);

    if (result < 0.0)
    {
        result += TwoPi;
    }

    return result;
}

// 将有限圆弧转换为0到2π上的一个或两个无向覆盖区间。
std::vector<std::pair<double, double> > circleArcRanges(const MyBRep::Geometry_Circle2D& circle, double firstParameter,
                                                        double lastParameter, double angularTolerance)
{
    std::vector<std::pair<double, double> > result;
    const double length = std::fabs(lastParameter - firstParameter);

    if (length >= TwoPi - angularTolerance)
    {
        result.push_back(std::make_pair(0.0, TwoPi));
        return result;
    }

    const double baseAngle = std::atan2(circle.xDir().y(), circle.xDir().x());
    const double worldSweep = circle.orientationSign() * (lastParameter - firstParameter);
    const double startParameter = worldSweep >= 0.0 ? firstParameter : lastParameter;
    const double startAngle = normalizeAngle(baseAngle + circle.orientationSign() * startParameter);
    const double endAngle = startAngle + length;

    if (endAngle <= TwoPi)
    {
        result.push_back(std::make_pair(startAngle, endAngle));
        return result;
    }

    result.push_back(std::make_pair(startAngle, TwoPi));
    result.push_back(std::make_pair(0.0, endAngle - TwoPi));
    return result;
}

// 判断两个二维点是否在几何容差内重合。
bool pointsNear(const MyMath::Vector2& first, const MyMath::Vector2& second, double tolerance)
{
    return vectorLength(first - second) <= tolerance;
}

// 向结果中增加离散交点，并去除同一几何交点的重复解。
void addPoint(MyBRep::Operation::Intersection::CurveCurve2DIntersectionResult& result, const MyMath::Vector2& point,
              double firstParameter, double secondParameter, double firstStartParameter, double firstEndParameter,
              double secondStartParameter, double secondEndParameter, double tolerance)
{
    for (std::size_t index = 0; index < result.points.size(); ++index)
    {
        if (pointsNear(result.points[index].point, point, tolerance))
        {
            return;
        }
    }

    MyBRep::Operation::Intersection::CurveCurve2DIntersectionPoint intersection;
    intersection.point = point;
    intersection.firstParameter = firstParameter;
    intersection.secondParameter = secondParameter;
    intersection.firstNormalizedParameter = normalizedParameter(firstParameter, firstStartParameter, firstEndParameter);
    intersection.secondNormalizedParameter = normalizedParameter(secondParameter, secondStartParameter, secondEndParameter);
    result.points.push_back(intersection);
}

// 按第一曲线当前有限使用方向排序离散交点。
void finalizePointResult(MyBRep::Operation::Intersection::CurveCurve2DIntersectionResult& result)
{
    if (result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Overlap)
    {
        result.points.clear();
        return;
    }

    if (result.points.empty())
    {
        result.kind = MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::None;
        return;
    }

    std::sort(result.points.begin(), result.points.end(),
              [](const MyBRep::Operation::Intersection::CurveCurve2DIntersectionPoint& first,
                 const MyBRep::Operation::Intersection::CurveCurve2DIntersectionPoint& second)
              {
                  return first.firstNormalizedParameter < second.firstNormalizedParameter;
              });

    result.kind = MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Points;
}

// 求两条有限二维直线使用的交集。
MyBRep::Operation::Intersection::CurveCurve2DIntersectionResult intersectLineLine(
    const MyBRep::Geometry_Line2D& firstLine, double firstStartParameter, double firstEndParameter,
    const MyBRep::Geometry_Line2D& secondLine, double secondStartParameter, double secondEndParameter, double tolerance)
{
    using namespace MyBRep::Operation::Intersection;

    CurveCurve2DIntersectionResult result;
    const MyMath::Vector2 offset = secondLine.origin() - firstLine.origin();
    const double denominator = MyMath::Vector2::cross(firstLine.direction(), secondLine.direction());
    const double directionTolerance = (std::numeric_limits<double>::epsilon)() * NumericalScale;
    const double firstParameterTolerance = curveParameterTolerance(firstLine, firstStartParameter, firstEndParameter, tolerance);
    const double secondParameterTolerance = curveParameterTolerance(secondLine, secondStartParameter, secondEndParameter, tolerance);

    if (std::fabs(denominator) <= directionTolerance)
    {
        const double lineDistance = std::fabs(MyMath::Vector2::cross(offset, firstLine.direction()));

        if (lineDistance > tolerance)
        {
            return result;
        }

        const MyMath::Vector2 secondStartPoint = secondLine.pointAt(secondStartParameter);
        const MyMath::Vector2 secondEndPoint = secondLine.pointAt(secondEndParameter);
        const double secondOnFirstStart = lineParameterAtPoint(firstLine, secondStartPoint);
        const double secondOnFirstEnd = lineParameterAtPoint(firstLine, secondEndPoint);
        const double overlapMinimum = (std::max)((std::min)(firstStartParameter, firstEndParameter),
                                                 (std::min)(secondOnFirstStart, secondOnFirstEnd));
        const double overlapMaximum = (std::min)((std::max)(firstStartParameter, firstEndParameter),
                                                 (std::max)(secondOnFirstStart, secondOnFirstEnd));

        if (overlapMaximum < overlapMinimum - firstParameterTolerance)
        {
            return result;
        }

        if (overlapMaximum - overlapMinimum > firstParameterTolerance)
        {
            result.kind = CurveCurve2DIntersectionKind::Overlap;
            return result;
        }

        const double firstParameter = (overlapMinimum + overlapMaximum) * 0.5;
        const MyMath::Vector2 point = firstLine.pointAt(firstParameter);
        double secondParameter = lineParameterAtPoint(secondLine, point);

        if (!resolveLinearParameter(secondParameter, secondStartParameter, secondEndParameter, secondParameterTolerance, secondParameter))
        {
            return CurveCurve2DIntersectionResult();
        }

        addPoint(result, point, firstParameter, secondParameter, firstStartParameter, firstEndParameter,
                 secondStartParameter, secondEndParameter, tolerance);
        finalizePointResult(result);
        return result;
    }

    const double firstCandidate = MyMath::Vector2::cross(offset, secondLine.direction()) / denominator;
    const double secondCandidate = MyMath::Vector2::cross(offset, firstLine.direction()) / denominator;
    double firstParameter = 0.0;
    double secondParameter = 0.0;

    if (!resolveLinearParameter(firstCandidate, firstStartParameter, firstEndParameter, firstParameterTolerance, firstParameter) ||
        !resolveLinearParameter(secondCandidate, secondStartParameter, secondEndParameter, secondParameterTolerance, secondParameter))
    {
        return result;
    }

    const MyMath::Vector2 firstPoint = firstLine.pointAt(firstParameter);
    const MyMath::Vector2 secondPoint = secondLine.pointAt(secondParameter);

    if (!pointsNear(firstPoint, secondPoint, tolerance))
    {
        return result;
    }

    addPoint(result, (firstPoint + secondPoint) * 0.5, firstParameter, secondParameter, firstStartParameter, firstEndParameter,
             secondStartParameter, secondEndParameter, tolerance);
    finalizePointResult(result);
    return result;
}

// 求有限二维直线使用与有限二维圆使用的交集。
MyBRep::Operation::Intersection::CurveCurve2DIntersectionResult intersectLineCircle(
    const MyBRep::Geometry_Line2D& line, double lineStartParameter, double lineEndParameter,
    const MyBRep::Geometry_Circle2D& circle, double circleStartParameter, double circleEndParameter,
    double tolerance, bool lineIsFirst)
{
    using namespace MyBRep::Operation::Intersection;

    CurveCurve2DIntersectionResult result;
    const MyMath::Vector2 offset = line.origin() - circle.center();
    const double projection = MyMath::Vector2::dot(offset, line.direction());
    const double constant = MyMath::Vector2::dot(offset, offset) - circle.radius() * circle.radius();
    double discriminant = projection * projection - constant;

    const double scale = (std::max)(1.0, (std::max)(circle.radius(), vectorLength(offset)));
    const double discriminantTolerance = (std::max)(scale * scale * (std::numeric_limits<double>::epsilon)() * NumericalScale,
                                                    tolerance * scale * 2.0);
    const double lineParameterTolerance = curveParameterTolerance(line, lineStartParameter, lineEndParameter, tolerance);
    const double circleParameterTolerance = curveParameterTolerance(circle, circleStartParameter, circleEndParameter, tolerance);

    if (discriminant < -discriminantTolerance)
    {
        return result;
    }

    if (discriminant < 0.0)
    {
        discriminant = 0.0;
    }

    const double root = std::sqrt(discriminant);
    const double candidates[2] = { -projection - root, -projection + root };
    const int candidateCount = root <= lineParameterTolerance ? 1 : 2;

    for (int index = 0; index < candidateCount; ++index)
    {
        double lineParameter = 0.0;

        if (!resolveLinearParameter(candidates[index], lineStartParameter, lineEndParameter, lineParameterTolerance, lineParameter))
        {
            continue;
        }

        const MyMath::Vector2 point = line.pointAt(lineParameter);
        const double standardCircleParameter = circleParameterAtPoint(circle, point);
        double circleParameter = 0.0;

        if (!resolvePeriodicParameter(standardCircleParameter, circle.period(), circleStartParameter, circleEndParameter,
                                      circleParameterTolerance, circleParameter))
        {
            continue;
        }

        if (!pointsNear(point, circle.pointAt(circleParameter), tolerance))
        {
            continue;
        }

        if (lineIsFirst)
        {
            addPoint(result, point, lineParameter, circleParameter, lineStartParameter, lineEndParameter,
                     circleStartParameter, circleEndParameter, tolerance);
        }
        else
        {
            addPoint(result, point, circleParameter, lineParameter, circleStartParameter, circleEndParameter,
                     lineStartParameter, lineEndParameter, tolerance);
        }
    }

    finalizePointResult(result);
    return result;
}

// 求两个有限重合圆弧使用的交集类型及离散端点接触。
MyBRep::Operation::Intersection::CurveCurve2DIntersectionResult intersectCoincidentCircles(
    const MyBRep::Geometry_Circle2D& firstCircle, double firstStartParameter, double firstEndParameter,
    const MyBRep::Geometry_Circle2D& secondCircle, double secondStartParameter, double secondEndParameter, double tolerance)
{
    using namespace MyBRep::Operation::Intersection;

    CurveCurve2DIntersectionResult result;
    const double firstParameterTolerance = curveParameterTolerance(firstCircle, firstStartParameter, firstEndParameter, tolerance);
    const double secondParameterTolerance = curveParameterTolerance(secondCircle, secondStartParameter, secondEndParameter, tolerance);
    const double angularTolerance = (std::max)(firstParameterTolerance, secondParameterTolerance);
    const std::vector<std::pair<double, double> > firstRanges =
        circleArcRanges(firstCircle, firstStartParameter, firstEndParameter, angularTolerance);
    const std::vector<std::pair<double, double> > secondRanges =
        circleArcRanges(secondCircle, secondStartParameter, secondEndParameter, angularTolerance);

    for (std::size_t firstIndex = 0; firstIndex < firstRanges.size(); ++firstIndex)
    {
        for (std::size_t secondIndex = 0; secondIndex < secondRanges.size(); ++secondIndex)
        {
            const double overlapStart = (std::max)(firstRanges[firstIndex].first, secondRanges[secondIndex].first);
            const double overlapEnd = (std::min)(firstRanges[firstIndex].second, secondRanges[secondIndex].second);

            if (overlapEnd < overlapStart - angularTolerance)
            {
                continue;
            }

            if (overlapEnd - overlapStart > angularTolerance)
            {
                result.kind = CurveCurve2DIntersectionKind::Overlap;
                result.points.clear();
                return result;
            }

            const double angle = normalizeAngle((overlapStart + overlapEnd) * 0.5);
            const MyMath::Vector2 point = firstCircle.center() +
                MyMath::Vector2(std::cos(angle), std::sin(angle)) * firstCircle.radius();
            double firstParameter = 0.0;
            double secondParameter = 0.0;

            if (!resolvePeriodicParameter(circleParameterAtPoint(firstCircle, point), firstCircle.period(),
                                          firstStartParameter, firstEndParameter, firstParameterTolerance, firstParameter) ||
                !resolvePeriodicParameter(circleParameterAtPoint(secondCircle, point), secondCircle.period(),
                                          secondStartParameter, secondEndParameter, secondParameterTolerance, secondParameter))
            {
                continue;
            }

            addPoint(result, point, firstParameter, secondParameter, firstStartParameter, firstEndParameter,
                     secondStartParameter, secondEndParameter, tolerance);
        }
    }

    finalizePointResult(result);
    return result;
}

// 求两个有限二维圆使用的交集。
MyBRep::Operation::Intersection::CurveCurve2DIntersectionResult intersectCircleCircle(
    const MyBRep::Geometry_Circle2D& firstCircle, double firstStartParameter, double firstEndParameter,
    const MyBRep::Geometry_Circle2D& secondCircle, double secondStartParameter, double secondEndParameter, double tolerance)
{
    using namespace MyBRep::Operation::Intersection;

    CurveCurve2DIntersectionResult result;
    const MyMath::Vector2 centerDelta = secondCircle.center() - firstCircle.center();
    const double centerDistance = vectorLength(centerDelta);
    const double radiusDifference = std::fabs(firstCircle.radius() - secondCircle.radius());

    if (centerDistance <= tolerance && radiusDifference <= tolerance)
    {
        return intersectCoincidentCircles(firstCircle, firstStartParameter, firstEndParameter,
                                          secondCircle, secondStartParameter, secondEndParameter, tolerance);
    }

    if (centerDistance <= tolerance)
    {
        return result;
    }

    const double radiusSum = firstCircle.radius() + secondCircle.radius();
    const double radiusDelta = std::fabs(firstCircle.radius() - secondCircle.radius());

    if (centerDistance > radiusSum + tolerance || centerDistance < radiusDelta - tolerance)
    {
        return result;
    }

    const double along = (firstCircle.radius() * firstCircle.radius() - secondCircle.radius() * secondCircle.radius() +
                          centerDistance * centerDistance) / (2.0 * centerDistance);
    double heightSquared = firstCircle.radius() * firstCircle.radius() - along * along;
    const double scale = (std::max)(1.0, (std::max)(firstCircle.radius(), secondCircle.radius()));
    const double squaredTolerance = (std::max)(scale * scale * (std::numeric_limits<double>::epsilon)() * NumericalScale,
                                              tolerance * scale * 2.0);

    if (heightSquared < -squaredTolerance)
    {
        return result;
    }

    if (heightSquared < 0.0)
    {
        heightSquared = 0.0;
    }

    const MyMath::Vector2 direction = centerDelta / centerDistance;
    const MyMath::Vector2 basePoint = firstCircle.center() + direction * along;
    const MyMath::Vector2 perpendicular(-direction.y(), direction.x());
    const double height = std::sqrt(heightSquared);
    const MyMath::Vector2 candidates[2] = { basePoint + perpendicular * height, basePoint - perpendicular * height };
    const int candidateCount = height <= tolerance ? 1 : 2;
    const double firstParameterTolerance = curveParameterTolerance(firstCircle, firstStartParameter, firstEndParameter, tolerance);
    const double secondParameterTolerance = curveParameterTolerance(secondCircle, secondStartParameter, secondEndParameter, tolerance);

    for (int index = 0; index < candidateCount; ++index)
    {
        double firstParameter = 0.0;
        double secondParameter = 0.0;

        if (!resolvePeriodicParameter(circleParameterAtPoint(firstCircle, candidates[index]), firstCircle.period(),
                                      firstStartParameter, firstEndParameter, firstParameterTolerance, firstParameter) ||
            !resolvePeriodicParameter(circleParameterAtPoint(secondCircle, candidates[index]), secondCircle.period(),
                                      secondStartParameter, secondEndParameter, secondParameterTolerance, secondParameter))
        {
            continue;
        }

        if (!pointsNear(firstCircle.pointAt(firstParameter), secondCircle.pointAt(secondParameter), tolerance))
        {
            continue;
        }

        addPoint(result, candidates[index], firstParameter, secondParameter, firstStartParameter, firstEndParameter,
                 secondStartParameter, secondEndParameter, tolerance);
    }

    finalizePointResult(result);
    return result;
}

}

namespace MyBRep
{
namespace Operation
{
namespace Intersection
{

CurveCurve2DIntersectionResult::CurveCurve2DIntersectionResult()
    : kind(CurveCurve2DIntersectionKind::None)
{
}

CurveCurve2DIntersectionResult intersectCurveCurve2D(const Geometry_Curve2D& firstCurve, double firstStartParameter, double firstEndParameter,
                                                      const Geometry_Curve2D& secondCurve, double secondStartParameter, double secondEndParameter,
                                                      double tolerance)
{
    MYBREP_ASSERT_MESSAGE(isFiniteNonNegative(tolerance), "CurveCurve2DIntersection tolerance must be finite and non-negative.");
    validateCurveUse(firstCurve, firstStartParameter, firstEndParameter);
    validateCurveUse(secondCurve, secondStartParameter, secondEndParameter);

    MYBREP_ASSERT_MESSAGE((firstCurve.kind() == CurveKind::Line || firstCurve.kind() == CurveKind::Circle ||
                           firstCurve.kind() == CurveKind::Bezier || firstCurve.kind() == CurveKind::BSpline) &&
                          (secondCurve.kind() == CurveKind::Line || secondCurve.kind() == CurveKind::Circle ||
                           secondCurve.kind() == CurveKind::Bezier || secondCurve.kind() == CurveKind::BSpline),
                          "CurveCurve2DIntersection does not support the current curve kind.");

    const bool firstAnalytic = firstCurve.kind() == CurveKind::Line || firstCurve.kind() == CurveKind::Circle;
    const bool secondAnalytic = secondCurve.kind() == CurveKind::Line || secondCurve.kind() == CurveKind::Circle;

    if (!firstAnalytic || !secondAnalytic)
    {
        return intersectGeneralCurveCurve2D(firstCurve, firstStartParameter, firstEndParameter,
                                            secondCurve, secondStartParameter, secondEndParameter, tolerance);
    }

    if (firstCurve.kind() == CurveKind::Line && secondCurve.kind() == CurveKind::Line)
    {
        return intersectLineLine(static_cast<const Geometry_Line2D&>(firstCurve), firstStartParameter, firstEndParameter,
                                 static_cast<const Geometry_Line2D&>(secondCurve), secondStartParameter, secondEndParameter, tolerance);
    }

    if (firstCurve.kind() == CurveKind::Line && secondCurve.kind() == CurveKind::Circle)
    {
        return intersectLineCircle(static_cast<const Geometry_Line2D&>(firstCurve), firstStartParameter, firstEndParameter,
                                   static_cast<const Geometry_Circle2D&>(secondCurve), secondStartParameter, secondEndParameter, tolerance, true);
    }

    if (firstCurve.kind() == CurveKind::Circle && secondCurve.kind() == CurveKind::Line)
    {
        return intersectLineCircle(static_cast<const Geometry_Line2D&>(secondCurve), secondStartParameter, secondEndParameter,
                                   static_cast<const Geometry_Circle2D&>(firstCurve), firstStartParameter, firstEndParameter, tolerance, false);
    }

    return intersectCircleCircle(static_cast<const Geometry_Circle2D&>(firstCurve), firstStartParameter, firstEndParameter,
                                 static_cast<const Geometry_Circle2D&>(secondCurve), secondStartParameter, secondEndParameter, tolerance);
}

}
}
}
