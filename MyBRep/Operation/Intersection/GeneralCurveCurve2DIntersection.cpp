#include "GeneralCurveCurve2DIntersection.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "MyBRep/Foundation/Diagnostic.h"
#include "MyBRep/Geometry/Curve2D/Geometry_BSpline2D.h"

namespace
{

const std::size_t MinimumSubdivisionDepth = 3; // 每个基础参数段至少细分为8段，避免仅凭平坦度漏掉局部弯折。
const std::size_t MaximumSubdivisionDepth = 12; // 通用候选折线单个基础参数段最多递归12层，限制病态输入成本。
const std::size_t MaximumRefinementIterations = 32; // 单个候选点局部数值精化最多执行32次。
const double ApproximationScale = 1.0e-4; // 候选折线允许的尺度相对弦偏差，仅用于搜索而不作为最终精度。
const double CandidateToleranceScale = 2.0; // 候选段包围盒扩张为折线近似容差的2倍，降低切触点漏检概率。
const double DerivativeStepScale = 1.0e-6; // 数值一阶导数步长相对当前有限参数区间使用1e-6。
const double DuplicateParameterScale = 64.0; // 参数去重覆盖浮点归一化误差使用的固定倍数。

struct PolylinePoint
{
    double parameter;
    MyMath::Vector2 point;
};

struct Candidate
{
    double firstParameter;
    double secondParameter;
    double firstMinimum;
    double firstMaximum;
    double secondMinimum;
    double secondMaximum;
};

// 判断标量是否为有限值。
bool isFiniteValue(double value)
{
    const double infinity = (std::numeric_limits<double>::infinity)();
    return value == value && value != infinity && value != -infinity;
}

// 返回二维点到有限线段的距离。
double pointSegmentDistance(const MyMath::Vector2& point, const MyMath::Vector2& start, const MyMath::Vector2& end)
{
    const MyMath::Vector2 segment = end - start;
    const double lengthSquared = MyMath::Vector2::dot(segment, segment);

    if (lengthSquared == 0.0)
    {
        return (point - start).length();
    }

    double parameter = MyMath::Vector2::dot(point - start, segment) / lengthSquared;
    parameter = (std::max)(0.0, (std::min)(1.0, parameter));
    return (point - (start + segment * parameter)).length();
}

// 返回当前两条有限曲线使用的坐标尺度。
double curveUseScale(const MyBRep::Geometry_Curve2D& curve, double firstParameter, double lastParameter)
{
    double scale = 1.0;

    for (int index = 0; index <= 16; ++index)
    {
        const double ratio = static_cast<double>(index) / 16.0;
        const double parameter = firstParameter + (lastParameter - firstParameter) * ratio;
        const MyMath::Vector2 point = curve.pointAt(parameter);

        scale = (std::max)(scale, std::fabs(point.x()));
        scale = (std::max)(scale, std::fabs(point.y()));
    }

    return scale;
}

// 返回候选折线构造使用的几何近似容差。
double approximationTolerance(const MyBRep::Geometry_Curve2D& firstCurve, double firstStartParameter, double firstEndParameter,
                              const MyBRep::Geometry_Curve2D& secondCurve, double secondStartParameter, double secondEndParameter, double tolerance)
{
    const double scale = (std::max)(curveUseScale(firstCurve, firstStartParameter, firstEndParameter),
                                    curveUseScale(secondCurve, secondStartParameter, secondEndParameter));
    return (std::max)(tolerance * 16.0, scale * ApproximationScale);
}

// 返回参数值在有限区间内的限制结果。
double clampParameter(double parameter, double firstParameter, double lastParameter)
{
    const double minimum = (std::min)(firstParameter, lastParameter);
    const double maximum = (std::max)(firstParameter, lastParameter);
    return (std::max)(minimum, (std::min)(maximum, parameter));
}

// 返回曲线在指定参数处的安全数值一阶导数，不依赖BSpline内部节点的解析导数存在性。
MyMath::Vector2 numericalDerivative(const MyBRep::Geometry_Curve2D& curve, double parameter, double firstParameter, double lastParameter)
{
    const double parameterScale = (std::max)(1.0, std::fabs(lastParameter - firstParameter));
    const double machineStep = (std::sqrt)((std::numeric_limits<double>::epsilon)()) * (std::max)(1.0, std::fabs(parameter));
    const double requestedStep = (std::max)(parameterScale * DerivativeStepScale, machineStep);
    const double before = clampParameter(parameter - requestedStep, firstParameter, lastParameter);
    const double after = clampParameter(parameter + requestedStep, firstParameter, lastParameter);

    if (after == before)
    {
        return MyMath::Vector2::zero();
    }

    return (curve.pointAt(after) - curve.pointAt(before)) / (after - before);
}

// 返回指定参数段的折线平坦度估计，使用1/4、1/2、3/4三个内部采样点避免单中点漏掉对称弯折。
double intervalFlatness(const MyBRep::Geometry_Curve2D& curve, double firstParameter, double lastParameter,
                        const MyMath::Vector2& firstPoint, const MyMath::Vector2& lastPoint)
{
    double flatness = 0.0;

    for (int index = 1; index <= 3; ++index)
    {
        const double ratio = static_cast<double>(index) * 0.25;
        const double parameter = firstParameter + (lastParameter - firstParameter) * ratio;
        flatness = (std::max)(flatness, pointSegmentDistance(curve.pointAt(parameter), firstPoint, lastPoint));
    }

    return flatness;
}

// 递归构造一段有限曲线使用的候选搜索折线。
void appendAdaptivePolyline(const MyBRep::Geometry_Curve2D& curve, double firstParameter, double lastParameter,
                            const MyMath::Vector2& firstPoint, const MyMath::Vector2& lastPoint, double tolerance,
                            std::size_t depth, std::vector<PolylinePoint>& points)
{
    const double flatness = intervalFlatness(curve, firstParameter, lastParameter, firstPoint, lastPoint);

    if (depth >= MaximumSubdivisionDepth || (depth >= MinimumSubdivisionDepth && flatness <= tolerance))
    {
        PolylinePoint point;
        point.parameter = lastParameter;
        point.point = lastPoint;
        points.push_back(point);
        return;
    }

    const double middleParameter = (firstParameter + lastParameter) * 0.5;
    const MyMath::Vector2 middlePoint = curve.pointAt(middleParameter);

    appendAdaptivePolyline(curve, firstParameter, middleParameter, firstPoint, middlePoint, tolerance, depth + 1, points);
    appendAdaptivePolyline(curve, middleParameter, lastParameter, middlePoint, lastPoint, tolerance, depth + 1, points);
}

// 返回BSpline有限使用区间内按照当前方向排列的基础节点跨度边界，其他曲线只返回首尾参数。
std::vector<double> baseParameters(const MyBRep::Geometry_Curve2D& curve, double firstParameter, double lastParameter)
{
    std::vector<double> parameters;
    parameters.push_back(firstParameter);

    if (curve.kind() == MyBRep::CurveKind::BSpline)
    {
        const MyBRep::Geometry_BSpline2D& spline = static_cast<const MyBRep::Geometry_BSpline2D&>(curve);
        const double minimum = (std::min)(firstParameter, lastParameter);
        const double maximum = (std::max)(firstParameter, lastParameter);
        std::vector<double> internalKnots;

        for (std::size_t index = 0; index < spline.knotCount(); ++index)
        {
            const double knot = spline.knot(index);

            if (knot <= minimum || knot >= maximum)
            {
                continue;
            }

            if (internalKnots.empty() || knot != internalKnots.back())
            {
                internalKnots.push_back(knot);
            }
        }

        if (firstParameter < lastParameter)
        {
            parameters.insert(parameters.end(), internalKnots.begin(), internalKnots.end());
        }
        else
        {
            parameters.insert(parameters.end(), internalKnots.rbegin(), internalKnots.rend());
        }
    }

    parameters.push_back(lastParameter);
    return parameters;
}

// 构造完整有限曲线使用的候选搜索折线。
std::vector<PolylinePoint> buildPolyline(const MyBRep::Geometry_Curve2D& curve, double firstParameter, double lastParameter, double tolerance)
{
    const std::vector<double> parameters = baseParameters(curve, firstParameter, lastParameter);
    std::vector<PolylinePoint> result;

    PolylinePoint first;
    first.parameter = parameters.front();
    first.point = curve.pointAt(parameters.front());
    result.push_back(first);

    for (std::size_t index = 0; index + 1 < parameters.size(); ++index)
    {
        const double startParameter = parameters[index];
        const double endParameter = parameters[index + 1];
        const MyMath::Vector2 startPoint = curve.pointAt(startParameter);
        const MyMath::Vector2 endPoint = curve.pointAt(endParameter);

        appendAdaptivePolyline(curve, startParameter, endParameter, startPoint, endPoint, tolerance, 0, result);
    }

    return result;
}

// 判断两个扩张后的二维线段包围盒是否重叠。
bool segmentBoundsIntersect(const MyMath::Vector2& firstStart, const MyMath::Vector2& firstEnd,
                            const MyMath::Vector2& secondStart, const MyMath::Vector2& secondEnd, double tolerance)
{
    const double firstMinimumX = (std::min)(firstStart.x(), firstEnd.x()) - tolerance;
    const double firstMaximumX = (std::max)(firstStart.x(), firstEnd.x()) + tolerance;
    const double firstMinimumY = (std::min)(firstStart.y(), firstEnd.y()) - tolerance;
    const double firstMaximumY = (std::max)(firstStart.y(), firstEnd.y()) + tolerance;
    const double secondMinimumX = (std::min)(secondStart.x(), secondEnd.x()) - tolerance;
    const double secondMaximumX = (std::max)(secondStart.x(), secondEnd.x()) + tolerance;
    const double secondMinimumY = (std::min)(secondStart.y(), secondEnd.y()) - tolerance;
    const double secondMaximumY = (std::max)(secondStart.y(), secondEnd.y()) + tolerance;

    return firstMaximumX >= secondMinimumX && secondMaximumX >= firstMinimumX &&
           firstMaximumY >= secondMinimumY && secondMaximumY >= firstMinimumY;
}

// 使用两条候选折线段求一个初始参数对；非平行时使用无限直线交点，退化时使用参数中点。
Candidate makeCandidate(const PolylinePoint& firstStart, const PolylinePoint& firstEnd,
                        const PolylinePoint& secondStart, const PolylinePoint& secondEnd)
{
    Candidate candidate;
    candidate.firstMinimum = (std::min)(firstStart.parameter, firstEnd.parameter);
    candidate.firstMaximum = (std::max)(firstStart.parameter, firstEnd.parameter);
    candidate.secondMinimum = (std::min)(secondStart.parameter, secondEnd.parameter);
    candidate.secondMaximum = (std::max)(secondStart.parameter, secondEnd.parameter);

    const MyMath::Vector2 firstDirection = firstEnd.point - firstStart.point;
    const MyMath::Vector2 secondDirection = secondEnd.point - secondStart.point;
    const MyMath::Vector2 offset = secondStart.point - firstStart.point;
    const double denominator = MyMath::Vector2::cross(firstDirection, secondDirection);

    if (std::fabs(denominator) > (std::numeric_limits<double>::epsilon)() * 64.0)
    {
        const double firstRatio = MyMath::Vector2::cross(offset, secondDirection) / denominator;
        const double secondRatio = MyMath::Vector2::cross(offset, firstDirection) / denominator;

        candidate.firstParameter = firstStart.parameter + (firstEnd.parameter - firstStart.parameter) * (std::max)(0.0, (std::min)(1.0, firstRatio));
        candidate.secondParameter = secondStart.parameter + (secondEnd.parameter - secondStart.parameter) * (std::max)(0.0, (std::min)(1.0, secondRatio));
    }
    else
    {
        candidate.firstParameter = (firstStart.parameter + firstEnd.parameter) * 0.5;
        candidate.secondParameter = (secondStart.parameter + secondEnd.parameter) * 0.5;
    }

    return candidate;
}

// 返回两个当前参数对应曲线点之间的距离。
double curveDistance(const MyBRep::Geometry_Curve2D& firstCurve, double firstParameter,
                     const MyBRep::Geometry_Curve2D& secondCurve, double secondParameter)
{
    return (firstCurve.pointAt(firstParameter) - secondCurve.pointAt(secondParameter)).length();
}

// 使用阻尼Gauss-Newton精化候选参数；解析导数不可靠的节点由安全数值导数替代。
void refineCandidate(const MyBRep::Geometry_Curve2D& firstCurve, const MyBRep::Geometry_Curve2D& secondCurve,
                     Candidate& candidate, double tolerance)
{
    double firstParameter = candidate.firstParameter;
    double secondParameter = candidate.secondParameter;
    double bestDistance = curveDistance(firstCurve, firstParameter, secondCurve, secondParameter);

    for (std::size_t iteration = 0; iteration < MaximumRefinementIterations; ++iteration)
    {
        if (bestDistance <= tolerance)
        {
            break;
        }

        const MyMath::Vector2 firstPoint = firstCurve.pointAt(firstParameter);
        const MyMath::Vector2 secondPoint = secondCurve.pointAt(secondParameter);
        const MyMath::Vector2 residual = firstPoint - secondPoint;
        const MyMath::Vector2 firstDerivative = numericalDerivative(firstCurve, firstParameter, candidate.firstMinimum, candidate.firstMaximum);
        const MyMath::Vector2 secondDerivative = numericalDerivative(secondCurve, secondParameter, candidate.secondMinimum, candidate.secondMaximum);

        const double a = MyMath::Vector2::dot(firstDerivative, firstDerivative);
        const double b = -MyMath::Vector2::dot(firstDerivative, secondDerivative);
        const double c = MyMath::Vector2::dot(secondDerivative, secondDerivative);
        const double rhsFirst = -MyMath::Vector2::dot(firstDerivative, residual);
        const double rhsSecond = MyMath::Vector2::dot(secondDerivative, residual);
        const double damping = (a + c + 1.0) * 1.0e-12;
        const double aa = a + damping;
        const double cc = c + damping;
        const double determinant = aa * cc - b * b;

        if (determinant <= 0.0)
        {
            break;
        }

        const double firstDelta = (rhsFirst * cc - b * rhsSecond) / determinant;
        const double secondDelta = (aa * rhsSecond - b * rhsFirst) / determinant;
        const double nextFirst = (std::max)(candidate.firstMinimum, (std::min)(candidate.firstMaximum, firstParameter + firstDelta));
        const double nextSecond = (std::max)(candidate.secondMinimum, (std::min)(candidate.secondMaximum, secondParameter + secondDelta));
        const double nextDistance = curveDistance(firstCurve, nextFirst, secondCurve, nextSecond);

        if (nextDistance < bestDistance)
        {
            firstParameter = nextFirst;
            secondParameter = nextSecond;
            bestDistance = nextDistance;
            continue;
        }

        const double halfFirst = firstDelta * 0.5;
        const double halfSecond = secondDelta * 0.5;
        const double dampedFirst = (std::max)(candidate.firstMinimum, (std::min)(candidate.firstMaximum, firstParameter + halfFirst));
        const double dampedSecond = (std::max)(candidate.secondMinimum, (std::min)(candidate.secondMaximum, secondParameter + halfSecond));
        const double dampedDistance = curveDistance(firstCurve, dampedFirst, secondCurve, dampedSecond);

        if (dampedDistance >= bestDistance)
        {
            break;
        }

        firstParameter = dampedFirst;
        secondParameter = dampedSecond;
        bestDistance = dampedDistance;
    }

    // 切触或近奇异Jacobian时使用局部模式搜索继续收缩参数矩形。
    double firstStep = (candidate.firstMaximum - candidate.firstMinimum) * 0.25;
    double secondStep = (candidate.secondMaximum - candidate.secondMinimum) * 0.25;

    for (std::size_t iteration = 0; iteration < MaximumRefinementIterations && bestDistance > tolerance; ++iteration)
    {
        bool improved = false;
        double bestFirst = firstParameter;
        double bestSecond = secondParameter;

        for (int firstOffset = -1; firstOffset <= 1; ++firstOffset)
        {
            for (int secondOffset = -1; secondOffset <= 1; ++secondOffset)
            {
                if (firstOffset == 0 && secondOffset == 0)
                {
                    continue;
                }

                const double candidateFirst = (std::max)(candidate.firstMinimum, (std::min)(candidate.firstMaximum, firstParameter + firstStep * firstOffset));
                const double candidateSecond = (std::max)(candidate.secondMinimum, (std::min)(candidate.secondMaximum, secondParameter + secondStep * secondOffset));
                const double distance = curveDistance(firstCurve, candidateFirst, secondCurve, candidateSecond);

                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    bestFirst = candidateFirst;
                    bestSecond = candidateSecond;
                    improved = true;
                }
            }
        }

        firstParameter = bestFirst;
        secondParameter = bestSecond;

        if (!improved)
        {
            firstStep *= 0.5;
            secondStep *= 0.5;
        }
    }

    candidate.firstParameter = firstParameter;
    candidate.secondParameter = secondParameter;
}

// 向最终结果增加去重后的离散交点。
void addIntersectionPoint(MyBRep::Operation::Intersection::CurveCurve2DIntersectionResult& result,
                          const MyBRep::Geometry_Curve2D& firstCurve, double firstParameter, double firstStartParameter, double firstEndParameter,
                          const MyBRep::Geometry_Curve2D& secondCurve, double secondParameter, double secondStartParameter, double secondEndParameter,
                          double tolerance)
{
    const MyMath::Vector2 firstPoint = firstCurve.pointAt(firstParameter);
    const MyMath::Vector2 secondPoint = secondCurve.pointAt(secondParameter);
    const MyMath::Vector2 point = (firstPoint + secondPoint) * 0.5;
    const double firstNormalized = (firstParameter - firstStartParameter) / (firstEndParameter - firstStartParameter);
    const double secondNormalized = (secondParameter - secondStartParameter) / (secondEndParameter - secondStartParameter);
    const double normalizedTolerance = (std::max)(tolerance, (std::numeric_limits<double>::epsilon)() * DuplicateParameterScale);

    for (std::size_t index = 0; index < result.points.size(); ++index)
    {
        if ((result.points[index].point - point).length() <= tolerance ||
            (std::fabs(result.points[index].firstNormalizedParameter - firstNormalized) <= normalizedTolerance &&
             std::fabs(result.points[index].secondNormalizedParameter - secondNormalized) <= normalizedTolerance))
        {
            return;
        }
    }

    MyBRep::Operation::Intersection::CurveCurve2DIntersectionPoint intersection;
    intersection.point = point;
    intersection.firstParameter = firstParameter;
    intersection.secondParameter = secondParameter;
    intersection.firstNormalizedParameter = firstNormalized;
    intersection.secondNormalizedParameter = secondNormalized;
    result.points.push_back(intersection);
}

// 处理同一非周期一般曲线资源的参数区间重合，避免将连续交集离散成大量点。
bool resolveSameCurveOverlap(const MyBRep::Geometry_Curve2D& firstCurve, double firstStartParameter, double firstEndParameter,
                             const MyBRep::Geometry_Curve2D& secondCurve, double secondStartParameter, double secondEndParameter,
                             double tolerance, MyBRep::Operation::Intersection::CurveCurve2DIntersectionResult& result)
{
    if (&firstCurve != &secondCurve || firstCurve.isPeriodic())
    {
        return false;
    }

    const double overlapMinimum = (std::max)((std::min)(firstStartParameter, firstEndParameter), (std::min)(secondStartParameter, secondEndParameter));
    const double overlapMaximum = (std::min)((std::max)(firstStartParameter, firstEndParameter), (std::max)(secondStartParameter, secondEndParameter));
    const double parameterTolerance = (std::max)(tolerance, (std::numeric_limits<double>::epsilon)() * DuplicateParameterScale);

    if (overlapMaximum < overlapMinimum - parameterTolerance)
    {
        return true;
    }

    if (overlapMaximum - overlapMinimum > parameterTolerance)
    {
        result.kind = MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Overlap;
        return true;
    }

    const double parameter = (overlapMinimum + overlapMaximum) * 0.5;
    addIntersectionPoint(result, firstCurve, parameter, firstStartParameter, firstEndParameter,
                         secondCurve, parameter, secondStartParameter, secondEndParameter, tolerance);
    result.kind = MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Points;
    return true;
}

}

namespace MyBRep
{
namespace Operation
{
namespace Intersection
{

CurveCurve2DIntersectionResult intersectGeneralCurveCurve2D(const Geometry_Curve2D& firstCurve, double firstStartParameter, double firstEndParameter,
                                                             const Geometry_Curve2D& secondCurve, double secondStartParameter, double secondEndParameter,
                                                             double tolerance)
{
    CurveCurve2DIntersectionResult result;

    if (resolveSameCurveOverlap(firstCurve, firstStartParameter, firstEndParameter,
                                secondCurve, secondStartParameter, secondEndParameter, tolerance, result))
    {
        return result;
    }

    const double approximation = approximationTolerance(firstCurve, firstStartParameter, firstEndParameter,
                                                        secondCurve, secondStartParameter, secondEndParameter, tolerance);
    const double candidateTolerance = approximation * CandidateToleranceScale;
    const std::vector<PolylinePoint> firstPolyline = buildPolyline(firstCurve, firstStartParameter, firstEndParameter, approximation);
    const std::vector<PolylinePoint> secondPolyline = buildPolyline(secondCurve, secondStartParameter, secondEndParameter, approximation);

    for (std::size_t firstIndex = 0; firstIndex + 1 < firstPolyline.size(); ++firstIndex)
    {
        for (std::size_t secondIndex = 0; secondIndex + 1 < secondPolyline.size(); ++secondIndex)
        {
            if (!segmentBoundsIntersect(firstPolyline[firstIndex].point, firstPolyline[firstIndex + 1].point,
                                        secondPolyline[secondIndex].point, secondPolyline[secondIndex + 1].point, candidateTolerance))
            {
                continue;
            }

            Candidate candidate = makeCandidate(firstPolyline[firstIndex], firstPolyline[firstIndex + 1],
                                                secondPolyline[secondIndex], secondPolyline[secondIndex + 1]);
            refineCandidate(firstCurve, secondCurve, candidate, tolerance);

            if (curveDistance(firstCurve, candidate.firstParameter, secondCurve, candidate.secondParameter) > tolerance)
            {
                continue;
            }

            addIntersectionPoint(result, firstCurve, candidate.firstParameter, firstStartParameter, firstEndParameter,
                                 secondCurve, candidate.secondParameter, secondStartParameter, secondEndParameter, tolerance);
        }
    }

    if (result.points.empty())
    {
        result.kind = CurveCurve2DIntersectionKind::None;
        return result;
    }

    std::sort(result.points.begin(), result.points.end(),
              [](const CurveCurve2DIntersectionPoint& first, const CurveCurve2DIntersectionPoint& second)
              {
                  return first.firstNormalizedParameter < second.firstNormalizedParameter;
              });

    result.kind = CurveCurve2DIntersectionKind::Points;
    return result;
}

}
}
}
