#include "Geometry_Revolved.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "MyBRep/Foundation/Diagnostic.h"
#include "MyBRep/Geometry/Curve/Geometry_Circle.h"
#include "MyBRep/Geometry/Curve/Geometry_Line.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 圆参数区间和圆弧交点计算统一使用弧度制。 const double TwoPi = Pi * 2.0; // 单个Circle母线段允许覆盖的最大周期。
    const double NumericalScale = 64.0; // 坐标、射线和参数边界判断覆盖舍入误差使用的固定倍数。

// 判断标量是否为有限值。
bool isFiniteValue(double value)
{
    const double infinity = (std::numeric_limits<double>::infinity)();

    return value == value && value != infinity && value != -infinity;
}

// 判断标量是否为有限非负数。
bool isFiniteNonNegative(double value)
{
    return isFiniteValue(value) && value >= 0.0;
}

// 判断半尺寸是否为有限非负数据。
bool isValidExtent(const MyMath::Vector3& extent)
{
    return extent.isFinite() && extent.x() >= 0.0 && extent.y() >= 0.0 && extent.z() >= 0.0;
}

// 将允许profileTolerance平面误差的三维点投影到严格局部XY母线平面。
MyMath::Vector3 projectedPoint(const MyMath::Vector3& point)
{
    return MyMath::Vector3(point.x(), point.y(), 0.0);
}

// 返回两个轮廓连接点使用的尺度自适应有效容差。
double effectivePointTolerance(const MyMath::Vector3& first, const MyMath::Vector3& second, double tolerance)
{
    double scale = 1.0;

    scale = (std::max)(scale, std::fabs(first.x()));
    scale = (std::max)(scale, std::fabs(first.y()));
    scale = (std::max)(scale, std::fabs(second.x()));
    scale = (std::max)(scale, std::fabs(second.y()));

    return (std::max)(tolerance, scale * (std::numeric_limits<double>::epsilon)() * NumericalScale);
}

// 返回二维叉积标量。
double cross2D(const MyMath::Vector3& first, const MyMath::Vector3& second)
{
    return first.x() * second.y() - first.y() * second.x();
}

// 使用缩放计算二维长度，避免中间平方溢出。
double stableLength(double first, double second)
{
    const double absoluteFirst = std::fabs(first);
    const double absoluteSecond = std::fabs(second);
    const double scale = (std::max)(absoluteFirst, absoluteSecond);

    if (scale == 0.0)
    {
        return 0.0;
    }

    const double normalizedFirst = absoluteFirst / scale;
    const double normalizedSecond = absoluteSecond / scale;

    return scale * std::sqrt(normalizedFirst * normalizedFirst + normalizedSecond * normalizedSecond);
}

// 返回坐标原点到闭区间的最短一维距离。
double distanceToInterval(double minimum, double maximum)
{
    if (minimum > 0.0)
    {
        return minimum;
    }

    if (maximum < 0.0)
    {
        return -maximum;
    }

    return 0.0;
}

// 判断周期参数candidate经过整数个period平移后是否落入指定有限闭区间。
bool periodicParameterInInterval(double candidate, double period, double firstParameter, double lastParameter)
{
    const double minimumParameter = (std::min)(firstParameter, lastParameter);
    const double maximumParameter = (std::max)(firstParameter, lastParameter);

    const double shiftCount = std::ceil((minimumParameter - candidate) / period);
    const double shiftedCandidate = candidate + shiftCount * period;

    return shiftedCandidate >= minimumParameter && shiftedCandidate <= maximumParameter;
}

// 返回Circle参数0对应的局部XY单位基方向。
MyMath::Vector3 circleXDirection(const MyBRep::Geometry_Circle& circle)
{
    return (circle.pointAt(0.0) - circle.center()).normalized(0.0);
}

// 返回Circle参数增加方向对应的局部XY单位基方向。
MyMath::Vector3 circleYDirection(const MyBRep::Geometry_Circle& circle)
{
    return circle.firstDerivativeAt(0.0).normalized(0.0);
}

// 返回局部XY点对应的Circle标准周期参数。
double circleParameterAtPoint(const MyBRep::Geometry_Circle& circle, const MyMath::Vector3& point)
{
    const MyMath::Vector3 relative = projectedPoint(point) - projectedPoint(circle.center());

    const MyMath::Vector3 xDirection = circleXDirection(circle);
    const MyMath::Vector3 yDirection = circleYDirection(circle);

    return std::atan2(MyMath::Vector3::dot(relative, yDirection), MyMath::Vector3::dot(relative, xDirection));
}

// 判断Circle有限参数区间是否包含指定局部XY圆上点。
bool circleSegmentContainsPoint(const MyBRep::Geometry_Circle& circle, double firstParameter, double lastParameter, const MyMath::Vector3& point, double tolerance)
{
    const MyMath::Vector3 center = projectedPoint(circle.center());
    const MyMath::Vector3 candidate = projectedPoint(point);
    const double radialDistance = stableLength(candidate.x() - center.x(), candidate.y() - center.y());

    if (std::fabs(radialDistance - circle.radius()) > tolerance)
    {
        return false;
    }

    const double parameter = circleParameterAtPoint(circle, candidate);

    return periodicParameterInInterval(parameter, TwoPi, firstParameter, lastParameter);
}

// 返回点到有限直线段的最短距离平方。
double pointSegmentDistanceSquared(const MyMath::Vector3& point, const MyMath::Vector3& start, const MyMath::Vector3& end)
{
    const MyMath::Vector3 segment = end - start;
    const double lengthSquared = segment.x() * segment.x() + segment.y() * segment.y();

    MYBREP_ASSERT_MESSAGE(lengthSquared > 0.0, "Geometry_Revolved line segment must be non-degenerate.");

    const MyMath::Vector3 relative = point - start;
    double parameter = (relative.x() * segment.x() + relative.y() * segment.y()) / lengthSquared;

    parameter = (std::max)(0.0, (std::min)(1.0, parameter));

    const MyMath::Vector3 closest = start + segment * parameter;
    const double deltaX = point.x() - closest.x();
    const double deltaY = point.y() - closest.y();

    return deltaX * deltaX + deltaY * deltaY;
}

// 返回点到Circle有限参数区间的最短距离平方。
double pointCircleSegmentDistanceSquared(const MyMath::Vector3& point, const MyBRep::Geometry_Circle& circle, double firstParameter, double lastParameter)
{
    const MyMath::Vector3 center = projectedPoint(circle.center());
    const double deltaX = point.x() - center.x();
    const double deltaY = point.y() - center.y();
    const double centerDistance = stableLength(deltaX, deltaY);

    if (centerDistance > 0.0)
    {
        const double parameter = circleParameterAtPoint(circle, point);

        if (periodicParameterInInterval(parameter, TwoPi, firstParameter, lastParameter))
        {
            const double radialDelta = centerDistance - circle.radius();

            return radialDelta * radialDelta;
        }
    }

    const MyMath::Vector3 start = projectedPoint(circle.pointAt(firstParameter));
    const MyMath::Vector3 end = projectedPoint(circle.pointAt(lastParameter));

    const double startDeltaX = point.x() - start.x();
    const double startDeltaY = point.y() - start.y();
    const double endDeltaX = point.x() - end.x();
    const double endDeltaY = point.y() - end.y();

    return (std::min)(startDeltaX * startDeltaX + startDeltaY * startDeltaY, endDeltaX * endDeltaX + endDeltaY * endDeltaY);
}

// 返回有限直线段与向右水平射线的交点数量，queryY已数值避开轮廓顶点。
unsigned int lineHorizontalRayIntersections(const MyMath::Vector3& point, double queryY, const MyMath::Vector3& start, const MyMath::Vector3& end, double tolerance)
{
    if ((start.y() > queryY) == (end.y() > queryY))
    {
        return 0;
    }

    const double parameter = (queryY - start.y()) / (end.y() - start.y());

    const double intersectionX = start.x() + (end.x() - start.x()) * parameter;

    return intersectionX > point.x() + tolerance ? 1U : 0U;
}

// 求Circle某个世界XY坐标分量等于target时的最多两个标准周期参数。
int circleCoordinateParameters(const MyBRep::Geometry_Circle& circle, bool solveX, double target, double parameters[2])
{
    const MyMath::Vector3 xDirection = circleXDirection(circle);
    const MyMath::Vector3 yDirection = circleYDirection(circle);
    const MyMath::Vector3 center = projectedPoint(circle.center());

    const double cosineCoefficient = circle.radius() * (solveX ? xDirection.x() : xDirection.y());
    const double sineCoefficient = circle.radius() * (solveX ? yDirection.x() : yDirection.y());
    const double value = target - (solveX ? center.x() : center.y());

    const double amplitude = stableLength(cosineCoefficient, sineCoefficient);

    if (amplitude == 0.0 || value < -amplitude || value > amplitude)
    {
        return 0;
    }

    const double phase = std::atan2(sineCoefficient, cosineCoefficient);
    const double ratio = (std::max)(-1.0, (std::min)(1.0, value / amplitude));
    const double delta = std::acos(ratio);

    parameters[0] = phase + delta;

    if (delta <= (std::numeric_limits<double>::epsilon)() * NumericalScale || std::fabs(delta - Pi) <= (std::numeric_limits<double>::epsilon)() * NumericalScale)
    {
        return 1;
    }

    parameters[1] = phase - delta;
    return 2;
}

// 返回Circle有限参数区间与向右水平射线的交点数量。
unsigned int circleHorizontalRayIntersections( const MyMath::Vector3& point, double queryY, const MyBRep::Geometry_Circle& circle, double firstParameter, double lastParameter,
    double tolerance)
{
    double parameters[2];
    const int count = circleCoordinateParameters(circle, false, queryY, parameters);

    unsigned int intersectionCount = 0;

    for (int index = 0; index < count; ++index)
    {
        if (!periodicParameterInInterval(parameters[index], TwoPi, firstParameter, lastParameter))
        {
            continue;
        }

        const MyMath::Vector3 derivative = circle.firstDerivativeAt(parameters[index]);
        const double derivativeScale = (std::max)(1.0, circle.radius());
        const double tangentTolerance = derivativeScale * (std::numeric_limits<double>::epsilon)() * NumericalScale;

        if (std::fabs(derivative.y()) <= tangentTolerance)
        {
            continue;
        }

        const MyMath::Vector3 candidate = projectedPoint(circle.pointAt(parameters[index]));

        if (candidate.x() > point.x() + tolerance)
        {
            ++intersectionCount;
        }
    }

    return intersectionCount;
}

// 判断点是否位于局部XY平面矩形内部或边界上。
bool rectangleContainsPoint(const MyBRep::Bounds3& bounds, const MyMath::Vector3& point, double tolerance)
{
    return point.x() >= bounds.minimum().x() - tolerance && point.x() <= bounds.maximum().x() + tolerance && point.y() >= bounds.minimum().y() - tolerance &&
        point.y() <= bounds.maximum().y() + tolerance;
}

// 使用二维Slab算法判断有限直线段是否与局部XY平面矩形相交或接触。
bool lineSegmentIntersectsRectangle(const MyMath::Vector3& start, const MyMath::Vector3& end, const MyBRep::Bounds3& bounds, double tolerance)
{
    double minimumParameter = 0.0;
    double maximumParameter = 1.0;

    const MyMath::Vector3 direction = end - start;

    const double starts[2] =
    {
        start.x(), start.y() };
    const double deltas[2] =
    {
        direction.x(), direction.y() };
    const double minimums[2] =
    {
        bounds.minimum().x() - tolerance, bounds.minimum().y() - tolerance };
    const double maximums[2] =
    {
        bounds.maximum().x() + tolerance, bounds.maximum().y() + tolerance };

    for (int axis = 0; axis < 2; ++axis)
    {
        if (deltas[axis] == 0.0)
        {
            if (starts[axis] < minimums[axis] || starts[axis] > maximums[axis])
            {
                return false;
            }

            continue;
        }

        double first = (minimums[axis] - starts[axis]) / deltas[axis];
        double second = (maximums[axis] - starts[axis]) / deltas[axis];

        if (first > second)
        {
            std::swap(first, second);
        }

        minimumParameter = (std::max)(minimumParameter, first);
        maximumParameter = (std::min)(maximumParameter, second);

        if (minimumParameter > maximumParameter)
        {
            return false;
        }
    }

    return true;
}

// 判断Circle有限参数区间是否与局部XY平面矩形相交或接触。
bool circleSegmentIntersectsRectangle(const MyBRep::Geometry_Circle& circle, double firstParameter, double lastParameter, const MyBRep::Bounds3& bounds, double tolerance)
{
    const MyMath::Vector3 start = projectedPoint(circle.pointAt(firstParameter));
    const MyMath::Vector3 end = projectedPoint(circle.pointAt(lastParameter));

    if (rectangleContainsPoint(bounds, start, tolerance) || rectangleContainsPoint(bounds, end, tolerance))
    {
        return true;
    }

    const double verticalEdges[2] =
    {
        bounds.minimum().x(), bounds.maximum().x() };

    for (int edgeIndex = 0; edgeIndex < 2; ++edgeIndex)
    {
        double parameters[2];
        const int count = circleCoordinateParameters(circle, true, verticalEdges[edgeIndex], parameters);

        for (int index = 0; index < count; ++index)
        {
            if (!periodicParameterInInterval(parameters[index], TwoPi, firstParameter, lastParameter))
            {
                continue;
            }

            const MyMath::Vector3 candidate = projectedPoint(circle.pointAt(parameters[index]));

            if (candidate.y() >= bounds.minimum().y() - tolerance && candidate.y() <= bounds.maximum().y() + tolerance)
            {
                return true;
            }
        }
    }

    const double horizontalEdges[2] =
    {
        bounds.minimum().y(), bounds.maximum().y() };

    for (int edgeIndex = 0; edgeIndex < 2; ++edgeIndex)
    {
        double parameters[2];
        const int count = circleCoordinateParameters(circle, false, horizontalEdges[edgeIndex], parameters);

        for (int index = 0; index < count; ++index)
        {
            if (!periodicParameterInInterval(parameters[index], TwoPi, firstParameter, lastParameter))
            {
                continue;
            }

            const MyMath::Vector3 candidate = projectedPoint(circle.pointAt(parameters[index]));

            if (candidate.x() >= bounds.minimum().x() - tolerance && candidate.x() <= bounds.maximum().x() + tolerance)
            {
                return true;
            }
        }
    }

    return false;
}

}

namespace MyBRep
{

Geometry_Revolved::Geometry_Revolved( const std::vector<ProfileSegment>& profileSegments, double profileTolerance) : m_profileSegments(profileSegments)
    , m_profileTolerance(profileTolerance) , m_profileSignedArea(0.0) , m_radialSign(0.0)
{
    MYBREP_ASSERT_MESSAGE(isFiniteNonNegative(profileTolerance), "Geometry_Revolved profile tolerance must be finite and non-negative.");
    MYBREP_ASSERT_MESSAGE(!m_profileSegments.empty(), "Geometry_Revolved requires a non-empty closed profile.");

    validateProfile();
    rebuildCaches();
}

/// 母线几何数据

std::size_t Geometry_Revolved::profileSegmentCount() const
{
    return m_profileSegments.size();
}

const Geometry_Revolved::ProfileSegment& Geometry_Revolved::profileSegment(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(index < m_profileSegments.size(), "Geometry_Revolved profile segment index is out of range.");

    return m_profileSegments[index];
}

const std::vector<Geometry_Revolved::ProfileSegment>& Geometry_Revolved::profileSegments() const
{
    return m_profileSegments;
}

double Geometry_Revolved::profileTolerance() const
{
    return m_profileTolerance;
}

const Bounds3& Geometry_Revolved::profileBounds() const
{
    return m_profileBounds;
}

double Geometry_Revolved::profileSignedArea() const
{
    return m_profileSignedArea;
}

double Geometry_Revolved::radialSign() const
{
    return m_radialSign;
}

/// 几何属性

ShapeKind Geometry_Revolved::kind() const
{
    return ShapeKind::Revolved;
}

/// 标准空间查询

bool Geometry_Revolved::containsLocalPoint(const MyMath::Vector3& point) const
{
    MYBREP_ASSERT_MESSAGE(point.isFinite(), "Geometry_Revolved query point must be finite.");

    if (!localBounds().contains(point))
    {
        return false;
    }

    const double radius = stableLength(point.x(), point.y());

    return containsProfilePoint(MyMath::Vector3(m_radialSign * radius, point.z(), 0.0), m_profileTolerance);
}

bool Geometry_Revolved::supportsSignedDistance() const
{
    return true;
}

double Geometry_Revolved::signedDistanceLocalPoint(const MyMath::Vector3& point) const
{
    MYBREP_ASSERT_MESSAGE(point.isFinite(), "Geometry_Revolved signed-distance query point must be finite.");

    const double radius = stableLength(point.x(), point.y());
    const MyMath::Vector3 profilePoint(m_radialSign * radius, point.z(), 0.0);

    const double distance = profileBoundaryDistance(profilePoint);

    if (distance == 0.0)
    {
        return 0.0;
    }

    return containsProfilePoint(profilePoint, m_profileTolerance) ? -distance : distance;
}

ShapeRelation Geometry_Revolved::classifyLocalBounds(const Bounds3& bounds) const
{
    MYBREP_ASSERT_MESSAGE(bounds.isValid(), "Geometry_Revolved query bounds must be valid.");

    return classifyRange(bounds.minimum(), bounds.maximum());
}

/// 快速空间查询

ShapeRelation Geometry_Revolved::classifyLocalBoundsFast(const MyMath::Vector3& center, const MyMath::Vector3& extent) const
{
    MYBREP_ASSERT_MESSAGE(center.isFinite(), "Geometry_Revolved query center must be finite.");
    MYBREP_ASSERT_MESSAGE(isValidExtent(extent), "Geometry_Revolved query extent must be finite and non-negative.");

    return classifyRange(center - extent, center + extent);
}

/// 内部校验

void Geometry_Revolved::validateProfile() const
{
    for (std::size_t index = 0; index < m_profileSegments.size(); ++index)
    {
        const ProfileSegment& segment = m_profileSegments[index];

        MYBREP_ASSERT_MESSAGE(segment.curve, "Geometry_Revolved profile segment curve must not be null.");
        MYBREP_ASSERT_MESSAGE(isFiniteValue(segment.firstParameter) && isFiniteValue(segment.lastParameter), "Geometry_Revolved profile segment parameters must be finite.");
        MYBREP_ASSERT_MESSAGE(segment.firstParameter != segment.lastParameter, "Geometry_Revolved profile segment parameter interval must be non-degenerate.");
        MYBREP_ASSERT_MESSAGE( segment.curve->isParameterInDomain(segment.firstParameter) && segment.curve->isParameterInDomain(segment.lastParameter),
            "Geometry_Revolved profile segment parameters must lie in the curve natural parameter domain.");
        MYBREP_ASSERT_MESSAGE( segment.curve->kind() == CurveKind::Line || segment.curve->kind() == CurveKind::Circle,
            "Geometry_Revolved currently supports only Line and Circle profile segments.");

        const MyMath::Vector3 sourceStart = segment.curve->pointAt(segment.firstParameter);
        const MyMath::Vector3 sourceEnd = segment.curve->pointAt(segment.lastParameter);

        MYBREP_ASSERT_MESSAGE( std::fabs(sourceStart.z()) <= m_profileTolerance && std::fabs(sourceEnd.z()) <= m_profileTolerance,
            "Geometry_Revolved profile segment endpoints must lie in the local XY plane.");

        if (segment.curve->kind() == CurveKind::Circle)
        {
            const Geometry_Circle& circle = static_cast<const Geometry_Circle&>(*segment.curve);
            const MyMath::Vector3 normal = circle.normal();
            const double sweep = std::fabs(segment.lastParameter - segment.firstParameter);

            MYBREP_ASSERT_MESSAGE(std::fabs(circle.center().z()) <= m_profileTolerance, "Geometry_Revolved profile circle center must lie in the local XY plane.");
            MYBREP_ASSERT_MESSAGE( std::fabs(normal.x()) <= MyMath::Vector3::DefaultEpsilon && std::fabs(normal.y()) <= MyMath::Vector3::DefaultEpsilon &&
                std::fabs(std::fabs(normal.z()) - 1.0) <= MyMath::Vector3::DefaultEpsilon, "Geometry_Revolved profile circle must lie in the local XY plane.");
            MYBREP_ASSERT_MESSAGE(sweep <= TwoPi + MyMath::Vector3::DefaultEpsilon, "Geometry_Revolved profile circle interval must not exceed one complete period.");
        }

        const ProfileSegment& next = m_profileSegments[(index + 1) % m_profileSegments.size()];
        const MyMath::Vector3 end = segmentEndPoint(segment);
        const MyMath::Vector3 nextStart = segmentStartPoint(next);

        const double connectionTolerance = effectivePointTolerance(end, nextStart, m_profileTolerance);

        MYBREP_ASSERT_MESSAGE(end.isEqualTo(nextStart, connectionTolerance), "Geometry_Revolved profile segments must form an ordered closed loop.");
    }
}

/// 缓存建立

void Geometry_Revolved::rebuildCaches()
{
    m_profileBounds.clear();
    m_profileSignedArea = 0.0;
    m_radialSign = 0.0;

    for (std::size_t index = 0; index < m_profileSegments.size(); ++index)
    {
        const ProfileSegment& segment = m_profileSegments[index];
        const MyMath::Vector3 start = segmentStartPoint(segment);
        const MyMath::Vector3 end = segmentEndPoint(segment);

        m_profileBounds.include(start);
        m_profileBounds.include(end);

        if (segment.curve->kind() == CurveKind::Line)
        {
            m_profileSignedArea += cross2D(start, end) * 0.5;

            continue;
        }

        const Geometry_Circle& circle = static_cast<const Geometry_Circle&>(*segment.curve);
        const MyMath::Vector3 center = projectedPoint(circle.center());
        const double orientationSign = circle.normal().z() >= 0.0 ? 1.0 : -1.0;
        const double sweep = segment.lastParameter - segment.firstParameter;

        m_profileSignedArea += 0.5 * (cross2D(center, end - start) + circle.radius() * circle.radius() * orientationSign * sweep);

        const MyMath::Vector3 extrema[4] =
        {
            MyMath::Vector3( center.x() + circle.radius(), center.y(), 0.0), MyMath::Vector3( center.x() - circle.radius(), center.y(), 0.0), MyMath::Vector3( center.x(),
                center.y() + circle.radius(), 0.0), MyMath::Vector3( center.x(), center.y() - circle.radius(), 0.0) };

        for (int extremaIndex = 0; extremaIndex < 4; ++extremaIndex)
        {
            if (circleSegmentContainsPoint(circle, segment.firstParameter, segment.lastParameter, extrema[extremaIndex], m_profileTolerance))
            {
                m_profileBounds.include(extrema[extremaIndex]);
            }
        }
    }

    MYBREP_ASSERT_MESSAGE(m_profileBounds.isValid(), "Geometry_Revolved profile bounds must be valid.");
    MYBREP_ASSERT_MESSAGE(isFiniteValue(m_profileSignedArea) && m_profileSignedArea != 0.0, "Geometry_Revolved profile must enclose a non-zero signed area.");

    const double minimumX = m_profileBounds.minimum().x();
    const double maximumX = m_profileBounds.maximum().x();

    const bool onPositiveSide = minimumX >= -m_profileTolerance;
    const bool onNegativeSide = maximumX <= m_profileTolerance;

    MYBREP_ASSERT_MESSAGE(onPositiveSide || onNegativeSide, "Geometry_Revolved profile must remain entirely on one side of the local Y axis.");

    m_radialSign = onPositiveSide ? 1.0 : -1.0;

    const double maximumRadius = (std::max)(std::fabs(minimumX), std::fabs(maximumX));
    const double minimumZ = m_profileBounds.minimum().y();
    const double maximumZ = m_profileBounds.maximum().y();

    const Bounds3 bounds(MyMath::Vector3(-maximumRadius, -maximumRadius, minimumZ), MyMath::Vector3(maximumRadius, maximumRadius, maximumZ));

    MYBREP_ASSERT_MESSAGE(bounds.isValid() && bounds.hasVolume(), "Geometry_Revolved profile must define a finite non-degenerate three-dimensional solid.");

    setLocalBounds(bounds);
}

MyMath::Vector3 Geometry_Revolved::segmentStartPoint(const ProfileSegment& segment) const
{
    return projectedPoint(segment.curve->pointAt(segment.firstParameter));
}

MyMath::Vector3 Geometry_Revolved::segmentEndPoint(const ProfileSegment& segment) const
{
    return projectedPoint(segment.curve->pointAt(segment.lastParameter));
}

/// 母线区域查询

bool Geometry_Revolved::containsProfilePoint(const MyMath::Vector3& point, double tolerance) const
{
    MYBREP_ASSERT_MESSAGE(isFiniteNonNegative(tolerance), "Geometry_Revolved profile query tolerance must be finite and non-negative.");

    if (!m_profileBounds.contains(point, tolerance))
    {
        return false;
    }

    const double toleranceSquared = tolerance * tolerance;

    for (std::size_t index = 0; index < m_profileSegments.size(); ++index)
    {
        const ProfileSegment& segment = m_profileSegments[index];

        if (segment.curve->kind() == CurveKind::Line)
        {
            if (pointSegmentDistanceSquared(point, segmentStartPoint(segment), segmentEndPoint(segment)) <= toleranceSquared)
            {
                return true;
            }

            continue;
        }

        const Geometry_Circle& circle = static_cast<const Geometry_Circle&>(*segment.curve);

        if (pointCircleSegmentDistanceSquared(point, circle, segment.firstParameter, segment.lastParameter) <= toleranceSquared)
        {
            return true;
        }
    }

    double coordinateScale = 1.0;

    coordinateScale = (std::max)(coordinateScale, std::fabs(point.x()));
    coordinateScale = (std::max)(coordinateScale, std::fabs(point.y()));
    coordinateScale = (std::max)(coordinateScale, std::fabs(m_profileBounds.minimum().x()));
    coordinateScale = (std::max)(coordinateScale, std::fabs(m_profileBounds.minimum().y()));
    coordinateScale = (std::max)(coordinateScale, std::fabs(m_profileBounds.maximum().x()));
    coordinateScale = (std::max)(coordinateScale, std::fabs(m_profileBounds.maximum().y()));

    const double queryY = point.y() + coordinateScale * (std::numeric_limits<double>::epsilon)() * NumericalScale;

    unsigned int intersectionCount = 0;

    for (std::size_t index = 0; index < m_profileSegments.size(); ++index)
    {
        const ProfileSegment& segment = m_profileSegments[index];

        if (segment.curve->kind() == CurveKind::Line)
        {
            intersectionCount += lineHorizontalRayIntersections(point, queryY, segmentStartPoint(segment), segmentEndPoint(segment), tolerance);

            continue;
        }

        const Geometry_Circle& circle = static_cast<const Geometry_Circle&>(*segment.curve);

        intersectionCount += circleHorizontalRayIntersections(point, queryY, circle, segment.firstParameter, segment.lastParameter, tolerance);
    }

    return (intersectionCount & 1U) != 0;
}

double Geometry_Revolved::profileBoundaryDistance(const MyMath::Vector3& point) const
{
    double minimumDistanceSquared = (std::numeric_limits<double>::infinity)();

    for (std::size_t index = 0; index < m_profileSegments.size(); ++index)
    {
        const ProfileSegment& segment = m_profileSegments[index];
        double distanceSquared = 0.0;

        if (segment.curve->kind() == CurveKind::Line)
        {
            distanceSquared = pointSegmentDistanceSquared(point, segmentStartPoint(segment), segmentEndPoint(segment));
        }
        else
        {
            const Geometry_Circle& circle = static_cast<const Geometry_Circle&>(*segment.curve);

            distanceSquared = pointCircleSegmentDistanceSquared(point, circle, segment.firstParameter, segment.lastParameter);
        }

        minimumDistanceSquared = (std::min)(minimumDistanceSquared, distanceSquared);
    }

    MYBREP_ASSERT_MESSAGE(isFiniteValue(minimumDistanceSquared) && minimumDistanceSquared >= 0.0, "Geometry_Revolved failed to resolve a finite profile boundary distance.");

    return std::sqrt(minimumDistanceSquared);
}

ShapeRelation Geometry_Revolved::classifyProfileBounds(const Bounds3& bounds) const
{
    if (!m_profileBounds.intersects(bounds, m_profileTolerance))
    {
        return ShapeRelation::Outside;
    }

    const MyMath::Vector3 corners[4] =
    {
        MyMath::Vector3(bounds.minimum().x(), bounds.minimum().y(), 0.0), MyMath::Vector3(bounds.maximum().x(), bounds.minimum().y(), 0.0), MyMath::Vector3(bounds.maximum().x(),
            bounds.maximum().y(), 0.0), MyMath::Vector3(bounds.minimum().x(), bounds.maximum().y(), 0.0) };

    bool allCornersInside = true;
    bool anyCornerInside = false;

    for (int index = 0; index < 4; ++index)
    {
        const bool inside = containsProfilePoint(corners[index], m_profileTolerance);

        allCornersInside = allCornersInside && inside;
        anyCornerInside = anyCornerInside || inside;
    }

    bool boundaryIntersects = false;

    for (std::size_t index = 0; index < m_profileSegments.size(); ++index)
    {
        const ProfileSegment& segment = m_profileSegments[index];

        if (segment.curve->kind() == CurveKind::Line)
        {
            if (lineSegmentIntersectsRectangle(segmentStartPoint(segment), segmentEndPoint(segment), bounds, m_profileTolerance))
            {
                boundaryIntersects = true;
                break;
            }

            continue;
        }

        const Geometry_Circle& circle = static_cast<const Geometry_Circle&>(*segment.curve);

        if (circleSegmentIntersectsRectangle(circle, segment.firstParameter, segment.lastParameter, bounds, m_profileTolerance))
        {
            boundaryIntersects = true;
            break;
        }
    }

    if (boundaryIntersects)
    {
        return ShapeRelation::Intersecting;
    }

    if (allCornersInside)
    {
        return ShapeRelation::Inside;
    }

    if (anyCornerInside)
    {
        return ShapeRelation::Intersecting;
    }

    return ShapeRelation::Outside;
}

/// 三维范围分类

ShapeRelation Geometry_Revolved::classifyRange(const MyMath::Vector3& minimum, const MyMath::Vector3& maximum) const
{
    const double minimumRadius = stableLength(distanceToInterval(minimum.x(), maximum.x()), distanceToInterval(minimum.y(), maximum.y()));

    double maximumRadius = 0.0;

    maximumRadius = (std::max)(maximumRadius, stableLength(minimum.x(), minimum.y()));
    maximumRadius = (std::max)(maximumRadius, stableLength(maximum.x(), minimum.y()));
    maximumRadius = (std::max)(maximumRadius, stableLength(maximum.x(), maximum.y()));
    maximumRadius = (std::max)(maximumRadius, stableLength(minimum.x(), maximum.y()));

    const double profileMinimumX = m_radialSign > 0.0 ? minimumRadius : -maximumRadius;
    const double profileMaximumX = m_radialSign > 0.0 ? maximumRadius : -minimumRadius;

    const Bounds3 profileQueryBounds(MyMath::Vector3(profileMinimumX, minimum.z(), 0.0), MyMath::Vector3(profileMaximumX, maximum.z(), 0.0));

    const ShapeRelation relation = classifyProfileBounds(profileQueryBounds);

    if (relation == ShapeRelation::Outside)
    {
        return ShapeRelation::Outside;
    }

    if (relation == ShapeRelation::Inside)
    {
        return ShapeRelation::Inside;
    }

    return ShapeRelation::Intersecting;
}

}
