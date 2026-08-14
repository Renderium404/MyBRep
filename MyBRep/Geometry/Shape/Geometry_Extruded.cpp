#include "Geometry_Extruded.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "MyBRep/Foundation/Diagnostic.h"
#include "MyBRep/Geometry/Curve/Geometry_Circle.h"
#include "MyBRep/Geometry/Curve/Geometry_Line.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 圆周率，圆弧母线参数判断统一使用弧度制。
const double TwoPi = Pi * 2.0; // 单个Circle母线区间允许覆盖的最大完整周期。
const double HalfScale = 0.5; // 完整拉伸高度转换为半高度使用的固定比例。
const double NumericalScale = 64.0; // 浮点角度和水平射线判定覆盖舍入误差使用的固定倍数。

// 判断标量是否为有限值。
bool isFiniteValue(double value)
{
    const double infinity = (std::numeric_limits<double>::infinity)();
    return value == value && value != infinity && value != -infinity;
}

// 判断标量是否为有限正数。
bool isFinitePositive(double value)
{
    return isFiniteValue(value) && value > 0.0;
}

// 判断标量是否为有限非负数。
bool isFiniteNonNegative(double value)
{
    return isFiniteValue(value) && value >= 0.0;
}

// 返回二维向量长度。
double length2D(double x, double y)
{
    return std::sqrt(x * x + y * y);
}

// 返回二维向量叉积标量。
double cross2D(const MyMath::Vector3& first, const MyMath::Vector3& second)
{
    return first.x() * second.y() - first.y() * second.x();
}

// 返回覆盖显式几何容差和有限浮点求值舍入误差的比较容差。
double effectivePointTolerance(const MyMath::Vector3& first,
                               const MyMath::Vector3& second,
                               double geometricTolerance)
{
    double scale = 1.0;
    scale = (std::max)(scale, std::fabs(first.x()));
    scale = (std::max)(scale, std::fabs(first.y()));
    scale = (std::max)(scale, std::fabs(first.z()));
    scale = (std::max)(scale, std::fabs(second.x()));
    scale = (std::max)(scale, std::fabs(second.y()));
    scale = (std::max)(scale, std::fabs(second.z()));

    return (std::max)(geometricTolerance,
                      scale * std::numeric_limits<double>::epsilon() * NumericalScale);
}

// 返回二维点到闭线段的最短距离平方。
double pointSegmentDistanceSquared(const MyMath::Vector3& point,
                                   const MyMath::Vector3& start,
                                   const MyMath::Vector3& end)
{
    const MyMath::Vector3 segment = end - start;
    const double lengthSquared = segment.x() * segment.x() + segment.y() * segment.y();

    MYBREP_ASSERT_MESSAGE(lengthSquared > 0.0,
                          "Geometry_Extruded line profile segment must not be degenerate.");

    double parameter =
        ((point.x() - start.x()) * segment.x() +
         (point.y() - start.y()) * segment.y()) / lengthSquared;

    parameter = (std::max)(0.0, (std::min)(1.0, parameter));

    const double closestX = start.x() + segment.x() * parameter;
    const double closestY = start.y() + segment.y() * parameter;
    const double deltaX = point.x() - closestX;
    const double deltaY = point.y() - closestY;
    return deltaX * deltaX + deltaY * deltaY;
}

// 判断周期参数的某个等价值是否落在无方向参数区间内。
bool periodicParameterInInterval(double parameter,
                                 double firstParameter,
                                 double lastParameter,
                                 double tolerance)
{
    const double minimumParameter = (std::min)(firstParameter, lastParameter);
    const double maximumParameter = (std::max)(firstParameter, lastParameter);
    const double shiftCount = std::ceil((minimumParameter - parameter - tolerance) / TwoPi);
    const double candidate = parameter + shiftCount * TwoPi;

    return candidate >= minimumParameter - tolerance &&
           candidate <= maximumParameter + tolerance;
}

// 将局部XY平面中的世界点转换为Circle自身角参数。
double circleParameterForPoint(const MyBRep::Geometry_Circle& circle,
                               const MyMath::Vector3& point)
{
    const MyMath::Vector3 relative = point - circle.center();
    const MyMath::Vector3 xDir = (circle.pointAt(0.0) - circle.center()).normalized(0.0);
    const MyMath::Vector3 yDir = circle.firstDerivativeAt(0.0).normalized(0.0);
    const double localX = MyMath::Vector3::dot(relative, xDir);
    const double localY = MyMath::Vector3::dot(relative, yDir);
    return std::atan2(localY, localX);
}

// 判断Circle有限区间是否包含指定局部XY平面点对应的圆周位置。
bool circleSegmentContainsPoint(const MyBRep::Geometry_Circle& circle,
                                double firstParameter,
                                double lastParameter,
                                const MyMath::Vector3& point,
                                double tolerance)
{
    const double parameter = circleParameterForPoint(circle, point);
    const double angleTolerance = tolerance > 0.0 ? tolerance / circle.radius() : 0.0;
    return periodicParameterInInterval(parameter,
                                       firstParameter,
                                       lastParameter,
                                       angleTolerance);
}

// 返回点到有限Circle区间的最短距离平方。
double pointCircleSegmentDistanceSquared(const MyMath::Vector3& point,
                                         const MyBRep::Geometry_Circle& circle,
                                         double firstParameter,
                                         double lastParameter,
                                         double tolerance)
{
    const double deltaX = point.x() - circle.center().x();
    const double deltaY = point.y() - circle.center().y();
    const double centerDistance = length2D(deltaX, deltaY);

    if (centerDistance > 0.0)
    {
        const MyMath::Vector3 radialPoint(circle.center().x() + deltaX * circle.radius() / centerDistance,
                                          circle.center().y() + deltaY * circle.radius() / centerDistance,
                                          0.0);

        if (circleSegmentContainsPoint(circle,
                                       firstParameter,
                                       lastParameter,
                                       radialPoint,
                                       tolerance))
        {
            const double radialDelta = centerDistance - circle.radius();
            return radialDelta * radialDelta;
        }
    }

    const MyMath::Vector3 start = circle.pointAt(firstParameter);
    const MyMath::Vector3 end = circle.pointAt(lastParameter);
    const double startDeltaX = point.x() - start.x();
    const double startDeltaY = point.y() - start.y();
    const double endDeltaX = point.x() - end.x();
    const double endDeltaY = point.y() - end.y();

    return (std::min)(startDeltaX * startDeltaX + startDeltaY * startDeltaY,
                      endDeltaX * endDeltaX + endDeltaY * endDeltaY);
}

// 使用二维Slab算法判断直线段是否与局部XY矩形相交或接触。
bool lineSegmentIntersectsRectangle(const MyMath::Vector3& start,
                                    const MyMath::Vector3& end,
                                    const MyBRep::Bounds3& bounds,
                                    double tolerance)
{
    double minimumParameter = 0.0;
    double maximumParameter = 1.0;
    const MyMath::Vector3 direction = end - start;
    const double starts[2] = {start.x(), start.y()};
    const double deltas[2] = {direction.x(), direction.y()};
    const double minimums[2] = {bounds.minimum().x() - tolerance, bounds.minimum().y() - tolerance};
    const double maximums[2] = {bounds.maximum().x() + tolerance, bounds.maximum().y() + tolerance};

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

// 判断点是否位于局部XY矩形内部或边界上。
bool rectangleContainsPoint(const MyBRep::Bounds3& bounds,
                            const MyMath::Vector3& point,
                            double tolerance)
{
    return point.x() >= bounds.minimum().x() - tolerance &&
           point.x() <= bounds.maximum().x() + tolerance &&
           point.y() >= bounds.minimum().y() - tolerance &&
           point.y() <= bounds.maximum().y() + tolerance;
}

}

namespace MyBRep
{

Geometry_Extruded::Geometry_Extruded(const std::vector<ProfileSegment>& profileSegments,
                                     double height,
                                     double profileTolerance)
    : m_profileSegments(profileSegments)
    , m_height(height)
    , m_halfHeight(height * HalfScale)
    , m_profileTolerance(profileTolerance)
    , m_profileSignedArea(0.0)
{
    MYBREP_ASSERT_MESSAGE(isFinitePositive(height),
                          "Geometry_Extruded height must be finite and positive.");
    MYBREP_ASSERT_MESSAGE(isFiniteNonNegative(profileTolerance),
                          "Geometry_Extruded profile tolerance must be finite and non-negative.");
    MYBREP_ASSERT_MESSAGE(!m_profileSegments.empty(),
                          "Geometry_Extruded requires a non-empty closed profile.");

    validateProfile();
    rebuildCaches();
}

/// 母线几何数据

std::size_t Geometry_Extruded::profileSegmentCount() const
{
    return m_profileSegments.size();
}

const Geometry_Extruded::ProfileSegment& Geometry_Extruded::profileSegment(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(index < m_profileSegments.size(),
                          "Geometry_Extruded profile segment index is out of range.");

    return m_profileSegments[index];
}

const std::vector<Geometry_Extruded::ProfileSegment>& Geometry_Extruded::profileSegments() const
{
    return m_profileSegments;
}

double Geometry_Extruded::profileTolerance() const
{
    return m_profileTolerance;
}

const Bounds3& Geometry_Extruded::profileBounds() const
{
    return m_profileBounds;
}

double Geometry_Extruded::profileSignedArea() const
{
    return m_profileSignedArea;
}

/// 拉伸参数

double Geometry_Extruded::height() const
{
    return m_height;
}

/// 几何属性

ShapeKind Geometry_Extruded::kind() const
{
    return ShapeKind::Extruded;
}

/// 标准空间查询

bool Geometry_Extruded::containsLocalPoint(const MyMath::Vector3& point) const
{
    MYBREP_ASSERT_MESSAGE(point.isFinite(),
                          "Geometry_Extruded query point must be finite.");

    if (point.z() < -m_halfHeight || point.z() > m_halfHeight)
    {
        return false;
    }

    return containsProfilePoint(MyMath::Vector3(point.x(), point.y(), 0.0), m_profileTolerance);
}

bool Geometry_Extruded::supportsSignedDistance() const
{
    return true;
}

double Geometry_Extruded::signedDistanceLocalPoint(const MyMath::Vector3& point) const
{
    MYBREP_ASSERT_MESSAGE(point.isFinite(),
                          "Geometry_Extruded signed-distance query point must be finite.");

    const MyMath::Vector3 profilePoint(point.x(), point.y(), 0.0);
    const double boundaryDistance = profileBoundaryDistance(profilePoint);
    const bool insideProfile = containsProfilePoint(profilePoint, 0.0);
    const double profileSignedDistance = insideProfile ? -boundaryDistance : boundaryDistance;
    const double axialSignedDistance = std::fabs(point.z()) - m_halfHeight;
    const double outsideProfile = (std::max)(profileSignedDistance, 0.0);
    const double outsideAxial = (std::max)(axialSignedDistance, 0.0);
    const double outsideDistance =
        std::sqrt(outsideProfile * outsideProfile + outsideAxial * outsideAxial);
    const double insideDistance =
        (std::min)((std::max)(profileSignedDistance, axialSignedDistance), 0.0);

    return outsideDistance + insideDistance;
}

ShapeRelation Geometry_Extruded::classifyLocalBounds(const Bounds3& bounds) const
{
    MYBREP_ASSERT_MESSAGE(bounds.isValid(),
                          "Geometry_Extruded query bounds must be valid.");

    const MyMath::Vector3& minimum = bounds.minimum();
    const MyMath::Vector3& maximum = bounds.maximum();

    if (maximum.z() < -m_halfHeight || minimum.z() > m_halfHeight)
    {
        return ShapeRelation::Outside;
    }

    const Bounds3 profileQueryBounds(MyMath::Vector3(minimum.x(), minimum.y(), 0.0),
                                     MyMath::Vector3(maximum.x(), maximum.y(), 0.0));
    const ShapeRelation profileRelation = classifyProfileBounds(profileQueryBounds);

    if (profileRelation == ShapeRelation::Outside)
    {
        return ShapeRelation::Outside;
    }

    if (profileRelation == ShapeRelation::Inside &&
        minimum.z() >= -m_halfHeight &&
        maximum.z() <= m_halfHeight)
    {
        return ShapeRelation::Inside;
    }

    return ShapeRelation::Intersecting;
}

/// 内部校验

void Geometry_Extruded::validateProfile() const
{
    for (std::size_t index = 0; index < m_profileSegments.size(); ++index)
    {
        const ProfileSegment& segment = m_profileSegments[index];

        MYBREP_ASSERT_MESSAGE(segment.curve,
                              "Geometry_Extruded profile segment curve must not be null.");
        MYBREP_ASSERT_MESSAGE(isFiniteValue(segment.firstParameter) &&
                              isFiniteValue(segment.lastParameter),
                              "Geometry_Extruded profile segment parameters must be finite.");
        MYBREP_ASSERT_MESSAGE(segment.firstParameter != segment.lastParameter,
                              "Geometry_Extruded profile segment parameter interval must be non-degenerate.");
        MYBREP_ASSERT_MESSAGE(segment.curve->kind() == CurveKind::Line ||
                              segment.curve->kind() == CurveKind::Circle,
                              "Geometry_Extruded currently supports only Line and Circle profile segments.");

        const MyMath::Vector3 start = segmentStartPoint(segment);
        const MyMath::Vector3 end = segmentEndPoint(segment);

        MYBREP_ASSERT_MESSAGE(std::fabs(start.z()) <= m_profileTolerance &&
                              std::fabs(end.z()) <= m_profileTolerance,
                              "Geometry_Extruded profile segment endpoints must lie in the local XY plane.");

        if (segment.curve->kind() == CurveKind::Circle)
        {
            const Geometry_Circle& circle =
                static_cast<const Geometry_Circle&>(*segment.curve);
            const MyMath::Vector3 normal = circle.normal();
            const double sweep = std::fabs(segment.lastParameter - segment.firstParameter);

            MYBREP_ASSERT_MESSAGE(std::fabs(circle.center().z()) <= m_profileTolerance,
                                  "Geometry_Extruded profile circle center must lie in the local XY plane.");
            MYBREP_ASSERT_MESSAGE(std::fabs(normal.x()) <= MyMath::Vector3::DefaultEpsilon &&
                                  std::fabs(normal.y()) <= MyMath::Vector3::DefaultEpsilon &&
                                  std::fabs(std::fabs(normal.z()) - 1.0) <= MyMath::Vector3::DefaultEpsilon,
                                  "Geometry_Extruded profile circle must lie in the local XY plane.");
            MYBREP_ASSERT_MESSAGE(sweep <= TwoPi + MyMath::Vector3::DefaultEpsilon,
                                  "Geometry_Extruded profile circle interval must not exceed one full period.");
        }

        const ProfileSegment& next =
            m_profileSegments[(index + 1) % m_profileSegments.size()];
        const MyMath::Vector3 nextStart = segmentStartPoint(next);

        const double connectionTolerance =
            effectivePointTolerance(end, nextStart, m_profileTolerance);

        MYBREP_ASSERT_MESSAGE(end.isEqualTo(nextStart, connectionTolerance),
                              "Geometry_Extruded profile segments must form an ordered closed loop.");
    }
}

void Geometry_Extruded::rebuildCaches()
{
    m_profileBounds.clear();
    m_profileSignedArea = 0.0;

    for (std::size_t index = 0; index < m_profileSegments.size(); ++index)
    {
        const ProfileSegment& segment = m_profileSegments[index];
        const MyMath::Vector3 start = segmentStartPoint(segment);
        const MyMath::Vector3 end = segmentEndPoint(segment);

        m_profileBounds.include(MyMath::Vector3(start.x(), start.y(), 0.0));
        m_profileBounds.include(MyMath::Vector3(end.x(), end.y(), 0.0));

        if (segment.curve->kind() == CurveKind::Line)
        {
            m_profileSignedArea += cross2D(start, end) * 0.5;
            continue;
        }

        const Geometry_Circle& circle =
            static_cast<const Geometry_Circle&>(*segment.curve);
        const MyMath::Vector3 center(circle.center().x(), circle.center().y(), 0.0);
        const double orientationSign = circle.normal().z() >= 0.0 ? 1.0 : -1.0;
        const double sweep = segment.lastParameter - segment.firstParameter;

        m_profileSignedArea +=
            0.5 * (cross2D(center, end - start) +
                   circle.radius() * circle.radius() * orientationSign * sweep);

        const MyMath::Vector3 extrema[4] =
        {
            MyMath::Vector3(circle.center().x() + circle.radius(), circle.center().y(), 0.0),
            MyMath::Vector3(circle.center().x() - circle.radius(), circle.center().y(), 0.0),
            MyMath::Vector3(circle.center().x(), circle.center().y() + circle.radius(), 0.0),
            MyMath::Vector3(circle.center().x(), circle.center().y() - circle.radius(), 0.0)
        };

        for (int extremaIndex = 0; extremaIndex < 4; ++extremaIndex)
        {
            if (circleSegmentContainsPoint(circle,
                                           segment.firstParameter,
                                           segment.lastParameter,
                                           extrema[extremaIndex],
                                           m_profileTolerance))
            {
                m_profileBounds.include(extrema[extremaIndex]);
            }
        }
    }

    MYBREP_ASSERT_MESSAGE(m_profileBounds.isValid(),
                          "Geometry_Extruded profile bounds must be valid.");
    MYBREP_ASSERT_MESSAGE(m_profileSignedArea != 0.0,
                          "Geometry_Extruded profile must enclose a non-zero signed area.");

    setLocalBounds(Bounds3(MyMath::Vector3(m_profileBounds.minimum().x(),
                                           m_profileBounds.minimum().y(),
                                           -m_halfHeight),
                           MyMath::Vector3(m_profileBounds.maximum().x(),
                                           m_profileBounds.maximum().y(),
                                           m_halfHeight)));
}

MyMath::Vector3 Geometry_Extruded::segmentStartPoint(const ProfileSegment& segment) const
{
    return segment.curve->pointAt(segment.firstParameter);
}

MyMath::Vector3 Geometry_Extruded::segmentEndPoint(const ProfileSegment& segment) const
{
    return segment.curve->pointAt(segment.lastParameter);
}

/// 母线区域查询

bool Geometry_Extruded::containsProfilePoint(const MyMath::Vector3& point, double tolerance) const
{
    MYBREP_ASSERT_MESSAGE(isFiniteNonNegative(tolerance),
                          "Geometry_Extruded profile query tolerance must be finite and non-negative.");

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
            if (pointSegmentDistanceSquared(point,
                                            segmentStartPoint(segment),
                                            segmentEndPoint(segment)) <= toleranceSquared)
            {
                return true;
            }
        }
        else
        {
            const Geometry_Circle& circle =
                static_cast<const Geometry_Circle&>(*segment.curve);
            const double distanceSquared =
                pointCircleSegmentDistanceSquared(point,
                                                  circle,
                                                  segment.firstParameter,
                                                  segment.lastParameter,
                                                  0.0);

            if (distanceSquared <= toleranceSquared)
            {
                return true;
            }
        }
    }

    double coordinateScale = 1.0;
    coordinateScale = (std::max)(coordinateScale, std::fabs(point.x()));
    coordinateScale = (std::max)(coordinateScale, std::fabs(point.y()));
    coordinateScale = (std::max)(coordinateScale, std::fabs(m_profileBounds.minimum().x()));
    coordinateScale = (std::max)(coordinateScale, std::fabs(m_profileBounds.minimum().y()));
    coordinateScale = (std::max)(coordinateScale, std::fabs(m_profileBounds.maximum().x()));
    coordinateScale = (std::max)(coordinateScale, std::fabs(m_profileBounds.maximum().y()));

    const double queryY =
        point.y() + coordinateScale * std::numeric_limits<double>::epsilon() * NumericalScale; // 避开母线顶点和圆弧水平极值，稳定奇偶射线计数。
    unsigned int intersectionCount = 0;

    for (std::size_t index = 0; index < m_profileSegments.size(); ++index)
    {
        const ProfileSegment& segment = m_profileSegments[index];

        if (segment.curve->kind() == CurveKind::Line)
        {
            const MyMath::Vector3 start = segmentStartPoint(segment);
            const MyMath::Vector3 end = segmentEndPoint(segment);

            if ((start.y() > queryY) == (end.y() > queryY))
            {
                continue;
            }

            const double parameter = (queryY - start.y()) / (end.y() - start.y());
            const double intersectionX = start.x() + (end.x() - start.x()) * parameter;

            if (intersectionX > point.x() + tolerance)
            {
                ++intersectionCount;
            }

            continue;
        }

        const Geometry_Circle& circle =
            static_cast<const Geometry_Circle&>(*segment.curve);
        const double sine = (queryY - circle.center().y()) / circle.radius();

        if (sine < -1.0 || sine > 1.0)
        {
            continue;
        }

        const double clampedSine = (std::max)(-1.0, (std::min)(1.0, sine));
        const double firstWorldAngle = std::asin(clampedSine);
        const double secondWorldAngle = Pi - firstWorldAngle;
        const double worldAngles[2] = {firstWorldAngle, secondWorldAngle};

        for (int candidateIndex = 0; candidateIndex < 2; ++candidateIndex)
        {
            const double worldAngle = worldAngles[candidateIndex];
            const double cosine = std::cos(worldAngle);

            if (std::fabs(cosine) <= std::numeric_limits<double>::epsilon() * NumericalScale)
            {
                continue;
            }

            const MyMath::Vector3 candidate(circle.center().x() + circle.radius() * cosine,
                                            queryY,
                                            0.0);

            if (!circleSegmentContainsPoint(circle,
                                            segment.firstParameter,
                                            segment.lastParameter,
                                            candidate,
                                            tolerance))
            {
                continue;
            }

            if (candidate.x() > point.x() + tolerance)
            {
                ++intersectionCount;
            }
        }
    }

    return (intersectionCount & 1U) != 0;
}

double Geometry_Extruded::profileBoundaryDistance(const MyMath::Vector3& point) const
{
    double minimumDistanceSquared = (std::numeric_limits<double>::infinity)();

    for (std::size_t index = 0; index < m_profileSegments.size(); ++index)
    {
        const ProfileSegment& segment = m_profileSegments[index];
        double distanceSquared = 0.0;

        if (segment.curve->kind() == CurveKind::Line)
        {
            distanceSquared =
                pointSegmentDistanceSquared(point,
                                            segmentStartPoint(segment),
                                            segmentEndPoint(segment));
        }
        else
        {
            const Geometry_Circle& circle =
                static_cast<const Geometry_Circle&>(*segment.curve);

            distanceSquared =
                pointCircleSegmentDistanceSquared(point,
                                                  circle,
                                                  segment.firstParameter,
                                                  segment.lastParameter,
                                                  0.0);
        }

        minimumDistanceSquared = (std::min)(minimumDistanceSquared, distanceSquared);
    }

    MYBREP_ASSERT_MESSAGE(isFiniteValue(minimumDistanceSquared) && minimumDistanceSquared >= 0.0,
                          "Geometry_Extruded failed to resolve a finite profile boundary distance.");

    return std::sqrt(minimumDistanceSquared);
}

ShapeRelation Geometry_Extruded::classifyProfileBounds(const Bounds3& bounds) const
{
    if (!m_profileBounds.intersects(bounds, m_profileTolerance))
    {
        return ShapeRelation::Outside;
    }

    const MyMath::Vector3 corners[4] =
    {
        MyMath::Vector3(bounds.minimum().x(), bounds.minimum().y(), 0.0),
        MyMath::Vector3(bounds.maximum().x(), bounds.minimum().y(), 0.0),
        MyMath::Vector3(bounds.maximum().x(), bounds.maximum().y(), 0.0),
        MyMath::Vector3(bounds.minimum().x(), bounds.maximum().y(), 0.0)
    };

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
            if (lineSegmentIntersectsRectangle(segmentStartPoint(segment),
                                               segmentEndPoint(segment),
                                               bounds,
                                               tolerance))
            {
                boundaryIntersects = true;
                break;
            }

            continue;
        }

        const Geometry_Circle& circle =
            static_cast<const Geometry_Circle&>(*segment.curve);
        const MyMath::Vector3 start = segmentStartPoint(segment);
        const MyMath::Vector3 end = segmentEndPoint(segment);

        if (rectangleContainsPoint(bounds, start, m_profileTolerance) ||
            rectangleContainsPoint(bounds, end, m_profileTolerance))
        {
            boundaryIntersects = true;
            break;
        }

        const double verticalEdges[2] = {bounds.minimum().x(), bounds.maximum().x()};

        for (int edgeIndex = 0; edgeIndex < 2 && !boundaryIntersects; ++edgeIndex)
        {
            const double cosine =
                (verticalEdges[edgeIndex] - circle.center().x()) / circle.radius();

            if (cosine < -1.0 || cosine > 1.0)
            {
                continue;
            }

            const double angle = std::acos((std::max)(-1.0, (std::min)(1.0, cosine)));
            const double candidateY[2] =
            {
                circle.center().y() + circle.radius() * std::sin(angle),
                circle.center().y() - circle.radius() * std::sin(angle)
            };

            for (int candidateIndex = 0; candidateIndex < 2; ++candidateIndex)
            {
                const MyMath::Vector3 candidate(verticalEdges[edgeIndex],
                                                candidateY[candidateIndex],
                                                0.0);

                if (candidate.y() >= bounds.minimum().y() - m_profileTolerance &&
                    candidate.y() <= bounds.maximum().y() + m_profileTolerance &&
                    circleSegmentContainsPoint(circle,
                                               segment.firstParameter,
                                               segment.lastParameter,
                                               candidate,
                                               tolerance))
                {
                    boundaryIntersects = true;
                    break;
                }
            }
        }

        const double horizontalEdges[2] = {bounds.minimum().y(), bounds.maximum().y()};

        for (int edgeIndex = 0; edgeIndex < 2 && !boundaryIntersects; ++edgeIndex)
        {
            const double sine =
                (horizontalEdges[edgeIndex] - circle.center().y()) / circle.radius();

            if (sine < -1.0 || sine > 1.0)
            {
                continue;
            }

            const double angle = std::asin((std::max)(-1.0, (std::min)(1.0, sine)));
            const double candidateX[2] =
            {
                circle.center().x() + circle.radius() * std::cos(angle),
                circle.center().x() - circle.radius() * std::cos(angle)
            };

            for (int candidateIndex = 0; candidateIndex < 2; ++candidateIndex)
            {
                const MyMath::Vector3 candidate(candidateX[candidateIndex],
                                                horizontalEdges[edgeIndex],
                                                0.0);

                if (candidate.x() >= bounds.minimum().x() - m_profileTolerance &&
                    candidate.x() <= bounds.maximum().x() + m_profileTolerance &&
                    circleSegmentContainsPoint(circle,
                                               segment.firstParameter,
                                               segment.lastParameter,
                                               candidate,
                                               tolerance))
                {
                    boundaryIntersects = true;
                    break;
                }
            }
        }

        if (boundaryIntersects)
        {
            break;
        }
    }

    if (allCornersInside && !boundaryIntersects)
    {
        return ShapeRelation::Inside;
    }

    if (boundaryIntersects ||
        anyCornerInside ||
        containsProfilePoint(bounds.center(), m_profileTolerance))
    {
        return ShapeRelation::Intersecting;
    }

    return ShapeRelation::Outside;
}

}