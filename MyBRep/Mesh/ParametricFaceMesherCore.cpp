#include "ParametricFaceMesherCore.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <vector>

namespace
{

const int MaximumAllowedBoundarySubdivisionDepth = 20; // 单条trimming Edge最多2^20个二分区间，避免错误配置造成指数级增长。
const int MaximumAllowedSurfaceSubdivisionRounds = 20; // 共享边一致细分最多传播20轮，避免极端容差导致无限增长。

struct Ring2D
{
    std::vector<MyMath::Vector2> points;
    int depth;
};

struct Triangle2D
{
    Triangle2D()
    {
    }

    Triangle2D(const MyMath::Vector2& firstValue, const MyMath::Vector2& secondValue, const MyMath::Vector2& thirdValue)
        : first(firstValue)
        , second(secondValue)
        , third(thirdValue)
    {
    }

    MyMath::Vector2 first;
    MyMath::Vector2 second;
    MyMath::Vector2 third;
};

struct IndexedTriangle
{
    IndexedTriangle() : first(0), second(0), third(0)
    {
    }

    IndexedTriangle(unsigned int firstValue, unsigned int secondValue, unsigned int thirdValue)
        : first(firstValue)
        , second(secondValue)
        , third(thirdValue)
    {
    }

    unsigned int first;
    unsigned int second;
    unsigned int third;
};

struct EdgeKey
{
    EdgeKey() : first(0), second(0)
    {
    }

    EdgeKey(unsigned int firstValue, unsigned int secondValue)
    {
        if (firstValue < secondValue)
        {
            first = firstValue;
            second = secondValue;
        }
        else
        {
            first = secondValue;
            second = firstValue;
        }
    }

    bool operator<(const EdgeKey& other) const
    {
        return first < other.first || (first == other.first && second < other.second);
    }

    unsigned int first;
    unsigned int second;
};

enum class TrimClassification
{
    Outside,
    Boundary,
    Inside
};

double absoluteValue(double value)
{
    return value >= 0.0 ? value : -value;
}

bool isFiniteValue(double value)
{
    const double infinity = (std::numeric_limits<double>::infinity)();
    return value == value && value != infinity && value != -infinity;
}

double nearestInteger(double value)
{
    return value >= 0.0 ? std::floor(value + 0.5) : std::ceil(value - 0.5);
}

double pointSegmentDistance2D(const MyMath::Vector2& point, const MyMath::Vector2& start, const MyMath::Vector2& end)
{
    const MyMath::Vector2 segment = end - start;
    const double lengthSquared = segment.lengthSquared();

    if (lengthSquared <= 0.0)
    {
        return point.distanceTo(start);
    }

    double parameter = MyMath::Vector2::dot(point - start, segment) / lengthSquared;
    parameter = (std::max)(0.0, (std::min)(1.0, parameter));
    return point.distanceTo(start + segment * parameter);
}

double pointSegmentDistance3D(const MyMath::Vector3& point, const MyMath::Vector3& start, const MyMath::Vector3& end)
{
    const MyMath::Vector3 segment = end - start;
    const double lengthSquared = MyMath::Vector3::dot(segment, segment);

    if (lengthSquared <= 0.0)
    {
        return point.distanceTo(start);
    }

    double parameter = MyMath::Vector3::dot(point - start, segment) / lengthSquared;
    parameter = (std::max)(0.0, (std::min)(1.0, parameter));
    return point.distanceTo(start + segment * parameter);
}

bool pointsEqual(const MyMath::Vector2& first, const MyMath::Vector2& second, double tolerance)
{
    return first.distanceSquaredTo(second) <= tolerance * tolerance;
}

double orientation(const MyMath::Vector2& first, const MyMath::Vector2& second, const MyMath::Vector2& third)
{
    return MyMath::Vector2::cross(second - first, third - first);
}

double signedArea(const std::vector<MyMath::Vector2>& polygon)
{
    double twiceArea = 0.0;

    for (std::size_t index = 0; index < polygon.size(); ++index)
    {
        const MyMath::Vector2& first = polygon[index];
        const MyMath::Vector2& second = polygon[(index + 1) % polygon.size()];
        twiceArea += first.x() * second.y() - second.x() * first.y();
    }

    return twiceArea * 0.5;
}

bool pointOnSegment(const MyMath::Vector2& point, const MyMath::Vector2& first, const MyMath::Vector2& second, double tolerance)
{
    if (pointSegmentDistance2D(point, first, second) > tolerance)
    {
        return false;
    }

    const double minimumX = (std::min)(first.x(), second.x()) - tolerance;
    const double maximumX = (std::max)(first.x(), second.x()) + tolerance;
    const double minimumY = (std::min)(first.y(), second.y()) - tolerance;
    const double maximumY = (std::max)(first.y(), second.y()) + tolerance;

    return point.x() >= minimumX && point.x() <= maximumX && point.y() >= minimumY && point.y() <= maximumY;
}

bool segmentsIntersect(const MyMath::Vector2& firstStart,
                       const MyMath::Vector2& firstEnd,
                       const MyMath::Vector2& secondStart,
                       const MyMath::Vector2& secondEnd,
                       double tolerance)
{
    const double firstA = orientation(firstStart, firstEnd, secondStart);
    const double firstB = orientation(firstStart, firstEnd, secondEnd);
    const double secondA = orientation(secondStart, secondEnd, firstStart);
    const double secondB = orientation(secondStart, secondEnd, firstEnd);

    const bool firstStraddles = (firstA > tolerance && firstB < -tolerance) || (firstA < -tolerance && firstB > tolerance);
    const bool secondStraddles = (secondA > tolerance && secondB < -tolerance) || (secondA < -tolerance && secondB > tolerance);

    if (firstStraddles && secondStraddles)
    {
        return true;
    }

    if (absoluteValue(firstA) <= tolerance && pointOnSegment(secondStart, firstStart, firstEnd, tolerance))
    {
        return true;
    }

    if (absoluteValue(firstB) <= tolerance && pointOnSegment(secondEnd, firstStart, firstEnd, tolerance))
    {
        return true;
    }

    if (absoluteValue(secondA) <= tolerance && pointOnSegment(firstStart, secondStart, secondEnd, tolerance))
    {
        return true;
    }

    return absoluteValue(secondB) <= tolerance && pointOnSegment(firstEnd, secondStart, secondEnd, tolerance);
}

bool pointInRing(const MyMath::Vector2& point, const std::vector<MyMath::Vector2>& ring, double tolerance)
{
    bool inside = false;

    for (std::size_t index = 0; index < ring.size(); ++index)
    {
        const MyMath::Vector2& first = ring[index];
        const MyMath::Vector2& second = ring[(index + 1) % ring.size()];

        if (pointOnSegment(point, first, second, tolerance))
        {
            return true;
        }

        const bool crossesY = (first.y() > point.y()) != (second.y() > point.y());

        if (!crossesY)
        {
            continue;
        }

        const double intersectionX = first.x() + (point.y() - first.y()) * (second.x() - first.x()) / (second.y() - first.y());

        if (intersectionX > point.x())
        {
            inside = !inside;
        }
    }

    return inside;
}

TrimClassification classifyTrim(const MyMath::Vector2& point, const std::vector<Ring2D>& rings, double tolerance)
{
    bool inside = false;

    for (std::size_t ringIndex = 0; ringIndex < rings.size(); ++ringIndex)
    {
        const std::vector<MyMath::Vector2>& ring = rings[ringIndex].points;

        for (std::size_t edgeIndex = 0; edgeIndex < ring.size(); ++edgeIndex)
        {
            if (pointOnSegment(point, ring[edgeIndex], ring[(edgeIndex + 1) % ring.size()], tolerance))
            {
                return TrimClassification::Boundary;
            }
        }

        bool insideCurrentRing = false;

        for (std::size_t edgeIndex = 0; edgeIndex < ring.size(); ++edgeIndex)
        {
            const MyMath::Vector2& first = ring[edgeIndex];
            const MyMath::Vector2& second = ring[(edgeIndex + 1) % ring.size()];
            const bool crossesY = (first.y() > point.y()) != (second.y() > point.y());

            if (!crossesY)
            {
                continue;
            }

            const double intersectionX = first.x() + (point.y() - first.y()) * (second.x() - first.x()) / (second.y() - first.y());

            if (intersectionX > point.x())
            {
                insideCurrentRing = !insideCurrentRing;
            }
        }

        if (insideCurrentRing)
        {
            inside = !inside;
        }
    }

    return inside ? TrimClassification::Inside : TrimClassification::Outside;
}

bool pointStrictlyInTriangle(const MyMath::Vector2& point,
                             const MyMath::Vector2& first,
                             const MyMath::Vector2& second,
                             const MyMath::Vector2& third,
                             double tolerance)
{
    const double firstSide = orientation(first, second, point);
    const double secondSide = orientation(second, third, point);
    const double thirdSide = orientation(third, first, point);

    return firstSide > tolerance && secondSide > tolerance && thirdSide > tolerance;
}

void removeConsecutiveDuplicates(std::vector<MyMath::Vector2>& points, double tolerance)
{
    if (points.empty())
    {
        return;
    }

    std::vector<MyMath::Vector2> filtered;
    filtered.reserve(points.size());
    filtered.push_back(points.front());

    for (std::size_t index = 1; index < points.size(); ++index)
    {
        if (!pointsEqual(points[index], filtered.back(), tolerance))
        {
            filtered.push_back(points[index]);
        }
    }

    if (filtered.size() > 1 && pointsEqual(filtered.front(), filtered.back(), tolerance))
    {
        filtered.pop_back();
    }

    points.swap(filtered);
}

void removeSimpleCollinearPoints(std::vector<MyMath::Vector2>& points, double tolerance)
{
    if (points.size() <= 3)
    {
        return;
    }

    bool changed = true;

    while (changed && points.size() > 3)
    {
        changed = false;

        for (std::size_t index = 0; index < points.size(); ++index)
        {
            const std::size_t previousIndex = (index + points.size() - 1) % points.size();
            const std::size_t nextIndex = (index + 1) % points.size();

            if (absoluteValue(orientation(points[previousIndex], points[index], points[nextIndex])) <= tolerance &&
                pointOnSegment(points[index], points[previousIndex], points[nextIndex], tolerance))
            {
                points.erase(points.begin() + index);
                changed = true;
                break;
            }
        }
    }
}

MyMath::Vector2 surfaceParameter(const MyBRep::Topology_Edge& edge, const MyBRep::Geometry_Surface& surface, double parameter)
{
    return edge.surfaceParameterAt(surface, parameter);
}

MyMath::Vector3 surfacePosition(const MyBRep::Geometry_Surface& surface, const MyMath::Vector2& parameter)
{
    return surface.pointAt(parameter.x(), parameter.y());
}

bool boundaryIntervalFlatEnough(const MyBRep::Topology_Edge& edge,
                                const MyBRep::Geometry_Surface& surface,
                                double firstParameter,
                                double lastParameter,
                                const MyMath::Vector2& firstUV,
                                const MyMath::Vector2& lastUV,
                                double chordTolerance)
{
    const double span = lastParameter - firstParameter;
    const double quarterParameter = firstParameter + span * 0.25; // 1/4、1/2、3/4共同检查，避免S形边界中点恰落弦上的遗漏。
    const double middleParameter = firstParameter + span * 0.5;
    const double threeQuarterParameter = firstParameter + span * 0.75;

    const MyMath::Vector3 firstPosition = surfacePosition(surface, firstUV);
    const MyMath::Vector3 lastPosition = surfacePosition(surface, lastUV);

    return pointSegmentDistance3D(surfacePosition(surface, surfaceParameter(edge, surface, quarterParameter)), firstPosition, lastPosition) <= chordTolerance &&
           pointSegmentDistance3D(surfacePosition(surface, surfaceParameter(edge, surface, middleParameter)), firstPosition, lastPosition) <= chordTolerance &&
           pointSegmentDistance3D(surfacePosition(surface, surfaceParameter(edge, surface, threeQuarterParameter)), firstPosition, lastPosition) <= chordTolerance;
}

void appendAdaptiveBoundaryInterval(const MyBRep::Topology_Edge& edge,
                                    const MyBRep::Geometry_Surface& surface,
                                    double firstParameter,
                                    double lastParameter,
                                    const MyMath::Vector2& firstUV,
                                    const MyMath::Vector2& lastUV,
                                    int depth,
                                    const MyBRep::ParametricFaceMeshOptions& options,
                                    std::vector<MyMath::Vector2>& points)
{
    const bool minimumDepthReached = depth >= options.minimumBoundarySubdivisionDepth;
    const bool maximumDepthReached = depth >= options.maximumBoundarySubdivisionDepth;

    if (maximumDepthReached || (minimumDepthReached &&
        boundaryIntervalFlatEnough(edge, surface, firstParameter, lastParameter, firstUV, lastUV, options.boundaryChordTolerance)))
    {
        points.push_back(lastUV);
        return;
    }

    const double middleParameter = (firstParameter + lastParameter) * 0.5;
    const MyMath::Vector2 middleUV = surfaceParameter(edge, surface, middleParameter);

    appendAdaptiveBoundaryInterval(edge, surface, firstParameter, middleParameter, firstUV, middleUV, depth + 1, options, points);
    appendAdaptiveBoundaryInterval(edge, surface, middleParameter, lastParameter, middleUV, lastUV, depth + 1, options, points);
}

bool sampleEdge(const MyBRep::Topology_Edge& edge,
                const MyBRep::Geometry_Surface& surface,
                const MyBRep::ParametricFaceMeshOptions& options,
                std::vector<MyMath::Vector2>& points)
{
    points.clear();

    if (!edge.isValid() || !edge.hasCurveOnSurface(surface))
    {
        return false;
    }

    const MyMath::Vector2 firstUV = surfaceParameter(edge, surface, 0.0);
    const MyMath::Vector2 lastUV = surfaceParameter(edge, surface, 1.0);
    points.push_back(firstUV);
    appendAdaptiveBoundaryInterval(edge, surface, 0.0, 1.0, firstUV, lastUV, 0, options, points);

    return points.size() >= 2;
}

double parameterCoordinate(const MyMath::Vector2& parameter, int axis)
{
    return axis == 0 ? parameter.x() : parameter.y();
}

void setParameterCoordinate(MyMath::Vector2& parameter, int axis, double value)
{
    if (axis == 0)
    {
        parameter.setX(value);
    }
    else
    {
        parameter.setY(value);
    }
}

void shiftPointsCoordinate(std::vector<MyMath::Vector2>& points, int axis, double shift)
{
    if (shift == 0.0)
    {
        return;
    }

    for (std::size_t index = 0; index < points.size(); ++index)
    {
        setParameterCoordinate(points[index], axis, parameterCoordinate(points[index], axis) + shift);
    }
}

bool alignPeriodicCoordinate(std::vector<MyMath::Vector2>& edgePoints, const MyMath::Vector2& previousPoint, int axis, double period)
{
    if (edgePoints.empty() || period <= 0.0)
    {
        return false;
    }

    const double previousCoordinate = parameterCoordinate(previousPoint, axis);
    const double edgeCoordinate = parameterCoordinate(edgePoints.front(), axis);
    const double shiftCount = nearestInteger((previousCoordinate - edgeCoordinate) / period);
    shiftPointsCoordinate(edgePoints, axis, shiftCount * period);
    return true;
}

bool singularParametersMeet(const MyBRep::Geometry_Surface& surface,
                            const MyMath::Vector2& first,
                            const MyMath::Vector2& second,
                            const MyBRep::ParametricFaceMeshPolicy& policy,
                            double tolerance)
{
    return policy.singularityHandler &&
           policy.singularityHandler->parametersMeetAtSingularity(surface, first, second, tolerance);
}

bool parametersConnect(const MyBRep::Geometry_Surface& surface,
                       const MyMath::Vector2& first,
                       const MyMath::Vector2& second,
                       const MyBRep::ParametricFaceMeshPolicy& policy,
                       double tolerance)
{
    return pointsEqual(first, second, tolerance) ||
           singularParametersMeet(surface, first, second, policy, tolerance);
}

bool alignEdgeSampleToPrevious(std::vector<MyMath::Vector2>& edgePoints,
                               const MyMath::Vector2& previousPoint,
                               const MyBRep::Geometry_Surface& surface,
                               const MyBRep::ParametricFaceMeshPolicy& policy,
                               double tolerance)
{
    if (edgePoints.empty())
    {
        return false;
    }

    // 参数奇点处不同周期坐标可以对应同一三维拓扑点，此时必须保留原始UV差异，不能先执行周期平移。
    if (parametersConnect(surface, previousPoint, edgePoints.front(), policy, tolerance))
    {
        return true;
    }

    if (policy.periodicU && !alignPeriodicCoordinate(edgePoints, previousPoint, 0, surface.uPeriod()))
    {
        return false;
    }

    if (policy.periodicV && !alignPeriodicCoordinate(edgePoints, previousPoint, 1, surface.vPeriod()))
    {
        return false;
    }

    return parametersConnect(surface, previousPoint, edgePoints.front(), policy, tolerance);
}

bool sampleWire(const MyBRep::Topology_Wire& wire,
                const MyBRep::Geometry_Surface& surface,
                const MyBRep::ParametricFaceMeshOptions& options,
                const MyBRep::ParametricFaceMeshPolicy& policy,
                std::vector<MyMath::Vector2>& points)
{
    points.clear();

    for (std::size_t edgeIndex = 0; edgeIndex < wire.edgeCount(); ++edgeIndex)
    {
        std::vector<MyMath::Vector2> edgePoints;

        if (!sampleEdge(wire.edge(edgeIndex), surface, options, edgePoints))
        {
            return false;
        }

        if (!points.empty())
        {
            if (!alignEdgeSampleToPrevious(edgePoints, points.back(), surface, policy, options.geometricTolerance))
            {
                return false;
            }

            // 普通连接点在两个Edge中是同一个UV，只保留一份。
            // 参数奇点处允许不同UV映射到同一个三维拓扑点，此时必须同时保留两侧UV，
            // 否则会吞掉pole/seam的一侧参数端点，破坏完整参数域及后续pole fan三角化。
            if (pointsEqual(points.back(), edgePoints.front(), options.geometricTolerance))
            {
                edgePoints.erase(edgePoints.begin());
            }
        }

        points.insert(points.end(), edgePoints.begin(), edgePoints.end());
    }

    if (points.size() < 4)
    {
        return false;
    }

    if (!alignEdgeSampleToPrevious(points, points.back(), surface, policy, options.geometricTolerance))
    {
        return false;
    }

    if (!parametersConnect(surface, points.front(), points.back(), policy, options.geometricTolerance))
    {
        return false;
    }

    removeConsecutiveDuplicates(points, options.geometricTolerance);
    removeSimpleCollinearPoints(points, options.geometricTolerance);

    return points.size() >= 3 && absoluteValue(signedArea(points)) > options.geometricTolerance * options.geometricTolerance;
}

double ringCenterCoordinate(const Ring2D& ring, int axis)
{
    double sum = 0.0;

    for (std::size_t index = 0; index < ring.points.size(); ++index)
    {
        sum += parameterCoordinate(ring.points[index], axis);
    }

    return sum / static_cast<double>(ring.points.size());
}

double ringCoordinateSpan(const Ring2D& ring, int axis)
{
    double minimum = parameterCoordinate(ring.points[0], axis);
    double maximum = minimum;

    for (std::size_t index = 1; index < ring.points.size(); ++index)
    {
        const double coordinate = parameterCoordinate(ring.points[index], axis);
        minimum = (std::min)(minimum, coordinate);
        maximum = (std::max)(maximum, coordinate);
    }

    return maximum - minimum;
}

bool normalizeRingPeriods(std::vector<Ring2D>& rings, int axis, double period, double tolerance)
{
    if (rings.empty() || period <= 0.0)
    {
        return false;
    }

    std::size_t referenceIndex = 0;
    double referenceArea = absoluteValue(signedArea(rings[0].points));

    for (std::size_t index = 1; index < rings.size(); ++index)
    {
        const double area = absoluteValue(signedArea(rings[index].points));

        if (area > referenceArea)
        {
            referenceArea = area;
            referenceIndex = index;
        }
    }

    const double referenceCenter = ringCenterCoordinate(rings[referenceIndex], axis);

    for (std::size_t index = 0; index < rings.size(); ++index)
    {
        if (ringCoordinateSpan(rings[index], axis) > period + tolerance)
        {
            return false;
        }

        if (index == referenceIndex)
        {
            continue;
        }

        const double currentCenter = ringCenterCoordinate(rings[index], axis);
        const double shiftCount = nearestInteger((referenceCenter - currentCenter) / period);
        shiftPointsCoordinate(rings[index].points, axis, shiftCount * period);
    }

    double globalMinimum = parameterCoordinate(rings[0].points[0], axis);
    double globalMaximum = globalMinimum;

    for (std::size_t ringIndex = 0; ringIndex < rings.size(); ++ringIndex)
    {
        for (std::size_t pointIndex = 0; pointIndex < rings[ringIndex].points.size(); ++pointIndex)
        {
            const double coordinate = parameterCoordinate(rings[ringIndex].points[pointIndex], axis);
            globalMinimum = (std::min)(globalMinimum, coordinate);
            globalMaximum = (std::max)(globalMaximum, coordinate);
        }
    }

    return globalMaximum - globalMinimum <= period + tolerance;
}

void calculateRingDepths(std::vector<Ring2D>& rings, double tolerance)
{
    for (std::size_t ringIndex = 0; ringIndex < rings.size(); ++ringIndex)
    {
        int depth = 0;
        const MyMath::Vector2 testPoint = rings[ringIndex].points[0];

        for (std::size_t otherIndex = 0; otherIndex < rings.size(); ++otherIndex)
        {
            if (ringIndex == otherIndex)
            {
                continue;
            }

            if (pointInRing(testPoint, rings[otherIndex].points, tolerance))
            {
                ++depth;
            }
        }

        rings[ringIndex].depth = depth;
    }
}

void orientRing(std::vector<MyMath::Vector2>& ring, bool counterClockwise)
{
    const bool currentlyCounterClockwise = signedArea(ring) > 0.0;

    if (currentlyCounterClockwise != counterClockwise)
    {
        std::reverse(ring.begin(), ring.end());
    }
}

std::size_t rightmostVertex(const std::vector<MyMath::Vector2>& ring)
{
    std::size_t result = 0;

    for (std::size_t index = 1; index < ring.size(); ++index)
    {
        if (ring[index].x() > ring[result].x() ||
            (ring[index].x() == ring[result].x() && ring[index].y() < ring[result].y()))
        {
            result = index;
        }
    }

    return result;
}

bool bridgeCrossesPolygon(const MyMath::Vector2& first,
                          const MyMath::Vector2& second,
                          const std::vector<MyMath::Vector2>& polygon,
                          double tolerance)
{
    for (std::size_t index = 0; index < polygon.size(); ++index)
    {
        const MyMath::Vector2& edgeStart = polygon[index];
        const MyMath::Vector2& edgeEnd = polygon[(index + 1) % polygon.size()];

        if (pointsEqual(edgeStart, first, tolerance) ||
            pointsEqual(edgeEnd, first, tolerance) ||
            pointsEqual(edgeStart, second, tolerance) ||
            pointsEqual(edgeEnd, second, tolerance))
        {
            continue;
        }

        if (segmentsIntersect(first, second, edgeStart, edgeEnd, tolerance))
        {
            return true;
        }
    }

    return false;
}

bool bridgeCrossesRing(const MyMath::Vector2& first,
                       const MyMath::Vector2& second,
                       const std::vector<MyMath::Vector2>& ring,
                       double tolerance)
{
    for (std::size_t index = 0; index < ring.size(); ++index)
    {
        const MyMath::Vector2& edgeStart = ring[index];
        const MyMath::Vector2& edgeEnd = ring[(index + 1) % ring.size()];

        if (pointsEqual(edgeStart, first, tolerance) ||
            pointsEqual(edgeEnd, first, tolerance) ||
            pointsEqual(edgeStart, second, tolerance) ||
            pointsEqual(edgeEnd, second, tolerance))
        {
            continue;
        }

        if (segmentsIntersect(first, second, edgeStart, edgeEnd, tolerance))
        {
            return true;
        }
    }

    return false;
}

bool findBridge(const std::vector<Ring2D>& allRings,
                const std::vector<MyMath::Vector2>& polygon,
                const std::vector<std::vector<MyMath::Vector2> >& remainingHoles,
                std::size_t currentHoleIndex,
                std::size_t holeVertexIndex,
                double tolerance,
                std::size_t& polygonVertexIndex)
{
    const MyMath::Vector2 holePoint = remainingHoles[currentHoleIndex][holeVertexIndex];

    bool found = false;
    double bestDistanceSquared = 0.0;

    for (std::size_t candidateIndex = 0; candidateIndex < polygon.size(); ++candidateIndex)
    {
        const MyMath::Vector2& candidate = polygon[candidateIndex];

        if (pointsEqual(holePoint, candidate, tolerance))
        {
            continue;
        }

        if (bridgeCrossesPolygon(holePoint, candidate, polygon, tolerance))
        {
            continue;
        }

        bool crossesHole = false;

        for (std::size_t holeIndex = currentHoleIndex; holeIndex < remainingHoles.size(); ++holeIndex)
        {
            if (bridgeCrossesRing(holePoint, candidate, remainingHoles[holeIndex], tolerance))
            {
                crossesHole = true;
                break;
            }
        }

        if (crossesHole)
        {
            continue;
        }

        const MyMath::Vector2 middle = (holePoint + candidate) * 0.5;

        if (classifyTrim(middle, allRings, tolerance) != TrimClassification::Inside)
        {
            continue;
        }

        const double distanceSquared = holePoint.distanceSquaredTo(candidate);

        if (!found || distanceSquared < bestDistanceSquared)
        {
            found = true;
            bestDistanceSquared = distanceSquared;
            polygonVertexIndex = candidateIndex;
        }
    }

    return found;
}

void stitchHole(std::vector<MyMath::Vector2>& polygon,
                std::size_t polygonVertexIndex,
                const std::vector<MyMath::Vector2>& hole,
                std::size_t holeVertexIndex)
{
    std::vector<MyMath::Vector2> merged;
    merged.reserve(polygon.size() + hole.size() + 2);

    for (std::size_t index = 0; index <= polygonVertexIndex; ++index)
    {
        merged.push_back(polygon[index]);
    }

    for (std::size_t offset = 0; offset < hole.size(); ++offset)
    {
        merged.push_back(hole[(holeVertexIndex + offset) % hole.size()]);
    }

    merged.push_back(hole[holeVertexIndex]);
    merged.push_back(polygon[polygonVertexIndex]);

    for (std::size_t index = polygonVertexIndex + 1; index < polygon.size(); ++index)
    {
        merged.push_back(polygon[index]);
    }

    polygon.swap(merged);
}

bool isEar(const std::vector<MyMath::Vector2>& polygon, std::size_t index, double tolerance)
{
    const std::size_t previousIndex = (index + polygon.size() - 1) % polygon.size();
    const std::size_t nextIndex = (index + 1) % polygon.size();

    const MyMath::Vector2& previous = polygon[previousIndex];
    const MyMath::Vector2& current = polygon[index];
    const MyMath::Vector2& next = polygon[nextIndex];

    if (orientation(previous, current, next) <= tolerance)
    {
        return false;
    }

    for (std::size_t testIndex = 0; testIndex < polygon.size(); ++testIndex)
    {
        if (testIndex == previousIndex || testIndex == index || testIndex == nextIndex)
        {
            continue;
        }

        const MyMath::Vector2& testPoint = polygon[testIndex];

        if (pointsEqual(testPoint, previous, tolerance) ||
            pointsEqual(testPoint, current, tolerance) ||
            pointsEqual(testPoint, next, tolerance))
        {
            continue;
        }

        if (pointStrictlyInTriangle(testPoint, previous, current, next, tolerance))
        {
            return false;
        }
    }

    return true;
}

bool removeOneDegenerateVertex(std::vector<MyMath::Vector2>& polygon, double tolerance)
{
    if (polygon.size() <= 3)
    {
        return false;
    }

    for (std::size_t index = 0; index < polygon.size(); ++index)
    {
        const std::size_t previousIndex = (index + polygon.size() - 1) % polygon.size();
        const std::size_t nextIndex = (index + 1) % polygon.size();

        if (pointsEqual(polygon[index], polygon[previousIndex], tolerance) ||
            pointsEqual(polygon[index], polygon[nextIndex], tolerance))
        {
            polygon.erase(polygon.begin() + index);
            return true;
        }

        if (absoluteValue(orientation(polygon[previousIndex], polygon[index], polygon[nextIndex])) <= tolerance &&
            pointOnSegment(polygon[index], polygon[previousIndex], polygon[nextIndex], tolerance))
        {
            polygon.erase(polygon.begin() + index);
            return true;
        }
    }

    return false;
}

bool earClip(std::vector<MyMath::Vector2> polygon, double tolerance, std::vector<Triangle2D>& triangles)
{
    removeConsecutiveDuplicates(polygon, tolerance);

    if (polygon.size() < 3)
    {
        return false;
    }

    if (signedArea(polygon) < 0.0)
    {
        std::reverse(polygon.begin(), polygon.end());
    }

    std::size_t guard = 0;
    const std::size_t guardLimit = polygon.size() * polygon.size() * 4; // 弱简单多边形耳切使用二次规模保护，避免异常输入无限循环。

    while (polygon.size() > 3 && guard < guardLimit)
    {
        bool clipped = false;

        for (std::size_t index = 0; index < polygon.size(); ++index)
        {
            if (!isEar(polygon, index, tolerance))
            {
                continue;
            }

            const std::size_t previousIndex = (index + polygon.size() - 1) % polygon.size();
            const std::size_t nextIndex = (index + 1) % polygon.size();

            triangles.push_back(Triangle2D(polygon[previousIndex], polygon[index], polygon[nextIndex]));
            polygon.erase(polygon.begin() + index);
            clipped = true;
            break;
        }

        if (!clipped && !removeOneDegenerateVertex(polygon, tolerance))
        {
            return false;
        }

        ++guard;
    }

    if (polygon.size() != 3 || orientation(polygon[0], polygon[1], polygon[2]) <= tolerance)
    {
        return false;
    }

    triangles.push_back(Triangle2D(polygon[0], polygon[1], polygon[2]));
    return true;
}

bool triangulateFilledRing(const std::vector<Ring2D>& allRings,
                           const Ring2D& outerRing,
                           const std::vector<const Ring2D*>& holeRings,
                           double tolerance,
                           std::vector<Triangle2D>& triangles)
{
    std::vector<MyMath::Vector2> polygon = outerRing.points;
    orientRing(polygon, true);

    std::vector<std::vector<MyMath::Vector2> > holes;
    holes.reserve(holeRings.size());

    for (std::size_t index = 0; index < holeRings.size(); ++index)
    {
        std::vector<MyMath::Vector2> hole = holeRings[index]->points;
        orientRing(hole, false);
        holes.push_back(hole);
    }

    std::sort(holes.begin(), holes.end(),
              [](const std::vector<MyMath::Vector2>& first, const std::vector<MyMath::Vector2>& second)
    {
        return first[rightmostVertex(first)].x() > second[rightmostVertex(second)].x();
    });

    for (std::size_t holeIndex = 0; holeIndex < holes.size(); ++holeIndex)
    {
        const std::size_t holeVertexIndex = rightmostVertex(holes[holeIndex]);
        std::size_t polygonVertexIndex = 0;

        if (!findBridge(allRings, polygon, holes, holeIndex, holeVertexIndex, tolerance, polygonVertexIndex))
        {
            return false;
        }

        stitchHole(polygon, polygonVertexIndex, holes[holeIndex], holeVertexIndex);
    }

    return earClip(polygon, tolerance, triangles);
}

unsigned int findOrAppendParameterVertex(const MyMath::Vector2& parameter,
                                         double tolerance,
                                         std::vector<MyMath::Vector2>& vertices)
{
    for (std::size_t index = 0; index < vertices.size(); ++index)
    {
        if (pointsEqual(parameter, vertices[index], tolerance))
        {
            return static_cast<unsigned int>(index);
        }
    }

    vertices.push_back(parameter);
    return static_cast<unsigned int>(vertices.size() - 1);
}

bool buildIndexedTriangles(const std::vector<Triangle2D>& triangles,
                           double tolerance,
                           std::vector<MyMath::Vector2>& vertices,
                           std::vector<IndexedTriangle>& indexedTriangles)
{
    vertices.clear();
    indexedTriangles.clear();

    for (std::size_t index = 0; index < triangles.size(); ++index)
    {
        const Triangle2D& triangle = triangles[index];
        const unsigned int first = findOrAppendParameterVertex(triangle.first, tolerance, vertices);
        const unsigned int second = findOrAppendParameterVertex(triangle.second, tolerance, vertices);
        const unsigned int third = findOrAppendParameterVertex(triangle.third, tolerance, vertices);

        if (first == second || second == third || third == first)
        {
            return false;
        }

        indexedTriangles.push_back(IndexedTriangle(first, second, third));
    }

    return !vertices.empty() && !indexedTriangles.empty();
}

bool parameterIsRegular(const MyBRep::Geometry_Surface& surface, const MyMath::Vector2& parameter)
{
    const MyMath::Vector3 derivativeU = surface.firstDerivativeUAt(parameter.x(), parameter.y());
    const MyMath::Vector3 derivativeV = surface.firstDerivativeVAt(parameter.x(), parameter.y());

    if (!derivativeU.isFinite() || !derivativeV.isFinite())
    {
        return false;
    }

    return MyMath::Vector3::cross(derivativeU, derivativeV).isVector(0.0);
}

bool parameterIsSingular(const MyBRep::Geometry_Surface& surface,
                         const MyMath::Vector2& parameter,
                         const MyBRep::ParametricFaceMeshPolicy& policy,
                         double tolerance)
{
    if (policy.singularityHandler)
    {
        return policy.singularityHandler->isSingular(surface, parameter, tolerance);
    }

    return !parameterIsRegular(surface, parameter);
}

double surfaceEdgeChordError(const MyBRep::Geometry_Surface& surface,
                             const std::vector<MyMath::Vector2>& vertices,
                             const EdgeKey& edge)
{
    const MyMath::Vector2 firstParameter = vertices[edge.first];
    const MyMath::Vector2 secondParameter = vertices[edge.second];
    const MyMath::Vector2 middleParameter = (firstParameter + secondParameter) * 0.5;

    const MyMath::Vector3 first = surfacePosition(surface, firstParameter);
    const MyMath::Vector3 second = surfacePosition(surface, secondParameter);
    const MyMath::Vector3 middle = surfacePosition(surface, middleParameter);

    return pointSegmentDistance3D(middle, first, second);
}

void collectTriangleEdges(const IndexedTriangle& triangle, std::set<EdgeKey>& edges)
{
    edges.insert(EdgeKey(triangle.first, triangle.second));
    edges.insert(EdgeKey(triangle.second, triangle.third));
    edges.insert(EdgeKey(triangle.third, triangle.first));
}

int splitEdgeCount(const IndexedTriangle& triangle, const std::set<EdgeKey>& splitEdges)
{
    int count = 0;

    if (splitEdges.find(EdgeKey(triangle.first, triangle.second)) != splitEdges.end())
    {
        ++count;
    }

    if (splitEdges.find(EdgeKey(triangle.second, triangle.third)) != splitEdges.end())
    {
        ++count;
    }

    if (splitEdges.find(EdgeKey(triangle.third, triangle.first)) != splitEdges.end())
    {
        ++count;
    }

    return count;
}

void addMissingThirdEdgeForTwoSplitTriangle(const IndexedTriangle& triangle, std::set<EdgeKey>& splitEdges)
{
    const EdgeKey edgeAB(triangle.first, triangle.second);
    const EdgeKey edgeBC(triangle.second, triangle.third);
    const EdgeKey edgeCA(triangle.third, triangle.first);
    const bool splitAB = splitEdges.find(edgeAB) != splitEdges.end();
    const bool splitBC = splitEdges.find(edgeBC) != splitEdges.end();
    const bool splitCA = splitEdges.find(edgeCA) != splitEdges.end();
    const int count = static_cast<int>(splitAB) + static_cast<int>(splitBC) + static_cast<int>(splitCA);

    if (count != 2)
    {
        return;
    }

    if (!splitAB)
    {
        splitEdges.insert(edgeAB);
    }
    else if (!splitBC)
    {
        splitEdges.insert(edgeBC);
    }
    else
    {
        splitEdges.insert(edgeCA);
    }
}

unsigned int midpointVertex(const EdgeKey& edge,
                            std::vector<MyMath::Vector2>& vertices,
                            std::map<EdgeKey, unsigned int>& midpointIndices)
{
    std::map<EdgeKey, unsigned int>::const_iterator found = midpointIndices.find(edge);

    if (found != midpointIndices.end())
    {
        return found->second;
    }

    const MyMath::Vector2 midpoint = (vertices[edge.first] + vertices[edge.second]) * 0.5;
    const unsigned int index = static_cast<unsigned int>(vertices.size());
    vertices.push_back(midpoint);
    midpointIndices[edge] = index;
    return index;
}

bool refineSurfaceOnce(const MyBRep::Geometry_Surface& surface,
                       double tolerance,
                       std::vector<MyMath::Vector2>& vertices,
                       std::vector<IndexedTriangle>& triangles,
                       bool& changed)
{
    changed = false;
    std::set<EdgeKey> allEdges;

    for (std::size_t index = 0; index < triangles.size(); ++index)
    {
        collectTriangleEdges(triangles[index], allEdges);
    }

    std::set<EdgeKey> splitEdges;

    for (std::set<EdgeKey>::const_iterator it = allEdges.begin(); it != allEdges.end(); ++it)
    {
        if (surfaceEdgeChordError(surface, vertices, *it) > tolerance)
        {
            splitEdges.insert(*it);
        }
    }

    if (splitEdges.empty())
    {
        return true;
    }

    // 与已验证的Sphere/Cylinder/Cone细分策略保持一致：
    // 一个Triangle若恰有两条边需要切分，则补切第三边，并把新增切分向邻接Triangle传播到稳定。
    // 稳定后每个Triangle只会出现0、1或3条切分边，避免2-edge分裂形成偏斜三角形。
    bool propagationChanged = true;

    while (propagationChanged)
    {
        propagationChanged = false;

        for (std::size_t index = 0; index < triangles.size(); ++index)
        {
            if (splitEdgeCount(triangles[index], splitEdges) != 2)
            {
                continue;
            }

            const std::size_t before = splitEdges.size();
            addMissingThirdEdgeForTwoSplitTriangle(triangles[index], splitEdges);

            if (splitEdges.size() != before)
            {
                propagationChanged = true;
            }
        }
    }

    changed = true;
    std::map<EdgeKey, unsigned int> midpointIndices;
    std::vector<IndexedTriangle> refined;
    refined.reserve(triangles.size() * 4);

    for (std::size_t index = 0; index < triangles.size(); ++index)
    {
        const IndexedTriangle& triangle = triangles[index];
        const EdgeKey edgeAB(triangle.first, triangle.second);
        const EdgeKey edgeBC(triangle.second, triangle.third);
        const EdgeKey edgeCA(triangle.third, triangle.first);
        const bool splitAB = splitEdges.find(edgeAB) != splitEdges.end();
        const bool splitBC = splitEdges.find(edgeBC) != splitEdges.end();
        const bool splitCA = splitEdges.find(edgeCA) != splitEdges.end();
        const int splitCount = static_cast<int>(splitAB) + static_cast<int>(splitBC) + static_cast<int>(splitCA);

        if (splitCount == 0)
        {
            refined.push_back(triangle);
            continue;
        }

        const unsigned int a = triangle.first;
        const unsigned int b = triangle.second;
        const unsigned int c = triangle.third;

        if (splitCount == 1)
        {
            if (splitAB)
            {
                const unsigned int ab = midpointVertex(edgeAB, vertices, midpointIndices);
                refined.push_back(IndexedTriangle(a, ab, c));
                refined.push_back(IndexedTriangle(ab, b, c));
            }
            else if (splitBC)
            {
                const unsigned int bc = midpointVertex(edgeBC, vertices, midpointIndices);
                refined.push_back(IndexedTriangle(a, b, bc));
                refined.push_back(IndexedTriangle(a, bc, c));
            }
            else
            {
                const unsigned int ca = midpointVertex(edgeCA, vertices, midpointIndices);
                refined.push_back(IndexedTriangle(a, b, ca));
                refined.push_back(IndexedTriangle(b, c, ca));
            }

            continue;
        }

        if (splitCount != 3)
        {
            return false;
        }

        const unsigned int ab = midpointVertex(edgeAB, vertices, midpointIndices);
        const unsigned int bc = midpointVertex(edgeBC, vertices, midpointIndices);
        const unsigned int ca = midpointVertex(edgeCA, vertices, midpointIndices);

        refined.push_back(IndexedTriangle(a, ab, ca));
        refined.push_back(IndexedTriangle(ab, b, bc));
        refined.push_back(IndexedTriangle(ca, bc, c));
        refined.push_back(IndexedTriangle(ab, bc, ca));
    }

    triangles.swap(refined);
    return true;
}

bool surfaceNeedsMoreRefinement(const MyBRep::Geometry_Surface& surface,
                                double tolerance,
                                const std::vector<MyMath::Vector2>& vertices,
                                const std::vector<IndexedTriangle>& triangles)
{
    std::set<EdgeKey> edges;

    for (std::size_t index = 0; index < triangles.size(); ++index)
    {
        collectTriangleEdges(triangles[index], edges);
    }

    for (std::set<EdgeKey>::const_iterator it = edges.begin(); it != edges.end(); ++it)
    {
        if (surfaceEdgeChordError(surface, vertices, *it) > tolerance)
        {
            return true;
        }
    }

    return false;
}

bool refineSurface(const MyBRep::Geometry_Surface& surface,
                   const MyBRep::ParametricFaceMeshOptions& options,
                   std::vector<MyMath::Vector2>& vertices,
                   std::vector<IndexedTriangle>& triangles)
{
    for (int round = 0; round < options.maximumSurfaceSubdivisionRounds; ++round)
    {
        bool changed = false;

        if (!refineSurfaceOnce(surface, options.surfaceChordTolerance, vertices, triangles, changed))
        {
            return false;
        }

        if (!changed)
        {
            return true;
        }
    }

    return !surfaceNeedsMoreRefinement(surface, options.surfaceChordTolerance, vertices, triangles);
}

bool trianglePositionsAreNonDegenerate(const MyBRep::Geometry_Surface& surface,
                                       const MyMath::Vector2& firstParameter,
                                       const MyMath::Vector2& secondParameter,
                                       const MyMath::Vector2& thirdParameter)
{
    const MyMath::Vector3 first = surfacePosition(surface, firstParameter);
    const MyMath::Vector3 second = surfacePosition(surface, secondParameter);
    const MyMath::Vector3 third = surfacePosition(surface, thirdParameter);
    return MyMath::Vector3::cross(second - first, third - first).isVector(0.0);
}

bool triangleUsesVertex(const IndexedTriangle& triangle, unsigned int vertexIndex)
{
    return triangle.first == vertexIndex || triangle.second == vertexIndex || triangle.third == vertexIndex;
}

bool findRegularApproachParameter(unsigned int singularVertexIndex,
                                  const MyBRep::Geometry_Surface& surface,
                                  const std::vector<MyMath::Vector2>& parameterVertices,
                                  const std::vector<IndexedTriangle>& triangles,
                                  const MyBRep::ParametricFaceMeshPolicy& policy,
                                  double tolerance,
                                  MyMath::Vector2& regularApproachParameter)
{
    for (std::size_t triangleIndex = 0; triangleIndex < triangles.size(); ++triangleIndex)
    {
        const IndexedTriangle& triangle = triangles[triangleIndex];

        if (!triangleUsesVertex(triangle, singularVertexIndex))
        {
            continue;
        }

        const unsigned int candidates[3] = {triangle.first, triangle.second, triangle.third};

        for (int candidateIndex = 0; candidateIndex < 3; ++candidateIndex)
        {
            const unsigned int index = candidates[candidateIndex];

            if (index == singularVertexIndex)
            {
                continue;
            }

            if (!parameterIsSingular(surface, parameterVertices[index], policy, tolerance))
            {
                regularApproachParameter = parameterVertices[index];
                return true;
            }
        }
    }

    return false;
}

bool triangleContainsSingularParameter(const IndexedTriangle& triangle,
                                       const MyBRep::Geometry_Surface& surface,
                                       const std::vector<MyMath::Vector2>& parameterVertices,
                                       const MyBRep::ParametricFaceMeshPolicy& policy,
                                       double tolerance)
{
    return parameterIsSingular(surface, parameterVertices[triangle.first], policy, tolerance) ||
           parameterIsSingular(surface, parameterVertices[triangle.second], policy, tolerance) ||
           parameterIsSingular(surface, parameterVertices[triangle.third], policy, tolerance);
}

bool evaluateMeshNormal(const MyBRep::Topology_Face& face,
                        unsigned int vertexIndex,
                        const std::vector<MyMath::Vector2>& parameterVertices,
                        const std::vector<IndexedTriangle>& triangles,
                        const MyBRep::ParametricFaceMeshPolicy& policy,
                        double tolerance,
                        MyMath::Vector3& normal)
{
    const MyMath::Vector2& parameter = parameterVertices[vertexIndex];

    if (!parameterIsSingular(face.geometry(), parameter, policy, tolerance))
    {
        normal = face.normalAt(parameter.x(), parameter.y());
        return normal.isUnit();
    }

    if (policy.rejectSingularParameters || !policy.singularityHandler)
    {
        return false;
    }

    MyMath::Vector2 regularApproachParameter;

    if (!findRegularApproachParameter(
            vertexIndex, face.geometry(), parameterVertices, triangles, policy, tolerance, regularApproachParameter))
    {
        return false;
    }

    return policy.singularityHandler->normalAt(
               face, parameter, regularApproachParameter, tolerance, normal) &&
           normal.isUnit();
}

MyBRep::FaceMesh buildFaceMesh(const MyBRep::Topology_Face& face,
                               const std::vector<MyMath::Vector2>& parameterVertices,
                               const std::vector<IndexedTriangle>& triangles,
                               const MyBRep::ParametricFaceMeshPolicy& policy,
                               double tolerance)
{
    MyBRep::FaceMesh result;

    for (std::size_t index = 0; index < parameterVertices.size(); ++index)
    {
        const MyMath::Vector2& parameter = parameterVertices[index];
        const MyMath::Vector3 position = face.geometry().pointAt(parameter.x(), parameter.y());
        MyMath::Vector3 normal;

        if (!evaluateMeshNormal(
                face, static_cast<unsigned int>(index), parameterVertices, triangles, policy, tolerance, normal))
        {
            return MyBRep::FaceMesh();
        }

        result.addVertex(MyBRep::FaceMeshVertex(parameter, position, normal));
    }

    for (std::size_t index = 0; index < triangles.size(); ++index)
    {
        const IndexedTriangle& triangle = triangles[index];

        if (!trianglePositionsAreNonDegenerate(face.geometry(),
                                              parameterVertices[triangle.first],
                                              parameterVertices[triangle.second],
                                              parameterVertices[triangle.third]))
        {
            // 允许的参数奇点会把一条UV边压缩为同一个三维点；只丢弃与已识别奇点相邻的退化三角形。
            if (!policy.rejectSingularParameters &&
                policy.singularityHandler &&
                triangleContainsSingularParameter(
                    triangle, face.geometry(), parameterVertices, policy, tolerance))
            {
                continue;
            }

            return MyBRep::FaceMesh();
        }

        if (face.isForward())
        {
            result.addTriangle(triangle.first, triangle.second, triangle.third);
        }
        else
        {
            result.addTriangle(triangle.first, triangle.third, triangle.second);
        }
    }

    return result.isValid() ? result : MyBRep::FaceMesh();
}

}

namespace MyBRep
{

ParametricFaceMeshOptions::ParametricFaceMeshOptions()
    : boundaryChordTolerance(1.0e-3)                      // 默认1e-3模型单位边界弦误差。
    , surfaceChordTolerance(1.0e-3)                       // 默认1e-3模型单位共享边弦高误差。
    , geometricTolerance(MyMath::Vector2::DefaultEpsilon) // UV拓扑计算沿用MyMath二维基础比较误差。
    , minimumBoundarySubdivisionDepth(2)                 // trimming Edge至少四等分后再允许停止。
    , maximumBoundarySubdivisionDepth(12)                // 单Edge最多4096个二分区间。
    , maximumSurfaceSubdivisionRounds(12)                // 最多12轮共享边一致细分。
{
}

bool ParametricFaceMeshOptions::isValid() const
{
    return isFiniteValue(boundaryChordTolerance) && boundaryChordTolerance > 0.0 &&
           isFiniteValue(surfaceChordTolerance) && surfaceChordTolerance > 0.0 &&
           isFiniteValue(geometricTolerance) && geometricTolerance > 0.0 &&
           minimumBoundarySubdivisionDepth >= 0 &&
           maximumBoundarySubdivisionDepth >= minimumBoundarySubdivisionDepth &&
           maximumBoundarySubdivisionDepth <= MaximumAllowedBoundarySubdivisionDepth &&
           maximumSurfaceSubdivisionRounds > 0 &&
           maximumSurfaceSubdivisionRounds <= MaximumAllowedSurfaceSubdivisionRounds;
}

ParametricFaceMeshPolicy::ParametricFaceMeshPolicy()
    : periodicU(false)
    , periodicV(false)
    , rejectSingularParameters(true)
    , singularityHandler(0)
{
}

bool ParametricFaceMesherCore::canMesh(const Topology_Face& face, const ParametricFaceMeshPolicy& policy)
{
    if (!face.isValid() || face.wireCount() == 0)
    {
        return false;
    }

    const Geometry_Surface& surface = face.geometry();

    if (policy.periodicU != surface.isUPeriodic())
    {
        return false;
    }

    if (policy.periodicV != surface.isVPeriodic())
    {
        return false;
    }

    if (policy.periodicU && surface.uPeriod() <= 0.0)
    {
        return false;
    }

    if (policy.periodicV && surface.vPeriod() <= 0.0)
    {
        return false;
    }

    for (std::size_t wireIndex = 0; wireIndex < face.wireCount(); ++wireIndex)
    {
        const Topology_Wire wire = face.wire(wireIndex);

        if (!wire.isValid() || !wire.isClosed())
        {
            return false;
        }

        for (std::size_t edgeIndex = 0; edgeIndex < wire.edgeCount(); ++edgeIndex)
        {
            if (!wire.edge(edgeIndex).hasCurveOnSurface(surface))
            {
                return false;
            }
        }
    }

    return true;
}

FaceMesh ParametricFaceMesherCore::mesh(const Topology_Face& face,
                                        const ParametricFaceMeshOptions& options,
                                        const ParametricFaceMeshPolicy& policy)
{
    if (!canMesh(face, policy) || !options.isValid())
    {
        return FaceMesh();
    }

    const Geometry_Surface& surface = face.geometry();
    std::vector<Ring2D> rings;
    rings.reserve(face.wireCount());

    for (std::size_t wireIndex = 0; wireIndex < face.wireCount(); ++wireIndex)
    {
        Ring2D ring;
        ring.depth = 0;

        if (!sampleWire(face.wire(wireIndex), surface, options, policy, ring.points))
        {
            return FaceMesh();
        }

        rings.push_back(ring);
    }

    if (policy.periodicU && !normalizeRingPeriods(rings, 0, surface.uPeriod(), options.geometricTolerance))
    {
        return FaceMesh();
    }

    if (policy.periodicV && !normalizeRingPeriods(rings, 1, surface.vPeriod(), options.geometricTolerance))
    {
        return FaceMesh();
    }

    calculateRingDepths(rings, options.geometricTolerance);

    std::vector<Triangle2D> triangles;

    for (std::size_t ringIndex = 0; ringIndex < rings.size(); ++ringIndex)
    {
        if ((rings[ringIndex].depth % 2) != 0)
        {
            continue;
        }

        std::vector<const Ring2D*> holes;

        for (std::size_t otherIndex = 0; otherIndex < rings.size(); ++otherIndex)
        {
            if (rings[otherIndex].depth != rings[ringIndex].depth + 1)
            {
                continue;
            }

            if (pointInRing(rings[otherIndex].points[0], rings[ringIndex].points, options.geometricTolerance))
            {
                holes.push_back(&rings[otherIndex]);
            }
        }

        if (!triangulateFilledRing(rings, rings[ringIndex], holes, options.geometricTolerance, triangles))
        {
            return FaceMesh();
        }
    }

    if (triangles.empty())
    {
        return FaceMesh();
    }

    for (std::size_t index = 0; index < triangles.size(); ++index)
    {
        const Triangle2D& triangle = triangles[index];
        const MyMath::Vector2 center = (triangle.first + triangle.second + triangle.third) / 3.0;

        if (classifyTrim(center, rings, options.geometricTolerance) != TrimClassification::Inside)
        {
            return FaceMesh();
        }
    }

    std::vector<MyMath::Vector2> parameterVertices;
    std::vector<IndexedTriangle> indexedTriangles;

    if (!buildIndexedTriangles(triangles, options.geometricTolerance, parameterVertices, indexedTriangles))
    {
        return FaceMesh();
    }

    if (!refineSurface(surface, options, parameterVertices, indexedTriangles))
    {
        return FaceMesh();
    }

    return buildFaceMesh(face, parameterVertices, indexedTriangles, policy, options.geometricTolerance);
}

}