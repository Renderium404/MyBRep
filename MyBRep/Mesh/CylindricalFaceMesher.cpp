#include "CylindricalFaceMesher.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <vector>

#include "MyBRep/Geometry/Surface/Geometry_CylindricalSurface.h"
#include "MyBRep/Geometry/Surface/SurfaceKind.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 圆柱周期与最大单个曲面三角形角跨度统一使用的圆周率。
const double HalfPi = Pi * 0.5;                       // 单个Surface三角形最大允许跨越90°，避免极宽三角形跨越圆柱背面。
const int MaximumAllowedBoundarySubdivisionDepth = 20; // 限制单条trimming Edge最坏二分规模，防止错误配置造成指数级采样。
const int MaximumAllowedSurfaceSubdivisionRounds = 20; // 限制共享边一致细分传播轮数，防止极端容差导致无限增长。

struct Ring2D
{
    std::vector<MyMath::Vector2> points;
    int depth;
};

struct Triangle2D
{
    MyMath::Vector2 first;
    MyMath::Vector2 second;
    MyMath::Vector2 third;
};

struct IndexedTriangle
{
    IndexedTriangle() : first(0), second(0), third(0)
    {
    }

    IndexedTriangle(unsigned int firstValue, unsigned int secondValue, unsigned int thirdValue) : first(firstValue), second(secondValue), third(thirdValue)
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
        return (point - start).length();
    }

    double parameter = MyMath::Vector3::dot(point - start, segment) / lengthSquared;
    parameter = (std::max)(0.0, (std::min)(1.0, parameter));
    return (point - (start + segment * parameter)).length();
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

bool pointsEqual(const MyMath::Vector2& first, const MyMath::Vector2& second, double tolerance)
{
    return first.distanceSquaredTo(second) <= tolerance * tolerance;
}

double orientation(const MyMath::Vector2& first, const MyMath::Vector2& second, const MyMath::Vector2& third)
{
    return MyMath::Vector2::cross(second - first, third - first);
}

bool pointOnSegment(const MyMath::Vector2& point, const MyMath::Vector2& first, const MyMath::Vector2& second, double tolerance)
{
    if (pointSegmentDistance2D(point, first, second) > tolerance)
    {
        return false;
    }

    const double minX = (std::min)(first.x(), second.x()) - tolerance;
    const double maxX = (std::max)(first.x(), second.x()) + tolerance;
    const double minY = (std::min)(first.y(), second.y()) - tolerance;
    const double maxY = (std::max)(first.y(), second.y()) + tolerance;

    return point.x() >= minX && point.x() <= maxX && point.y() >= minY && point.y() <= maxY;
}

bool segmentsIntersect(const MyMath::Vector2& firstStart, const MyMath::Vector2& firstEnd, const MyMath::Vector2& secondStart, const MyMath::Vector2& secondEnd, double tolerance)
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

bool pointStrictlyInTriangle(const MyMath::Vector2& point, const MyMath::Vector2& first, const MyMath::Vector2& second, const MyMath::Vector2& third, double tolerance)
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
    filtered.push_back(points[0]);

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
    const double quarterParameter = firstParameter + span * 0.25; // 三个内部采样点共同检查，避免P-Curve局部弯曲被单一中点遗漏。
    const double middleParameter = firstParameter + span * 0.5;
    const double threeQuarterParameter = firstParameter + span * 0.75;

    const MyMath::Vector3 firstPosition = surfacePosition(surface, firstUV);
    const MyMath::Vector3 lastPosition = surfacePosition(surface, lastUV);

    const MyMath::Vector2 quarterUV = surfaceParameter(edge, surface, quarterParameter);
    const MyMath::Vector2 middleUV = surfaceParameter(edge, surface, middleParameter);
    const MyMath::Vector2 threeQuarterUV = surfaceParameter(edge, surface, threeQuarterParameter);

    return pointSegmentDistance3D(surfacePosition(surface, quarterUV), firstPosition, lastPosition) <= chordTolerance &&
           pointSegmentDistance3D(surfacePosition(surface, middleUV), firstPosition, lastPosition) <= chordTolerance &&
           pointSegmentDistance3D(surfacePosition(surface, threeQuarterUV), firstPosition, lastPosition) <= chordTolerance;
}

void appendAdaptiveBoundaryInterval(const MyBRep::Topology_Edge& edge,
                                    const MyBRep::Geometry_Surface& surface,
                                    double firstParameter,
                                    double lastParameter,
                                    const MyMath::Vector2& firstUV,
                                    const MyMath::Vector2& lastUV,
                                    int depth,
                                    const MyBRep::CylindricalFaceMeshOptions& options,
                                    std::vector<MyMath::Vector2>& points)
{
    const bool minimumDepthReached = depth >= options.minimumBoundarySubdivisionDepth;
    const bool maximumDepthReached = depth >= options.maximumBoundarySubdivisionDepth;

    if (maximumDepthReached || (minimumDepthReached && boundaryIntervalFlatEnough(edge, surface, firstParameter, lastParameter, firstUV, lastUV, options.boundaryChordTolerance)))
    {
        points.push_back(lastUV);
        return;
    }

    const double middleParameter = (firstParameter + lastParameter) * 0.5;
    const MyMath::Vector2 middleUV = surfaceParameter(edge, surface, middleParameter);

    appendAdaptiveBoundaryInterval(edge, surface, firstParameter, middleParameter, firstUV, middleUV, depth + 1, options, points);

    appendAdaptiveBoundaryInterval(edge, surface, middleParameter, lastParameter, middleUV, lastUV, depth + 1, options, points);
}

bool sampleEdge(const MyBRep::Topology_Edge& edge, const MyBRep::Geometry_Surface& surface, const MyBRep::CylindricalFaceMeshOptions& options, std::vector<MyMath::Vector2>& points)
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

    removeConsecutiveDuplicates(points, options.geometricTolerance);
    return points.size() >= 2;
}

void shiftPointsU(std::vector<MyMath::Vector2>& points, double shift)
{
    if (shift == 0.0)
    {
        return;
    }

    for (std::size_t index = 0; index < points.size(); ++index)
    {
        points[index].setX(points[index].x() + shift);
    }
}

bool alignEdgeSampleToPrevious(std::vector<MyMath::Vector2>& edgePoints, const MyMath::Vector2& previousPoint, double period, double tolerance)
{
    if (edgePoints.empty())
    {
        return false;
    }

    const double shiftCount = nearestInteger((previousPoint.x() - edgePoints.front().x()) / period);
    const double shift = shiftCount * period;
    shiftPointsU(edgePoints, shift);

    return pointsEqual(previousPoint, edgePoints.front(), tolerance);
}

bool sampleWire(const MyBRep::Topology_Wire& wire, const MyBRep::Geometry_Surface& surface, const MyBRep::CylindricalFaceMeshOptions& options, std::vector<MyMath::Vector2>& points)
{
    points.clear();

    if (!wire.isValid() || !wire.isClosed())
    {
        return false;
    }

    const double period = surface.uPeriod();

    for (std::size_t edgeIndex = 0; edgeIndex < wire.edgeCount(); ++edgeIndex)
    {
        std::vector<MyMath::Vector2> edgePoints;

        if (!sampleEdge(wire.edge(edgeIndex), surface, options, edgePoints))
        {
            return false;
        }

        if (!points.empty())
        {
            if (!alignEdgeSampleToPrevious(edgePoints, points.back(), period, options.geometricTolerance))
            {
                return false;
            }

            edgePoints.erase(edgePoints.begin());
        }

        points.insert(points.end(), edgePoints.begin(), edgePoints.end());
    }

    if (points.size() < 4 || !pointsEqual(points.front(), points.back(), options.geometricTolerance))
    {
        return false;
    }

    removeConsecutiveDuplicates(points, options.geometricTolerance);
    removeSimpleCollinearPoints(points, options.geometricTolerance);

    return points.size() >= 3 && absoluteValue(signedArea(points)) > options.geometricTolerance * options.geometricTolerance;
}

double ringCenterU(const Ring2D& ring)
{
    double sum = 0.0;

    for (std::size_t index = 0; index < ring.points.size(); ++index)
    {
        sum += ring.points[index].x();
    }

    return sum / static_cast<double>(ring.points.size());
}

void shiftRingU(Ring2D& ring, double shift)
{
    shiftPointsU(ring.points, shift);
}

double ringUSpan(const Ring2D& ring)
{
    double minimum = ring.points[0].x();
    double maximum = ring.points[0].x();

    for (std::size_t index = 1; index < ring.points.size(); ++index)
    {
        minimum = (std::min)(minimum, ring.points[index].x());
        maximum = (std::max)(maximum, ring.points[index].x());
    }

    return maximum - minimum;
}

bool normalizeRingPeriods(std::vector<Ring2D>& rings, double period, double tolerance)
{
    if (rings.empty())
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

    const double referenceCenter = ringCenterU(rings[referenceIndex]);

    for (std::size_t index = 0; index < rings.size(); ++index)
    {
        if (ringUSpan(rings[index]) > period + tolerance)
        {
            return false;
        }

        if (index == referenceIndex)
        {
            continue;
        }

        const double shiftCount = nearestInteger((referenceCenter - ringCenterU(rings[index])) / period);

        shiftRingU(rings[index], shiftCount * period);
    }

    double globalMinimum = rings[0].points[0].x();
    double globalMaximum = rings[0].points[0].x();

    for (std::size_t ringIndex = 0; ringIndex < rings.size(); ++ringIndex)
    {
        for (std::size_t pointIndex = 0; pointIndex < rings[ringIndex].points.size(); ++pointIndex)
        {
            globalMinimum = (std::min)(globalMinimum, rings[ringIndex].points[pointIndex].x());
            globalMaximum = (std::max)(globalMaximum, rings[ringIndex].points[pointIndex].x());
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
        if (ring[index].x() > ring[result].x() || (ring[index].x() == ring[result].x() && ring[index].y() < ring[result].y()))
        {
            result = index;
        }
    }

    return result;
}

bool bridgeCrossesPolygon(const MyMath::Vector2& first, const MyMath::Vector2& second, const std::vector<MyMath::Vector2>& polygon, double tolerance)
{
    for (std::size_t index = 0; index < polygon.size(); ++index)
    {
        const MyMath::Vector2& edgeStart = polygon[index];
        const MyMath::Vector2& edgeEnd = polygon[(index + 1) % polygon.size()];

        if (pointsEqual(edgeStart, first, tolerance) || pointsEqual(edgeEnd, first, tolerance) || pointsEqual(edgeStart, second, tolerance) || pointsEqual(edgeEnd, second, tolerance))
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

bool bridgeCrossesRing(const MyMath::Vector2& first, const MyMath::Vector2& second, const std::vector<MyMath::Vector2>& ring, double tolerance)
{
    for (std::size_t index = 0; index < ring.size(); ++index)
    {
        const MyMath::Vector2& edgeStart = ring[index];
        const MyMath::Vector2& edgeEnd = ring[(index + 1) % ring.size()];

        if (pointsEqual(edgeStart, first, tolerance) || pointsEqual(edgeEnd, first, tolerance) || pointsEqual(edgeStart, second, tolerance) || pointsEqual(edgeEnd, second, tolerance))
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

void stitchHole(std::vector<MyMath::Vector2>& polygon, std::size_t polygonVertexIndex, const std::vector<MyMath::Vector2>& hole, std::size_t holeVertexIndex)
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

        if (pointsEqual(testPoint, previous, tolerance) || pointsEqual(testPoint, current, tolerance) || pointsEqual(testPoint, next, tolerance))
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

        if (pointsEqual(polygon[index], polygon[previousIndex], tolerance) || pointsEqual(polygon[index], polygon[nextIndex], tolerance))
        {
            polygon.erase(polygon.begin() + index);
            return true;
        }

        if (absoluteValue(orientation(polygon[previousIndex],
                                      polygon[index],
                                      polygon[nextIndex])) <= tolerance &&
            pointOnSegment(polygon[index],
                           polygon[previousIndex],
                           polygon[nextIndex],
                           tolerance))
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
    const std::size_t guardLimit = polygon.size() * polygon.size() * 4; // 二次规模保护用于识别无法继续耳切的退化弱简单多边形。

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

            Triangle2D triangle;
            triangle.first = polygon[previousIndex];
            triangle.second = polygon[index];
            triangle.third = polygon[nextIndex];
            triangles.push_back(triangle);

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

    Triangle2D triangle;
    triangle.first = polygon[0];
    triangle.second = polygon[1];
    triangle.third = polygon[2];
    triangles.push_back(triangle);

    return true;
}

bool triangulateFilledRing(const std::vector<Ring2D>& allRings, const Ring2D& outerRing, const std::vector<const Ring2D*>& holeRings, double tolerance, std::vector<Triangle2D>& triangles)
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

unsigned int findOrAddParameterVertex( std::vector<MyMath::Vector2>& vertices, const MyMath::Vector2& parameter, double tolerance)
{
    for (std::size_t index = 0; index < vertices.size(); ++index)
    {
        if (pointsEqual(vertices[index], parameter, tolerance))
        {
            return static_cast<unsigned int>(index);
        }
    }

    vertices.push_back(parameter);
    return static_cast<unsigned int>(vertices.size() - 1);
}

bool buildIndexedTriangles(const std::vector<Triangle2D>& triangles, double tolerance, std::vector<MyMath::Vector2>& vertices, std::vector<IndexedTriangle>& indexedTriangles)
{
    vertices.clear();
    indexedTriangles.clear();

    for (std::size_t index = 0; index < triangles.size(); ++index)
    {
        const Triangle2D& triangle = triangles[index];

        const unsigned int first = findOrAddParameterVertex(vertices, triangle.first, tolerance);

        const unsigned int second = findOrAddParameterVertex(vertices, triangle.second, tolerance);

        const unsigned int third = findOrAddParameterVertex(vertices, triangle.third, tolerance);

        if (first == second || second == third || third == first)
        {
            return false;
        }

        indexedTriangles.push_back( IndexedTriangle(first, second, third));
    }

    return !vertices.empty() && !indexedTriangles.empty();
}

double maximumAngularStep(double radius, double chordTolerance)
{
    double cosineValue = 1.0 - chordTolerance / radius;
    cosineValue = (std::max)(-1.0, (std::min)(1.0, cosineValue));

    double angularStep = 2.0 * std::acos(cosineValue);

    // 即使调用方给出很大的弦误差，也不让单个三角形跨越超过90°，
    // 防止一条参数边直接横跨圆柱背面并形成视觉上不稳定的宽三角形。
    angularStep = (std::min)(angularStep, HalfPi);
    return angularStep;
}

bool edgeNeedsSurfaceSplit(const std::vector<MyMath::Vector2>& vertices, const EdgeKey& edge, double angularStep)
{
    return absoluteValue(vertices[edge.first].x() - vertices[edge.second].x()) > angularStep;
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

void addMissingThirdEdgeForTwoSplitTriangle( const IndexedTriangle& triangle, std::set<EdgeKey>& splitEdges)
{
    const EdgeKey firstSecond(triangle.first, triangle.second);
    const EdgeKey secondThird(triangle.second, triangle.third);
    const EdgeKey thirdFirst(triangle.third, triangle.first);

    const bool splitFirstSecond = splitEdges.find(firstSecond) != splitEdges.end();
    const bool splitSecondThird = splitEdges.find(secondThird) != splitEdges.end();
    const bool splitThirdFirst = splitEdges.find(thirdFirst) != splitEdges.end();

    const int count = (splitFirstSecond ? 1 : 0) + (splitSecondThird ? 1 : 0) + (splitThirdFirst ? 1 : 0);

    if (count != 2)
    {
        return;
    }

    if (!splitFirstSecond)
    {
        splitEdges.insert(firstSecond);
    }
    else if (!splitSecondThird)
    {
        splitEdges.insert(secondThird);
    }
    else
    {
        splitEdges.insert(thirdFirst);
    }
}

unsigned int midpointVertex( const EdgeKey& edge, std::vector<MyMath::Vector2>& vertices, std::map<EdgeKey, unsigned int>& midpointIndices)
{
    std::map<EdgeKey, unsigned int>::const_iterator iterator = midpointIndices.find(edge);

    if (iterator != midpointIndices.end())
    {
        return iterator->second;
    }

    const MyMath::Vector2 midpoint = (vertices[edge.first] + vertices[edge.second]) * 0.5;

    vertices.push_back(midpoint);

    const unsigned int index = static_cast<unsigned int>(vertices.size() - 1);

    midpointIndices[edge] = index;
    return index;
}

bool refineSurfaceOnce(std::vector<MyMath::Vector2>& vertices, std::vector<IndexedTriangle>& triangles, double angularStep, bool& changed)
{
    changed = false;

    std::set<EdgeKey> splitEdges;

    for (std::size_t index = 0; index < triangles.size(); ++index)
    {
        const IndexedTriangle& triangle = triangles[index];

        const EdgeKey firstSecond(triangle.first, triangle.second);
        const EdgeKey secondThird(triangle.second, triangle.third);
        const EdgeKey thirdFirst(triangle.third, triangle.first);

        if (edgeNeedsSurfaceSplit(vertices, firstSecond, angularStep))
        {
            splitEdges.insert(firstSecond);
        }

        if (edgeNeedsSurfaceSplit(vertices, secondThird, angularStep))
        {
            splitEdges.insert(secondThird);
        }

        if (edgeNeedsSurfaceSplit(vertices, thirdFirst, angularStep))
        {
            splitEdges.insert(thirdFirst);
        }
    }

    if (splitEdges.empty())
    {
        return true;
    }

    // 一个Triangle若恰有两条共享边需要切分，则补切第三边。
    // 传播到稳定后，每个Triangle只会出现0、1或3条切分边，
    // 这样相邻Triangle共享同一Edge midpoint，不产生T-junction。
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

            addMissingThirdEdgeForTwoSplitTriangle( triangles[index], splitEdges);

            if (splitEdges.size() != before)
            {
                propagationChanged = true;
            }
        }
    }

    std::map<EdgeKey, unsigned int> midpointIndices;
    std::vector<IndexedTriangle> refined;
    refined.reserve(triangles.size() * 4);

    for (std::size_t index = 0; index < triangles.size(); ++index)
    {
        const IndexedTriangle& triangle = triangles[index];

        const EdgeKey firstSecond(triangle.first, triangle.second);
        const EdgeKey secondThird(triangle.second, triangle.third);
        const EdgeKey thirdFirst(triangle.third, triangle.first);

        const bool splitFirstSecond = splitEdges.find(firstSecond) != splitEdges.end();
        const bool splitSecondThird = splitEdges.find(secondThird) != splitEdges.end();
        const bool splitThirdFirst = splitEdges.find(thirdFirst) != splitEdges.end();

        const int count = (splitFirstSecond ? 1 : 0) + (splitSecondThird ? 1 : 0) + (splitThirdFirst ? 1 : 0);

        if (count == 0)
        {
            refined.push_back(triangle);
            continue;
        }

        if (count == 1)
        {
            if (splitFirstSecond)
            {
                const unsigned int midpoint = midpointVertex(firstSecond, vertices, midpointIndices);

                refined.push_back( IndexedTriangle(triangle.first, midpoint, triangle.third));

                refined.push_back( IndexedTriangle(midpoint, triangle.second, triangle.third));
            }
            else if (splitSecondThird)
            {
                const unsigned int midpoint = midpointVertex(secondThird, vertices, midpointIndices);

                refined.push_back( IndexedTriangle(triangle.first, triangle.second, midpoint));

                refined.push_back( IndexedTriangle(triangle.first, midpoint, triangle.third));
            }
            else
            {
                const unsigned int midpoint = midpointVertex(thirdFirst, vertices, midpointIndices);

                refined.push_back( IndexedTriangle(triangle.first, triangle.second, midpoint));

                refined.push_back( IndexedTriangle(triangle.second, triangle.third, midpoint));
            }

            continue;
        }

        if (count != 3)
        {
            return false;
        }

        const unsigned int firstSecondMidpoint = midpointVertex(firstSecond, vertices, midpointIndices);

        const unsigned int secondThirdMidpoint = midpointVertex(secondThird, vertices, midpointIndices);

        const unsigned int thirdFirstMidpoint = midpointVertex(thirdFirst, vertices, midpointIndices);

        refined.push_back( IndexedTriangle(triangle.first, firstSecondMidpoint, thirdFirstMidpoint));

        refined.push_back( IndexedTriangle(firstSecondMidpoint, triangle.second, secondThirdMidpoint));

        refined.push_back( IndexedTriangle(thirdFirstMidpoint, secondThirdMidpoint, triangle.third));

        refined.push_back( IndexedTriangle(firstSecondMidpoint, secondThirdMidpoint, thirdFirstMidpoint));
    }

    triangles.swap(refined);
    changed = true;
    return true;
}

bool surfaceNeedsMoreRefinement( const std::vector<MyMath::Vector2>& vertices, const std::vector<IndexedTriangle>& triangles, double angularStep)
{
    for (std::size_t index = 0; index < triangles.size(); ++index)
    {
        const IndexedTriangle& triangle = triangles[index];

        if (edgeNeedsSurfaceSplit(vertices,EdgeKey(triangle.first, triangle.second),angularStep) ||
            edgeNeedsSurfaceSplit(vertices, EdgeKey(triangle.second, triangle.third), angularStep) ||
            edgeNeedsSurfaceSplit(vertices,EdgeKey(triangle.third, triangle.first),angularStep))
        {
            return true;
        }
    }

    return false;
}

bool refineCylinderSurface( double radius, const MyBRep::CylindricalFaceMeshOptions& options, std::vector<MyMath::Vector2>& vertices, std::vector<IndexedTriangle>& triangles)
{
    const double angularStep = maximumAngularStep(radius, options.surfaceChordTolerance);

    if (!isFiniteValue(angularStep) || angularStep <= 0.0)
    {
        return false;
    }

    for (int round = 0; round < options.maximumSurfaceSubdivisionRounds; ++round)
    {
        bool changed = false;

        if (!refineSurfaceOnce(vertices, triangles, angularStep, changed))
        {
            return false;
        }

        if (!changed)
        {
            return true;
        }
    }

    return !surfaceNeedsMoreRefinement(vertices, triangles, angularStep);
}

bool triangleIsNonDegenerate3D( const MyBRep::Geometry_Surface& surface, const MyMath::Vector2& firstParameter, const MyMath::Vector2& secondParameter, const MyMath::Vector2& thirdParameter)
{
    const MyMath::Vector3 first = surfacePosition(surface, firstParameter);
    const MyMath::Vector3 second = surfacePosition(surface, secondParameter);
    const MyMath::Vector3 third = surfacePosition(surface, thirdParameter);

    const MyMath::Vector3 cross = MyMath::Vector3::cross(second - first, third - first);

    return cross.isVector(0.0);
}

MyBRep::FaceMesh buildFaceMesh( const MyBRep::Topology_Face& face, const std::vector<MyMath::Vector2>& parameterVertices, const std::vector<IndexedTriangle>& triangles)
{
    MyBRep::FaceMesh result;

    for (std::size_t index = 0; index < parameterVertices.size(); ++index)
    {
        const MyMath::Vector2& parameter = parameterVertices[index];

        const MyMath::Vector3 position = face.geometry().pointAt(parameter.x(), parameter.y());

        const MyMath::Vector3 normal = face.normalAt(parameter.x(), parameter.y());

        result.addVertex( MyBRep::FaceMeshVertex(parameter, position, normal));
    }

    for (std::size_t index = 0; index < triangles.size(); ++index)
    {
        const IndexedTriangle& triangle = triangles[index];

        if (!triangleIsNonDegenerate3D( face.geometry(), parameterVertices[triangle.first], parameterVertices[triangle.second], parameterVertices[triangle.third]))
        {
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

CylindricalFaceMeshOptions::CylindricalFaceMeshOptions()
    : boundaryChordTolerance(1.0e-3)                     // 默认1e-3模型单位边界弦误差，直接约束Surface上的三维边界近似。
    , surfaceChordTolerance(1.0e-3)                      // 默认1e-3模型单位圆柱弦高误差，控制U方向曲面细分。
    , geometricTolerance(MyMath::Vector2::DefaultEpsilon)// UV连接与周期展开沿用MyMath二维基础比较误差。
    , minimumBoundarySubdivisionDepth(2)                // trimming Edge至少四等分后再允许停止，降低复杂P-Curve局部变化遗漏。
    , maximumBoundarySubdivisionDepth(12)               // 单Edge最多4096个二分区间，限制异常曲率或过小容差造成的采样规模。
    , maximumSurfaceSubdivisionRounds(12)               // 最多12轮共享边一致细分，兼顾细圆柱和小弦高误差。
{
}

bool CylindricalFaceMeshOptions::isValid() const
{
    return isFiniteValue(boundaryChordTolerance) &&
           boundaryChordTolerance > 0.0 &&
           isFiniteValue(surfaceChordTolerance) &&
           surfaceChordTolerance > 0.0 &&
           isFiniteValue(geometricTolerance) &&
           geometricTolerance > 0.0 &&
           minimumBoundarySubdivisionDepth >= 0 &&
           maximumBoundarySubdivisionDepth >=
               minimumBoundarySubdivisionDepth &&
           maximumBoundarySubdivisionDepth <=
               MaximumAllowedBoundarySubdivisionDepth &&
           maximumSurfaceSubdivisionRounds > 0 &&
           maximumSurfaceSubdivisionRounds <= MaximumAllowedSurfaceSubdivisionRounds;
}

bool CylindricalFaceMesher::canMesh( const Topology_Face& face)
{
    if (!face.isValid() || face.geometry().kind() != SurfaceKind::Cylindrical || face.wireCount() == 0)
    {
        return false;
    }

    const Geometry_Surface& surface = face.geometry();

    if (!surface.isUPeriodic() || surface.uPeriod() <= 0.0 || surface.isVPeriodic())
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
            if (!wire.edge(edgeIndex) .hasCurveOnSurface(surface))
            {
                return false;
            }
        }
    }

    return true;
}

FaceMesh CylindricalFaceMesher::mesh( const Topology_Face& face, const CylindricalFaceMeshOptions& options)
{
    FaceMesh result;

    if (!canMesh(face) || !options.isValid())
    {
        return result;
    }

    const Geometry_CylindricalSurface& cylinder = static_cast<const Geometry_CylindricalSurface&>( face.geometry());

    const double period = cylinder.uPeriod();

    std::vector<Ring2D> rings;
    rings.reserve(face.wireCount());

    for (std::size_t wireIndex = 0; wireIndex < face.wireCount(); ++wireIndex)
    {
        Ring2D ring;
        ring.depth = 0;

        if (!sampleWire(face.wire(wireIndex), cylinder, options, ring.points))
        {
            return FaceMesh();
        }

        rings.push_back(ring);
    }

    if (!normalizeRingPeriods(rings, period, options.geometricTolerance))
    {
        return FaceMesh();
    }

    calculateRingDepths(rings, options.geometricTolerance);

    std::vector<Triangle2D> triangles;

    for (std::size_t ringIndex = 0; ringIndex < rings.size(); ++ringIndex)
    {
        if (rings[ringIndex].depth % 2 != 0)
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

            if (pointInRing( rings[otherIndex].points[0], rings[ringIndex].points, options.geometricTolerance))
            {
                holes.push_back(&rings[otherIndex]);
            }
        }

        if (!triangulateFilledRing( rings, rings[ringIndex], holes, options.geometricTolerance, triangles))
        {
            return FaceMesh();
        }
    }

    if (triangles.empty())
    {
        return result;
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

    if (!refineCylinderSurface( cylinder.radius(), options, parameterVertices, indexedTriangles))
    {
        return FaceMesh();
    }

    return buildFaceMesh(face, parameterVertices, indexedTriangles);
}

}