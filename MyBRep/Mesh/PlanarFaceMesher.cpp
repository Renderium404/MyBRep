#include "PlanarFaceMesher.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "MyBRep/Geometry/Curve/CurveKind.h"
#include "MyBRep/Geometry/Surface/SurfaceKind.h"
#include "MyBRep/Tool/Query/FaceClassifier2D.h"

namespace
{

const int MaximumAllowedSubdivisionDepth = 20; // 限制单条P-Curve的最坏细分规模，避免错误配置造成指数级顶点数量。

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

double absoluteValue(double value)
{
    return value >= 0.0 ? value : -value;
}

bool isFiniteValue(double value)
{
    const double infinity = (std::numeric_limits<double>::infinity)();
    return value == value && value != infinity && value != -infinity;
}

double pointSegmentDistance(const MyMath::Vector2& point, const MyMath::Vector2& start, const MyMath::Vector2& end)
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
    if (pointSegmentDistance(point, first, second) > tolerance)
    {
        return false;
    }

    const double minX = (std::min)(first.x(), second.x()) - tolerance;
    const double maxX = (std::max)(first.x(), second.x()) + tolerance;
    const double minY = (std::min)(first.y(), second.y()) - tolerance;
    const double maxY = (std::max)(first.y(), second.y()) + tolerance;

    return point.x() >= minX && point.x() <= maxX && point.y() >= minY && point.y() <= maxY;
}

bool segmentsIntersect(const MyMath::Vector2& firstStart, const MyMath::Vector2& firstEnd,
                       const MyMath::Vector2& secondStart, const MyMath::Vector2& secondEnd, double tolerance)
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

bool pointStrictlyInTriangle(const MyMath::Vector2& point, const MyMath::Vector2& first, const MyMath::Vector2& second,
                             const MyMath::Vector2& third, double tolerance)
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

MyMath::Vector2 surfacePoint(const MyBRep::Topology_Edge& edge, const MyBRep::Geometry_Surface& surface, double parameter)
{
    return edge.surfaceParameterAt(surface, parameter);
}

bool intervalFlatEnough(const MyBRep::Topology_Edge& edge, const MyBRep::Geometry_Surface& surface,
                        double firstParameter, double lastParameter, const MyMath::Vector2& firstPoint,
                        const MyMath::Vector2& lastPoint, double chordTolerance)
{
    const double span = lastParameter - firstParameter;
    const double quarterParameter = firstParameter + span * 0.25; // 1/4、1/2、3/4三点共同检查，避免S形P-Curve中点落弦造成误判。
    const double middleParameter = firstParameter + span * 0.5;
    const double threeQuarterParameter = firstParameter + span * 0.75;

    return pointSegmentDistance(surfacePoint(edge, surface, quarterParameter), firstPoint, lastPoint) <= chordTolerance &&
           pointSegmentDistance(surfacePoint(edge, surface, middleParameter), firstPoint, lastPoint) <= chordTolerance &&
           pointSegmentDistance(surfacePoint(edge, surface, threeQuarterParameter), firstPoint, lastPoint) <= chordTolerance;
}

void appendAdaptiveInterval(const MyBRep::Topology_Edge& edge, const MyBRep::Geometry_Surface& surface,
                            double firstParameter, double lastParameter, const MyMath::Vector2& firstPoint,
                            const MyMath::Vector2& lastPoint, int depth, const MyBRep::PlanarFaceMeshOptions& options,
                            std::vector<MyMath::Vector2>& points)
{
    const bool minimumDepthReached = depth >= options.minimumSubdivisionDepth;
    const bool maximumDepthReached = depth >= options.maximumSubdivisionDepth;

    if (maximumDepthReached ||
        (minimumDepthReached && intervalFlatEnough(edge, surface, firstParameter, lastParameter, firstPoint, lastPoint, options.chordTolerance)))
    {
        points.push_back(lastPoint);
        return;
    }

    const double middleParameter = (firstParameter + lastParameter) * 0.5;
    const MyMath::Vector2 middlePoint = surfacePoint(edge, surface, middleParameter);

    appendAdaptiveInterval(edge, surface, firstParameter, middleParameter, firstPoint, middlePoint, depth + 1, options, points);
    appendAdaptiveInterval(edge, surface, middleParameter, lastParameter, middlePoint, lastPoint, depth + 1, options, points);
}

bool sampleWire(const MyBRep::Topology_Wire& wire, const MyBRep::Geometry_Surface& surface,
                const MyBRep::PlanarFaceMeshOptions& options, std::vector<MyMath::Vector2>& points)
{
    points.clear();

    if (!wire.isValid() || !wire.isClosed())
    {
        return false;
    }

    for (std::size_t edgeIndex = 0; edgeIndex < wire.edgeCount(); ++edgeIndex)
    {
        const MyBRep::Topology_Edge edge = wire.edge(edgeIndex);

        if (!edge.isValid() || !edge.hasCurveOnSurface(surface))
        {
            return false;
        }

        const MyMath::Vector2 firstPoint = surfacePoint(edge, surface, 0.0);
        const MyMath::Vector2 lastPoint = surfacePoint(edge, surface, 1.0);

        if (points.empty())
        {
            points.push_back(firstPoint);
        }
        else if (!pointsEqual(points.back(), firstPoint, options.geometricTolerance))
        {
            return false;
        }

        if (edge.curveOnSurface(surface).kind() == MyBRep::CurveKind::Line)
        {
            points.push_back(lastPoint);
        }
        else
        {
            appendAdaptiveInterval(edge, surface, 0.0, 1.0, firstPoint, lastPoint, 0, options, points);
        }
    }

    removeConsecutiveDuplicates(points, options.geometricTolerance);
    removeSimpleCollinearPoints(points, options.geometricTolerance);

    return points.size() >= 3 && absoluteValue(signedArea(points)) > options.geometricTolerance * options.geometricTolerance;
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

bool bridgeCrossesPolygon(const MyMath::Vector2& first, const MyMath::Vector2& second,
                          const std::vector<MyMath::Vector2>& polygon, double tolerance)
{
    for (std::size_t index = 0; index < polygon.size(); ++index)
    {
        const MyMath::Vector2& edgeStart = polygon[index];
        const MyMath::Vector2& edgeEnd = polygon[(index + 1) % polygon.size()];

        if (pointsEqual(edgeStart, first, tolerance) || pointsEqual(edgeEnd, first, tolerance) ||
            pointsEqual(edgeStart, second, tolerance) || pointsEqual(edgeEnd, second, tolerance))
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

bool bridgeCrossesRing(const MyMath::Vector2& first, const MyMath::Vector2& second,
                       const std::vector<MyMath::Vector2>& ring, double tolerance)
{
    for (std::size_t index = 0; index < ring.size(); ++index)
    {
        const MyMath::Vector2& edgeStart = ring[index];
        const MyMath::Vector2& edgeEnd = ring[(index + 1) % ring.size()];

        if (pointsEqual(edgeStart, first, tolerance) || pointsEqual(edgeEnd, first, tolerance) ||
            pointsEqual(edgeStart, second, tolerance) || pointsEqual(edgeEnd, second, tolerance))
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

bool findBridge(const MyBRep::Topology_Face& face, const std::vector<MyMath::Vector2>& polygon,
                const std::vector<std::vector<MyMath::Vector2> >& remainingHoles, std::size_t currentHoleIndex,
                std::size_t holeVertexIndex, double tolerance, std::size_t& polygonVertexIndex)
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

        if (MyBRep::classifyFaceUV(face, middle, tolerance) != MyBRep::FaceUVClassification::Inside)
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

void stitchHole(std::vector<MyMath::Vector2>& polygon, std::size_t polygonVertexIndex,
                const std::vector<MyMath::Vector2>& hole, std::size_t holeVertexIndex)
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

bool triangulateFilledRing(const MyBRep::Topology_Face& face, const Ring2D& outerRing,
                           const std::vector<const Ring2D*>& holeRings, double tolerance, std::vector<Triangle2D>& triangles)
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

    std::sort(holes.begin(), holes.end(), [](const std::vector<MyMath::Vector2>& first, const std::vector<MyMath::Vector2>& second)
    {
        return first[rightmostVertex(first)].x() > second[rightmostVertex(second)].x();
    });

    for (std::size_t holeIndex = 0; holeIndex < holes.size(); ++holeIndex)
    {
        const std::size_t holeVertexIndex = rightmostVertex(holes[holeIndex]);
        std::size_t polygonVertexIndex = 0;

        if (!findBridge(face, polygon, holes, holeIndex, holeVertexIndex, tolerance, polygonVertexIndex))
        {
            return false;
        }

        stitchHole(polygon, polygonVertexIndex, holes[holeIndex], holeVertexIndex);
    }

    return earClip(polygon, tolerance, triangles);
}

unsigned int findOrAddVertex(MyBRep::FaceMesh& mesh, const MyBRep::Topology_Face& face, const MyMath::Vector2& parameter, double tolerance)
{
    const std::vector<MyBRep::FaceMeshVertex>& vertices = mesh.vertices();

    for (std::size_t index = 0; index < vertices.size(); ++index)
    {
        if (pointsEqual(vertices[index].parameter, parameter, tolerance))
        {
            return static_cast<unsigned int>(index);
        }
    }

    const MyMath::Vector3 position = face.geometry().pointAt(parameter.x(), parameter.y());
    const MyMath::Vector3 normal = face.normalAt(parameter.x(), parameter.y());
    return mesh.addVertex(MyBRep::FaceMeshVertex(parameter, position, normal));
}

}

namespace MyBRep
{

PlanarFaceMeshOptions::PlanarFaceMeshOptions()
    : chordTolerance(1.0e-3)                         // 默认1e-3模型单位弦误差，平面UV参数与世界距离同尺度。
    , geometricTolerance(MyMath::Vector2::DefaultEpsilon) // 默认沿用MyMath二维基础几何比较误差，不额外扩大调用者精度假设。
    , minimumSubdivisionDepth(2)                    // 非直线P-Curve至少四等分，降低局部曲率采样遗漏。
    , maximumSubdivisionDepth(12)                   // 最多4096个参数区间，限制异常曲率或过小弦误差导致的细分规模。
{
}

bool PlanarFaceMeshOptions::isValid() const
{
    return isFiniteValue(chordTolerance) && chordTolerance > 0.0 &&
           isFiniteValue(geometricTolerance) && geometricTolerance > 0.0 &&
           minimumSubdivisionDepth >= 0 &&
           maximumSubdivisionDepth >= minimumSubdivisionDepth &&
           maximumSubdivisionDepth <= MaximumAllowedSubdivisionDepth;
}

bool PlanarFaceMesher::canMesh(const Topology_Face& face)
{
    if (!face.isValid() || face.geometry().kind() != SurfaceKind::Plane || face.wireCount() == 0)
    {
        return false;
    }

    const Geometry_Surface& surface = face.geometry();

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

FaceMesh PlanarFaceMesher::mesh(const Topology_Face& face, const PlanarFaceMeshOptions& options)
{
    FaceMesh result;

    if (!canMesh(face) || !options.isValid())
    {
        return result;
    }

    std::vector<Ring2D> rings;
    rings.reserve(face.wireCount());

    for (std::size_t wireIndex = 0; wireIndex < face.wireCount(); ++wireIndex)
    {
        Ring2D ring;
        ring.depth = 0;

        if (!sampleWire(face.wire(wireIndex), face.geometry(), options, ring.points))
        {
            return FaceMesh();
        }

        rings.push_back(ring);
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

            if (pointInRing(rings[otherIndex].points[0], rings[ringIndex].points, options.geometricTolerance))
            {
                holes.push_back(&rings[otherIndex]);
            }
        }

        if (!triangulateFilledRing(face, rings[ringIndex], holes, options.geometricTolerance, triangles))
        {
            return FaceMesh();
        }
    }

    if (triangles.empty())
    {
        return result;
    }

    for (std::size_t triangleIndex = 0; triangleIndex < triangles.size(); ++triangleIndex)
    {
        const Triangle2D& triangle = triangles[triangleIndex];
        const MyMath::Vector2 center = (triangle.first + triangle.second + triangle.third) / 3.0;

        if (classifyFaceUV(face, center, options.geometricTolerance) != FaceUVClassification::Inside)
        {
            return FaceMesh();
        }

        const unsigned int first = findOrAddVertex(result, face, triangle.first, options.geometricTolerance);
        const unsigned int second = findOrAddVertex(result, face, triangle.second, options.geometricTolerance);
        const unsigned int third = findOrAddVertex(result, face, triangle.third, options.geometricTolerance);

        if (face.isForward())
        {
            result.addTriangle(first, second, third);
        }
        else
        {
            result.addTriangle(first, third, second);
        }
    }

    return result.isValid() ? result : FaceMesh();
}

}
