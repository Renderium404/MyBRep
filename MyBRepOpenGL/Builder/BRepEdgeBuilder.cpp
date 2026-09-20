#include "BRepEdgeBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "MyBRep/Foundation/Diagnostic.h"
#include "MyBRep/Geometry/Curve/CurveKind.h"
#include "MyOpenGL/Resource/BufferGeometry.h"

namespace
{

const int MaximumAllowedSubdivisionDepth = 20;

bool isFiniteValue(double value)
{
    const double infinity = (std::numeric_limits<double>::infinity)();
    return value == value && value != infinity && value != -infinity;
}

bool isFinitePositive(double value)
{
    return isFiniteValue(value) && value > 0.0;
}

double pointSegmentDistance(const MyMath::Vector3& point, const MyMath::Vector3& start, const MyMath::Vector3& end)
{
    const MyMath::Vector3 segment = end - start;
    const double lengthSquared = MyMath::Vector3::dot(segment, segment);

    if (lengthSquared <= 0.0) return (point - start).length();

    double parameter = MyMath::Vector3::dot(point - start, segment) / lengthSquared;
    parameter = (std::max)(0.0, (std::min)(1.0, parameter));
    return (point - (start + segment * parameter)).length();
}

MyMath::Vector3 edgePoint(const MyBRep::Topology_Edge& edge, double parameter)
{
    const MyMath::Vector3 point = edge.pointAt(parameter);
    MYBREP_ASSERT_MESSAGE(point.isFinite(), "BRep Edge point must be finite.");
    return point;
}

void appendLineSegment(const MyMath::Vector3& start, const MyMath::Vector3& end,
                       std::vector<GLfloat>& vertices, std::vector<GLuint>& indices)
{
    const GLuint firstIndex = static_cast<GLuint>(vertices.size() / 3);

    vertices.push_back(static_cast<GLfloat>(start.x()));
    vertices.push_back(static_cast<GLfloat>(start.y()));
    vertices.push_back(static_cast<GLfloat>(start.z()));

    vertices.push_back(static_cast<GLfloat>(end.x()));
    vertices.push_back(static_cast<GLfloat>(end.y()));
    vertices.push_back(static_cast<GLfloat>(end.z()));

    indices.push_back(firstIndex);
    indices.push_back(firstIndex + 1);
}

bool edgeIntervalFlatEnough(const MyBRep::Topology_Edge& edge, double firstParameter, double lastParameter,
                            const MyMath::Vector3& firstPoint, const MyMath::Vector3& lastPoint, double tolerance)
{
    const double parameterSpan = lastParameter - firstParameter;
    const double quarterParameter = firstParameter + parameterSpan * 0.25;
    const double middleParameter = firstParameter + parameterSpan * 0.5;
    const double threeQuarterParameter = firstParameter + parameterSpan * 0.75;

    const MyMath::Vector3 quarterPoint = edgePoint(edge, quarterParameter);
    const MyMath::Vector3 middlePoint = edgePoint(edge, middleParameter);
    const MyMath::Vector3 threeQuarterPoint = edgePoint(edge, threeQuarterParameter);

    return pointSegmentDistance(quarterPoint, firstPoint, lastPoint) <= tolerance &&
           pointSegmentDistance(middlePoint, firstPoint, lastPoint) <= tolerance &&
           pointSegmentDistance(threeQuarterPoint, firstPoint, lastPoint) <= tolerance;
}

void appendAdaptiveEdgeInterval(const MyBRep::Topology_Edge& edge, double firstParameter, double lastParameter,
                                const MyMath::Vector3& firstPoint, const MyMath::Vector3& lastPoint, int depth,
                                const MyBRep::Display::BRepEdgeBuildOptions& options,
                                std::vector<GLfloat>& vertices, std::vector<GLuint>& indices)
{
    const bool minimumDepthReached = depth >= options.minimumSubdivisionDepth;
    const bool maximumDepthReached = depth >= options.maximumSubdivisionDepth;

    if (maximumDepthReached ||
        (minimumDepthReached && edgeIntervalFlatEnough(edge, firstParameter, lastParameter, firstPoint, lastPoint, options.chordTolerance)))
    {
        appendLineSegment(firstPoint, lastPoint, vertices, indices);
        return;
    }

    const double middleParameter = (firstParameter + lastParameter) * 0.5;
    const MyMath::Vector3 middlePoint = edgePoint(edge, middleParameter);

    appendAdaptiveEdgeInterval(edge, firstParameter, middleParameter, firstPoint, middlePoint, depth + 1, options, vertices, indices);
    appendAdaptiveEdgeInterval(edge, middleParameter, lastParameter, middlePoint, lastPoint, depth + 1, options, vertices, indices);
}

}

namespace MyBRep
{
namespace Display
{

BRepEdgeBuildOptions::BRepEdgeBuildOptions()
    : chordTolerance(1.0e-3)
    , minimumSubdivisionDepth(2)
    , maximumSubdivisionDepth(12)
{
}

bool BRepEdgeBuildOptions::isValid() const
{
    return isFinitePositive(chordTolerance) &&
           minimumSubdivisionDepth >= 0 &&
           maximumSubdivisionDepth >= minimumSubdivisionDepth &&
           maximumSubdivisionDepth <= MaximumAllowedSubdivisionDepth;
}

BufferGeometry* BRepEdgeBuilder::build(const Topology_Edge& edge, const QString& name, const BRepEdgeBuildOptions& options)
{
    MYBREP_ASSERT_MESSAGE(edge.isValid(), "BRep Edge build requires a valid Topology_Edge.");
    MYBREP_ASSERT_MESSAGE(options.isValid(), "BRep Edge build options are invalid.");

    if (!edge.isValid() || !options.isValid()) return 0;

    std::vector<GLfloat> vertices;
    std::vector<GLuint> indices;

    const MyMath::Vector3 firstPoint = edgePoint(edge, 0.0);
    const MyMath::Vector3 lastPoint = edgePoint(edge, 1.0);

    if (edge.geometry().kind() == CurveKind::Line)
    {
        appendLineSegment(firstPoint, lastPoint, vertices, indices);
    }
    else
    {
        appendAdaptiveEdgeInterval(edge, 0.0, 1.0, firstPoint, lastPoint, 0, options, vertices, indices);
    }

    if (vertices.empty() || indices.empty()) return 0;

    BufferGeometry* geometry = new BufferGeometry(name, BufferUsage::Static, RenderType::Lines);

    std::vector<GeometryVertexAttribute> attributes;

    GeometryVertexAttribute positionAttribute;
    positionAttribute.location = GeometryAttribute::Position;
    positionAttribute.componentCount = 3;
    positionAttribute.valueOffset = 0;
    attributes.push_back(positionAttribute);

    geometry->setVertexLayout(3, attributes);
    geometry->setVertexData(vertices);
    geometry->setIndexData(indices);

    return geometry;
}

}
}