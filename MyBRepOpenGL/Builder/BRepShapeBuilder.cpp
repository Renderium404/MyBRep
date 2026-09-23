#include "BRepShapeBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "MyBRep/Foundation/Diagnostic.h"
#include "MyBRep/Geometry/Construction/Geometry_Revolved.h"
#include "MyBRep/Geometry/Curve/CurveKind.h"
#include "MyOpenGL/Resource/BufferGeometry.h"

namespace
{

const int MaximumAllowedProfileSubdivisionDepth = 20;
const int MaximumAllowedRevolutionSegments = 4096;
const double Pi = 3.1415926535897932384626433832795;
const double TwoPi = Pi * 2.0;

struct ProfileSample
{
    MyMath::Vector2 point;
    MyMath::Vector2 tangent;
};

bool isFiniteValue(double value)
{
    const double infinity = (std::numeric_limits<double>::infinity)();
    return value == value && value != infinity && value != -infinity;
}

bool isFinitePositive(double value)
{
    return isFiniteValue(value) && value > 0.0;
}

double pointSegmentDistance(const MyMath::Vector2& point, const MyMath::Vector2& start, const MyMath::Vector2& end)
{
    const MyMath::Vector2 segment = end - start;
    const double lengthSquared = MyMath::Vector2::dot(segment, segment);

    if (lengthSquared <= 0.0) return (point - start).length();

    double parameter = MyMath::Vector2::dot(point - start, segment) / lengthSquared;
    parameter = (std::max)(0.0, (std::min)(1.0, parameter));
    return (point - (start + segment * parameter)).length();
}

MyMath::Vector2 segmentTangent(const MyBRep::Geometry_Revolved::ProfileSegment& segment, double parameter)
{
    MyMath::Vector2 tangent = segment.curve->tangentAt(parameter);
    if (segment.lastParameter < segment.firstParameter) tangent *= -1.0;
    return tangent;
}

bool profileIntervalFlatEnough(const MyBRep::Geometry_Revolved::ProfileSegment& segment,
                               double firstParameter, double lastParameter,
                               const MyMath::Vector2& firstPoint, const MyMath::Vector2& lastPoint,
                               double tolerance)
{
    const double span = lastParameter - firstParameter;
    const double quarterParameter = firstParameter + span * 0.25;
    const double middleParameter = firstParameter + span * 0.5;
    const double threeQuarterParameter = firstParameter + span * 0.75;

    return pointSegmentDistance(segment.curve->pointAt(quarterParameter), firstPoint, lastPoint) <= tolerance &&
           pointSegmentDistance(segment.curve->pointAt(middleParameter), firstPoint, lastPoint) <= tolerance &&
           pointSegmentDistance(segment.curve->pointAt(threeQuarterParameter), firstPoint, lastPoint) <= tolerance;
}

void appendAdaptiveProfileInterval(const MyBRep::Geometry_Revolved::ProfileSegment& segment,
                                   double firstParameter, double lastParameter,
                                   const MyMath::Vector2& firstPoint, const MyMath::Vector2& lastPoint,
                                   int depth, const MyBRep::Display::BRepShapeBuildOptions& options,
                                   std::vector<ProfileSample>& samples)
{
    const bool minimumDepthReached = depth >= options.minimumProfileSubdivisionDepth;
    const bool maximumDepthReached = depth >= options.maximumProfileSubdivisionDepth;

    if (maximumDepthReached ||
        (minimumDepthReached && profileIntervalFlatEnough(segment, firstParameter, lastParameter,
                                                         firstPoint, lastPoint, options.profileChordTolerance)))
    {
        ProfileSample sample;
        sample.point = lastPoint;
        sample.tangent = segmentTangent(segment, lastParameter);
        samples.push_back(sample);
        return;
    }

    const double middleParameter = (firstParameter + lastParameter) * 0.5;
    const MyMath::Vector2 middlePoint = segment.curve->pointAt(middleParameter);

    appendAdaptiveProfileInterval(segment, firstParameter, middleParameter, firstPoint, middlePoint, depth + 1, options, samples);
    appendAdaptiveProfileInterval(segment, middleParameter, lastParameter, middlePoint, lastPoint, depth + 1, options, samples);
}

std::vector<ProfileSample> sampleProfileSegment(const MyBRep::Geometry_Revolved::ProfileSegment& segment,
                                                const MyBRep::Display::BRepShapeBuildOptions& options)
{
    std::vector<ProfileSample> samples;

    ProfileSample first;
    first.point = segment.curve->pointAt(segment.firstParameter);
    first.tangent = segmentTangent(segment, segment.firstParameter);
    samples.push_back(first);

    const MyMath::Vector2 lastPoint = segment.curve->pointAt(segment.lastParameter);

    if (segment.curve->kind() == MyBRep::CurveKind::Line)
    {
        ProfileSample last;
        last.point = lastPoint;
        last.tangent = segmentTangent(segment, segment.lastParameter);
        samples.push_back(last);
        return samples;
    }

    appendAdaptiveProfileInterval(segment, segment.firstParameter, segment.lastParameter,
                                  first.point, lastPoint, 0, options, samples);
    return samples;
}

int revolutionSegmentCount(const MyBRep::Geometry_Revolved& revolved,
                           const MyBRep::Display::BRepShapeBuildOptions& options)
{
    const MyBRep::Bounds3& bounds = revolved.profileBounds();
    const double maximumRadius = (std::max)(std::fabs(bounds.minimum().x()), std::fabs(bounds.maximum().x()));

    if (maximumRadius <= 0.0) return options.minimumRevolutionSegments;

    const double ratio = options.revolutionChordTolerance / maximumRadius;

    if (ratio >= 1.0) return options.minimumRevolutionSegments;

    const double cosine = (std::max)(-1.0, (std::min)(1.0, 1.0 - ratio));
    const double halfAngle = std::acos(cosine);

    if (halfAngle <= 0.0) return options.maximumRevolutionSegments;

    int count = static_cast<int>(std::ceil(Pi / halfAngle));
    count = (std::max)(count, options.minimumRevolutionSegments);
    count = (std::min)(count, options.maximumRevolutionSegments);
    return count;
}

void appendVertex(std::vector<GLfloat>& vertices, const MyMath::Vector3& position, const MyMath::Vector3& normal)
{
    vertices.push_back(static_cast<GLfloat>(position.x()));
    vertices.push_back(static_cast<GLfloat>(position.y()));
    vertices.push_back(static_cast<GLfloat>(position.z()));
    vertices.push_back(static_cast<GLfloat>(normal.x()));
    vertices.push_back(static_cast<GLfloat>(normal.y()));
    vertices.push_back(static_cast<GLfloat>(normal.z()));
}

void appendTriangle(std::vector<GLuint>& indices, GLuint first, GLuint second, GLuint third, bool reversed)
{
    indices.push_back(first);
    indices.push_back(reversed ? third : second);
    indices.push_back(reversed ? second : third);
}

bool appendRevolvedSegment(const MyBRep::Geometry_Revolved& revolved,
                           const MyBRep::Geometry_Revolved::ProfileSegment& segment,
                           int revolutionSegments,
                           const MyBRep::Display::BRepShapeBuildOptions& options,
                           std::vector<GLfloat>& vertices,
                           std::vector<GLuint>& indices)
{
    const std::vector<ProfileSample> samples = sampleProfileSegment(segment, options);
    if (samples.size() < 2) return false;

    const int columnCount = revolutionSegments + 1;
    const GLuint baseIndex = static_cast<GLuint>(vertices.size() / 6);
    const double radialSign = revolved.radialSign();
    const double orientationValue = revolved.profileSignedArea() * radialSign;
    const bool reverseWinding = orientationValue < 0.0;
    const double axisTolerance = revolved.profileTolerance();

    std::vector<double> radii;
    radii.reserve(samples.size());

    for (std::size_t sampleIndex = 0; sampleIndex < samples.size(); ++sampleIndex)
    {
        double radius = radialSign * samples[sampleIndex].point.x();

        if (radius < 0.0 && std::fabs(radius) <= axisTolerance) radius = 0.0;
        if (radius < 0.0) return false;

        radii.push_back(radius);

        const double radialDerivative = radialSign * samples[sampleIndex].tangent.x();
        const double axialDerivative = samples[sampleIndex].tangent.y();

        for (int angleIndex = 0; angleIndex <= revolutionSegments; ++angleIndex)
        {
            const double angle = TwoPi * static_cast<double>(angleIndex) / static_cast<double>(revolutionSegments);
            const double cosine = std::cos(angle);
            const double sine = std::sin(angle);

            const MyMath::Vector3 position(radius * cosine, radius * sine, samples[sampleIndex].point.y());
            MyMath::Vector3 normal(axialDerivative * cosine, axialDerivative * sine, -radialDerivative);

            if (orientationValue < 0.0) normal *= -1.0;
            if (!position.isFinite() || !normal.isVector(0.0)) return false;

            normal.normalize(0.0);
            if (!normal.isUnit()) return false;

            appendVertex(vertices, position, normal);
        }
    }

    for (std::size_t row = 0; row + 1 < samples.size(); ++row)
    {
        for (int angleIndex = 0; angleIndex < revolutionSegments; ++angleIndex)
        {
            const GLuint first = baseIndex + static_cast<GLuint>(row * columnCount + angleIndex);
            const GLuint firstNext = first + 1;
            const GLuint second = baseIndex + static_cast<GLuint>((row + 1) * columnCount + angleIndex);
            const GLuint secondNext = second + 1;

            if (radii[row] > 0.0) appendTriangle(indices, first, firstNext, second, reverseWinding);
            if (radii[row + 1] > 0.0) appendTriangle(indices, second, firstNext, secondNext, reverseWinding);
        }
    }

    return true;
}

}

namespace MyBRep
{
namespace Display
{

BRepShapeBuildOptions::BRepShapeBuildOptions()
    : profileChordTolerance(1.0e-3)
    , revolutionChordTolerance(1.0e-3)
    , minimumProfileSubdivisionDepth(2)
    , maximumProfileSubdivisionDepth(12)
    , minimumRevolutionSegments(24)
    , maximumRevolutionSegments(512)
{
}

bool BRepShapeBuildOptions::isValid() const
{
    return isFinitePositive(profileChordTolerance) &&
           isFinitePositive(revolutionChordTolerance) &&
           minimumProfileSubdivisionDepth >= 0 &&
           maximumProfileSubdivisionDepth >= minimumProfileSubdivisionDepth &&
           maximumProfileSubdivisionDepth <= MaximumAllowedProfileSubdivisionDepth &&
           minimumRevolutionSegments >= 3 &&
           maximumRevolutionSegments >= minimumRevolutionSegments &&
           maximumRevolutionSegments <= MaximumAllowedRevolutionSegments;
}

bool BRepShapeBuilder::canBuild(const Topology_Shape& shape)
{
    return shape.isValid() && shape.geometry().kind() == ShapeKind::Revolved;
}

BufferGeometry* BRepShapeBuilder::build(const Topology_Shape& shape, const QString& name, const BRepShapeBuildOptions& options)
{
    MYBREP_ASSERT_MESSAGE(shape.isValid(), "BRep Shape build requires a valid Topology_Shape.");
    MYBREP_ASSERT_MESSAGE(options.isValid(), "BRep Shape build options are invalid.");

    if (!canBuild(shape) || !options.isValid()) return 0;

    const Geometry_Revolved& revolved = static_cast<const Geometry_Revolved&>(shape.geometry());
    const int angularSegments = revolutionSegmentCount(revolved, options);

    std::vector<GLfloat> vertices;
    std::vector<GLuint> indices;

    for (std::size_t index = 0; index < revolved.profileSegmentCount(); ++index)
    {
        if (!appendRevolvedSegment(revolved, revolved.profileSegment(index), angularSegments, options, vertices, indices))
        {
            return 0;
        }
    }

    if (vertices.empty() || indices.empty()) return 0;

    BufferGeometry* geometry = new BufferGeometry(name, BufferUsage::Static, RenderType::Triangles);

    std::vector<GeometryVertexAttribute> attributes;

    GeometryVertexAttribute positionAttribute;
    positionAttribute.location = GeometryAttribute::Position;
    positionAttribute.componentCount = 3;
    positionAttribute.valueOffset = 0;
    attributes.push_back(positionAttribute);

    GeometryVertexAttribute normalAttribute;
    normalAttribute.location = GeometryAttribute::Normal;
    normalAttribute.componentCount = 3;
    normalAttribute.valueOffset = 3;
    attributes.push_back(normalAttribute);

    geometry->setVertexLayout(6, attributes);
    geometry->setVertexData(vertices);
    geometry->setIndexData(indices);

    return geometry;
}

}
}
