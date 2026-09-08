#include "BRepWireframeBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "MyBRep/Foundation/Diagnostic.h"
#include "MyBRep/Geometry/Curve/CurveKind.h"
#include "MyOpenGL/Resource/BufferGeometry.h"

namespace
{

const int MaximumAllowedSubdivisionDepth = 20; // 防止错误配置导致单条Edge产生超过百万级递归叶节点。

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

    if (lengthSquared <= 0.0)
    {
        return (point - start).length();
    }

    double parameter = MyMath::Vector3::dot(point - start, segment) / lengthSquared;
    parameter = (std::max)(0.0, (std::min)(1.0, parameter));
    return (point - (start + segment * parameter)).length();
}

MyMath::Vector3 transformedPoint(const MyBRep::Topology_Edge& edge, double parameter, const MyMath::Matrix4& localToWorld)
{
    const MyMath::Vector3 point = localToWorld.transformPoint(edge.pointAt(parameter));
    MYBREP_ASSERT_MESSAGE(point.isFinite(), "BRep wireframe transformed Edge point must be finite.");
    return point;
}

void appendLineSegment(const MyMath::Vector3& start, const MyMath::Vector3& end, std::vector<GLfloat>& vertices, std::vector<GLuint>& indices)
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

bool edgeIntervalFlatEnough(const MyBRep::Topology_Edge& edge, double firstParameter, double lastParameter, const MyMath::Vector3& firstPoint,
                            const MyMath::Vector3& lastPoint, const MyMath::Matrix4& localToWorld, double tolerance)
{
    const double parameterSpan = lastParameter - firstParameter;
    const double quarterParameter = firstParameter + parameterSpan * 0.25; // 1/4与3/4采样用于避免S形曲线中点恰落在弦上的退化。
    const double middleParameter = firstParameter + parameterSpan * 0.5;
    const double threeQuarterParameter = firstParameter + parameterSpan * 0.75;

    const MyMath::Vector3 quarterPoint = transformedPoint(edge, quarterParameter, localToWorld);
    const MyMath::Vector3 middlePoint = transformedPoint(edge, middleParameter, localToWorld);
    const MyMath::Vector3 threeQuarterPoint = transformedPoint(edge, threeQuarterParameter, localToWorld);

    return pointSegmentDistance(quarterPoint, firstPoint, lastPoint) <= tolerance &&
           pointSegmentDistance(middlePoint, firstPoint, lastPoint) <= tolerance &&
           pointSegmentDistance(threeQuarterPoint, firstPoint, lastPoint) <= tolerance;
}

void appendAdaptiveEdgeInterval(const MyBRep::Topology_Edge& edge, double firstParameter, double lastParameter, const MyMath::Vector3& firstPoint,
                                const MyMath::Vector3& lastPoint, int depth, const MyMath::Matrix4& localToWorld,
                                const MyBRep::Display::BRepWireframeBuildOptions& options, std::vector<GLfloat>& vertices, std::vector<GLuint>& indices)
{
    const bool minimumDepthReached = depth >= options.minimumSubdivisionDepth;
    const bool maximumDepthReached = depth >= options.maximumSubdivisionDepth;

    if (maximumDepthReached || (minimumDepthReached && edgeIntervalFlatEnough(edge, firstParameter, lastParameter, firstPoint, lastPoint, localToWorld, options.chordTolerance)))
    {
        appendLineSegment(firstPoint, lastPoint, vertices, indices);
        return;
    }

    const double middleParameter = (firstParameter + lastParameter) * 0.5;
    const MyMath::Vector3 middlePoint = transformedPoint(edge, middleParameter, localToWorld);

    appendAdaptiveEdgeInterval(edge, firstParameter, middleParameter, firstPoint, middlePoint, depth + 1, localToWorld, options, vertices, indices);
    appendAdaptiveEdgeInterval(edge, middleParameter, lastParameter, middlePoint, lastPoint, depth + 1, localToWorld, options, vertices, indices);
}

void appendEdgeGeometry(const MyBRep::Topology_Edge& edge, const MyMath::Matrix4& localToWorld, const MyBRep::Display::BRepWireframeBuildOptions& options,
                        std::vector<GLfloat>& vertices, std::vector<GLuint>& indices)
{
    MYBREP_ASSERT_MESSAGE(edge.isValid(), "BRep wireframe requires a valid Topology_Edge.");

    const MyMath::Vector3 firstPoint = transformedPoint(edge, 0.0, localToWorld);
    const MyMath::Vector3 lastPoint = transformedPoint(edge, 1.0, localToWorld);

    if (edge.geometry().kind() == MyBRep::CurveKind::Line)
    {
        appendLineSegment(firstPoint, lastPoint, vertices, indices);
        return;
    }

    appendAdaptiveEdgeInterval(edge, 0.0, 1.0, firstPoint, lastPoint, 0, localToWorld, options, vertices, indices);
}

bool containsSameEdge(const std::vector<MyBRep::Topology_Edge>& edges, const MyBRep::Topology_Edge& candidate)
{
    for (std::size_t index = 0; index < edges.size(); ++index)
    {
        if (edges[index].isSame(candidate))
        {
            return true;
        }
    }

    return false;
}

void appendUniqueEdge(std::vector<MyBRep::Topology_Edge>& edges, const MyBRep::Topology_Edge& edge)
{
    if (!containsSameEdge(edges, edge))
    {
        edges.push_back(edge);
    }
}

void collectWireEdges(const MyBRep::Topology_Wire& wire, std::vector<MyBRep::Topology_Edge>& edges, bool uniqueOnly)
{
    MYBREP_ASSERT_MESSAGE(wire.isValid(), "BRep wireframe requires a valid Topology_Wire.");

    for (std::size_t index = 0; index < wire.edgeCount(); ++index)
    {
        const MyBRep::Topology_Edge edge = wire.edge(index);

        if (uniqueOnly)
        {
            appendUniqueEdge(edges, edge);
        }
        else
        {
            edges.push_back(edge);
        }
    }
}

void collectFaceEdges(const MyBRep::Topology_Face& face, std::vector<MyBRep::Topology_Edge>& edges)
{
    MYBREP_ASSERT_MESSAGE(face.isValid(), "BRep wireframe requires a valid Topology_Face.");

    for (std::size_t wireIndex = 0; wireIndex < face.wireCount(); ++wireIndex)
    {
        collectWireEdges(face.wire(wireIndex), edges, true);
    }
}

void collectShellEdges(const MyBRep::Topology_Shell& shell, std::vector<MyBRep::Topology_Edge>& edges)
{
    MYBREP_ASSERT_MESSAGE(shell.isValid(), "BRep wireframe requires a valid Topology_Shell.");

    for (std::size_t faceIndex = 0; faceIndex < shell.faceCount(); ++faceIndex)
    {
        collectFaceEdges(shell.face(faceIndex), edges);
    }
}

void collectSolidEdges(const MyBRep::Topology_Solid& solid, std::vector<MyBRep::Topology_Edge>& edges)
{
    MYBREP_ASSERT_MESSAGE(solid.isValid(), "BRep wireframe requires a valid Topology_Solid.");

    for (std::size_t shellIndex = 0; shellIndex < solid.shellCount(); ++shellIndex)
    {
        collectShellEdges(solid.shell(shellIndex), edges);
    }
}

BufferGeometry* buildGeometry(const std::vector<MyBRep::Topology_Edge>& edges, const MyMath::Matrix4& localToWorld, const QString& name,
                              const MyBRep::Display::BRepWireframeBuildOptions& options)
{
    MYBREP_ASSERT_MESSAGE(localToWorld.isAffine() && localToWorld.isInvertible(), "BRep wireframe transform must be an invertible affine Matrix4.");
    MYBREP_ASSERT_MESSAGE(options.isValid(), "BRep wireframe build options are invalid.");

    if (edges.empty() || !localToWorld.isAffine() || !localToWorld.isInvertible() || !options.isValid())
    {
        return 0;
    }

    std::vector<GLfloat> vertices;
    std::vector<GLuint> indices;

    for (std::size_t edgeIndex = 0; edgeIndex < edges.size(); ++edgeIndex)
    {
        appendEdgeGeometry(edges[edgeIndex], localToWorld, options, vertices, indices);
    }

    if (vertices.empty() || indices.empty())
    {
        return 0;
    }

    BufferGeometry* geometry = new BufferGeometry(name, BufferUsage::Static, RenderType::Lines);
    std::vector<GeometryVertexAttribute> attributes;

    GeometryVertexAttribute position;
    position.location = GeometryAttribute::Position;
    position.componentCount = 3;
    position.valueOffset = 0;
    attributes.push_back(position);

    geometry->setVertexLayout(3, attributes);
    geometry->setVertexData(vertices);
    geometry->setIndexData(indices);
    geometry->setLineWidth(options.lineWidth);

    return geometry;
}

}

namespace MyBRep
{
namespace Display
{

BRepWireframeBuildOptions::BRepWireframeBuildOptions()
    : chordTolerance(1.0e-3)       // 默认1e-3世界单位弦误差，首轮显示可由调用方按模型尺度覆盖。
    , minimumSubdivisionDepth(2)   // 非直线至少四等分，降低S形曲线局部曲率被单点采样漏掉的风险。
    , maximumSubdivisionDepth(12)  // 最多4096个参数区间，限制异常高曲率或过小容差造成的细分规模。
    , lineWidth(1.5f)              // 默认1.5 Pixel边线宽度，保证普通模型边界可见。
{
}

bool BRepWireframeBuildOptions::isValid() const
{
    return isFinitePositive(chordTolerance) && minimumSubdivisionDepth >= 0 && maximumSubdivisionDepth >= minimumSubdivisionDepth &&
           maximumSubdivisionDepth <= MaximumAllowedSubdivisionDepth && isFinitePositive(static_cast<double>(lineWidth));
}

BufferGeometry* BRepWireframeBuilder::build(const Topology_Edge& edge, const QString& name, const BRepWireframeBuildOptions& options)
{
    return build(edge, MyMath::Matrix4::identity(), name, options);
}

BufferGeometry* BRepWireframeBuilder::build(const Topology_Edge& edge, const MyMath::Matrix4& localToWorld, const QString& name, const BRepWireframeBuildOptions& options)
{
    if (!edge.isValid())
    {
        return 0;
    }

    std::vector<Topology_Edge> edges(1, edge);
    return buildGeometry(edges, localToWorld, name, options);
}

BufferGeometry* BRepWireframeBuilder::build(const Topology_Wire& wire, const QString& name, const BRepWireframeBuildOptions& options)
{
    return build(wire, MyMath::Matrix4::identity(), name, options);
}

BufferGeometry* BRepWireframeBuilder::build(const Topology_Wire& wire, const MyMath::Matrix4& localToWorld, const QString& name, const BRepWireframeBuildOptions& options)
{
    if (!wire.isValid())
    {
        return 0;
    }

    std::vector<Topology_Edge> edges;
    edges.reserve(wire.edgeCount());
    collectWireEdges(wire, edges, false);
    return buildGeometry(edges, localToWorld, name, options);
}

BufferGeometry* BRepWireframeBuilder::build(const Topology_Face& face, const QString& name, const BRepWireframeBuildOptions& options)
{
    return build(face, MyMath::Matrix4::identity(), name, options);
}

BufferGeometry* BRepWireframeBuilder::build(const Topology_Face& face, const MyMath::Matrix4& localToWorld, const QString& name, const BRepWireframeBuildOptions& options)
{
    if (!face.isValid())
    {
        return 0;
    }

    std::vector<Topology_Edge> edges;
    collectFaceEdges(face, edges);
    return buildGeometry(edges, localToWorld, name, options);
}

BufferGeometry* BRepWireframeBuilder::build(const Topology_Shell& shell, const QString& name, const BRepWireframeBuildOptions& options)
{
    return build(shell, MyMath::Matrix4::identity(), name, options);
}

BufferGeometry* BRepWireframeBuilder::build(const Topology_Shell& shell, const MyMath::Matrix4& localToWorld, const QString& name, const BRepWireframeBuildOptions& options)
{
    if (!shell.isValid())
    {
        return 0;
    }

    std::vector<Topology_Edge> edges;
    collectShellEdges(shell, edges);
    return buildGeometry(edges, localToWorld, name, options);
}

BufferGeometry* BRepWireframeBuilder::build(const Topology_Solid& solid, const QString& name, const BRepWireframeBuildOptions& options)
{
    return build(solid, MyMath::Matrix4::identity(), name, options);
}

BufferGeometry* BRepWireframeBuilder::build(const Topology_Solid& solid, const MyMath::Matrix4& localToWorld, const QString& name, const BRepWireframeBuildOptions& options)
{
    if (!solid.isValid())
    {
        return 0;
    }

    std::vector<Topology_Edge> edges;
    collectSolidEdges(solid, edges);
    return buildGeometry(edges, localToWorld, name, options);
}

}
}
