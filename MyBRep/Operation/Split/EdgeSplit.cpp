#include "EdgeSplit.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "MyBRep/Foundation/Diagnostic.h"
#include "MyBRep/Topology/Topology_Builder.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"

namespace
{

// 判断标量是否为有限值。
bool isFiniteValue(double value)
{
    const double infinity = (std::numeric_limits<double>::infinity)();
    return value == value && value != infinity && value != -infinity;
}

// 校验切分参数严格位于当前Edge使用方向的(0,1)内并保持严格递增。
bool areValidSplitParameters(const std::vector<double>& parameters)
{
    if (parameters.empty())
    {
        return false;
    }

    for (std::size_t index = 0; index < parameters.size(); ++index)
    {
        if (!isFiniteValue(parameters[index]) || parameters[index] <= 0.0 || parameters[index] >= 1.0)
        {
            return false;
        }

        if (index > 0 && parameters[index] <= parameters[index - 1])
        {
            return false;
        }
    }

    return true;
}

// 返回重建Edge端点与同一Geometry_Curve精确求值之间所需的最小数值容差。
double endpointTolerance(const MyMath::Vector3& startVertexPoint, const MyMath::Vector3& startGeometryPoint,
                         const MyMath::Vector3& endVertexPoint, const MyMath::Vector3& endGeometryPoint)
{
    const double startError = (startVertexPoint - startGeometryPoint).length();
    const double endError = (endVertexPoint - endGeometryPoint).length();
    double scale = 1.0;

    scale = (std::max)(scale, std::fabs(startVertexPoint.x()));
    scale = (std::max)(scale, std::fabs(startVertexPoint.y()));
    scale = (std::max)(scale, std::fabs(startVertexPoint.z()));
    scale = (std::max)(scale, std::fabs(endVertexPoint.x()));
    scale = (std::max)(scale, std::fabs(endVertexPoint.y()));
    scale = (std::max)(scale, std::fabs(endVertexPoint.z()));

    const double numericalTolerance = scale * (std::numeric_limits<double>::epsilon)() * 64.0; // 覆盖周期曲线端点等重复求值的舍入误差。
    return (std::max)((std::max)(startError, endError), numericalTolerance);
}

// 将输入Edge使用方向上的递增切分参数转换为底层TEdge Forward方向上的递增参数。
std::vector<double> forwardSplitParameters(const MyBRep::Topology_Edge& edge, const std::vector<double>& parameters)
{
    if (edge.isForward())
    {
        return parameters;
    }

    std::vector<double> result;
    result.reserve(parameters.size());

    for (std::size_t index = 0; index < parameters.size(); ++index)
    {
        result.push_back(1.0 - parameters[parameters.size() - 1 - index]);
    }

    return result;
}

}

namespace MyBRep
{
namespace Operation
{
namespace Split
{

std::vector<Topology_Edge> splitEdge(const Topology_Edge& edge, double parameter)
{
    std::vector<double> parameters;
    parameters.push_back(parameter);
    return splitEdge(edge, parameters);
}

std::vector<Topology_Edge> splitEdge(const Topology_Edge& edge, const std::vector<double>& parameters)
{
    MYBREP_ASSERT_MESSAGE(edge.isValid(), "EdgeSplit requires a valid Topology_Edge.");
    MYBREP_ASSERT_MESSAGE(areValidSplitParameters(parameters),
                          "EdgeSplit parameters must be finite, strictly increasing and strictly inside the normalized interval (0,1).");

    const Topology_Edge forwardEdge = edge.isForward() ? edge : edge.reversed();
    const std::vector<double> splitParameters = forwardSplitParameters(edge, parameters);

    std::vector<double> boundaries;
    boundaries.reserve(splitParameters.size() + 2);
    boundaries.push_back(0.0);
    boundaries.insert(boundaries.end(), splitParameters.begin(), splitParameters.end());
    boundaries.push_back(1.0);

    std::vector<Topology_Vertex> vertices;
    vertices.reserve(boundaries.size());
    vertices.push_back(forwardEdge.startVertex());

    for (std::size_t index = 0; index < splitParameters.size(); ++index)
    {
        vertices.push_back(Topology_Vertex(forwardEdge.pointAt(splitParameters[index])));
    }

    vertices.push_back(forwardEdge.endVertex());

    std::vector<Topology_Edge> forwardResults;
    forwardResults.reserve(boundaries.size() - 1);

    for (std::size_t index = 0; index + 1 < boundaries.size(); ++index)
    {
        const double firstNormalizedParameter = boundaries[index];
        const double lastNormalizedParameter = boundaries[index + 1];
        const double firstCurveParameter = forwardEdge.curveParameterAt(firstNormalizedParameter);
        const double lastCurveParameter = forwardEdge.curveParameterAt(lastNormalizedParameter);
        const MyMath::Vector3 firstGeometryPoint = forwardEdge.geometry().pointAt(firstCurveParameter);
        const MyMath::Vector3 lastGeometryPoint = forwardEdge.geometry().pointAt(lastCurveParameter);
        const double tolerance = endpointTolerance(vertices[index].point(), firstGeometryPoint,
                                                   vertices[index + 1].point(), lastGeometryPoint);

        Topology_Edge result(vertices[index], vertices[index + 1], forwardEdge.geometryResource(),
                             firstCurveParameter, lastCurveParameter, tolerance);

        Topology_Builder::copyCurveOnSurfaceRange(forwardEdge, firstNormalizedParameter, lastNormalizedParameter, result);
        forwardResults.push_back(result);
    }

    if (edge.isForward())
    {
        return forwardResults;
    }

    std::vector<Topology_Edge> results;
    results.reserve(forwardResults.size());

    for (std::size_t index = 0; index < forwardResults.size(); ++index)
    {
        results.push_back(forwardResults[forwardResults.size() - 1 - index].reversed());
    }

    return results;
}

}
}
}
