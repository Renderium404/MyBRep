#include "WireSplit.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "MyBRep/Foundation/Diagnostic.h"
#include "MyBRep/Operation/Split/EdgeSplit.h"

namespace
{

bool isFiniteValue(double value)
{
    const double infinity = (std::numeric_limits<double>::infinity)();
    return value == value && value != infinity && value != -infinity;
}

// 判断替换Edge序列是否非空、有效、内部连续，并保持原Edge-use的起终Topology_Vertex身份。
bool isValidReplacement(const MyBRep::Topology_Edge& sourceEdge, const std::vector<MyBRep::Topology_Edge>& replacementEdges)
{
    if (replacementEdges.empty() || !replacementEdges.front().isValid() || !replacementEdges.back().isValid())
    {
        return false;
    }

    if (!replacementEdges.front().startVertex().isSame(sourceEdge.startVertex()) ||
        !replacementEdges.back().endVertex().isSame(sourceEdge.endVertex()))
    {
        return false;
    }

    for (std::size_t index = 0; index < replacementEdges.size(); ++index)
    {
        if (!replacementEdges[index].isValid())
        {
            return false;
        }

        if (index + 1 < replacementEdges.size() &&
            !replacementEdges[index].endVertex().isSame(replacementEdges[index + 1].startVertex()))
        {
            return false;
        }
    }

    return true;
}

// 返回指定Vertex作为当前闭合Wire Edge起点出现的唯一索引，同时统计出现次数。
std::size_t findUniqueStartVertexIndex(const MyBRep::Topology_Wire& wire, const MyBRep::Topology_Vertex& vertex, std::size_t& occurrenceCount)
{
    std::size_t result = wire.edgeCount();
    occurrenceCount = 0;

    for (std::size_t index = 0; index < wire.edgeCount(); ++index)
    {
        if (wire.edge(index).startVertex().isSame(vertex))
        {
            result = index;
            ++occurrenceCount;
        }
    }

    return result;
}

// 沿闭合Wire当前遍历方向提取从startVertexIndex到endVertexIndex之前的完整Edge路径。
std::vector<MyBRep::Topology_Edge> collectPath(const MyBRep::Topology_Wire& wire, std::size_t startVertexIndex, std::size_t endVertexIndex)
{
    std::vector<MyBRep::Topology_Edge> result;
    std::size_t index = startVertexIndex;

    while (index != endVertexIndex)
    {
        result.push_back(wire.edge(index));
        index = (index + 1) % wire.edgeCount();
    }

    return result;
}

bool containsEdgeIdentity(const MyBRep::Topology_Wire& wire, const MyBRep::Topology_Edge& edge)
{
    for (std::size_t index = 0; index < wire.edgeCount(); ++index)
    {
        if (wire.edge(index).isSame(edge))
        {
            return true;
        }
    }

    return false;
}

bool isValidBoundaryLocation(const MyBRep::Topology_Wire& wire, const MyBRep::Operation::Split::WireSplitLocation& location)
{
    return location.edgeIndex < wire.edgeCount() && isFiniteValue(location.parameter) &&
           location.parameter >= 0.0 && location.parameter <= 1.0;
}

MyBRep::Topology_Vertex endpointVertex(const MyBRep::Topology_Edge& edge, double parameter)
{
    MYBREP_ASSERT_MESSAGE(parameter == 0.0 || parameter == 1.0, "WireSplit endpoint parameter must be exactly 0 or 1.");
    return parameter == 0.0 ? edge.startVertex() : edge.endVertex();
}

// 在一个源Edge-use上根据最多两个内部参数切分，并返回对应输入位置产生的共享Vertex。
void splitSourceEdge(const MyBRep::Topology_Edge& edge, bool hasFirst, double firstParameter, bool hasSecond, double secondParameter,
                     std::vector<MyBRep::Topology_Edge>& resultEdges, MyBRep::Topology_Vertex& firstVertex, MyBRep::Topology_Vertex& secondVertex)
{
    std::vector<double> parameters;

    if (hasFirst && firstParameter > 0.0 && firstParameter < 1.0)
    {
        parameters.push_back(firstParameter);
    }

    if (hasSecond && secondParameter > 0.0 && secondParameter < 1.0)
    {
        parameters.push_back(secondParameter);
    }

    if (parameters.size() == 2 && parameters[0] > parameters[1])
    {
        std::swap(parameters[0], parameters[1]);
    }

    if (parameters.size() == 2)
    {
        MYBREP_ASSERT_MESSAGE(parameters[0] != parameters[1], "WireSplit boundary locations on the same Edge-use must be different.");
    }

    resultEdges = parameters.empty() ? std::vector<MyBRep::Topology_Edge>(1, edge) : MyBRep::Operation::Split::splitEdge(edge, parameters);

    if (hasFirst)
    {
        if (firstParameter == 0.0 || firstParameter == 1.0)
        {
            firstVertex = endpointVertex(edge, firstParameter);
        }
        else
        {
            const std::size_t splitIndex = parameters.size() == 1 || firstParameter == parameters[0] ? 0U : 1U;
            firstVertex = resultEdges[splitIndex].endVertex();
        }
    }

    if (hasSecond)
    {
        if (secondParameter == 0.0 || secondParameter == 1.0)
        {
            secondVertex = endpointVertex(edge, secondParameter);
        }
        else
        {
            const std::size_t splitIndex = parameters.size() == 1 || secondParameter == parameters[0] ? 0U : 1U;
            secondVertex = resultEdges[splitIndex].endVertex();
        }
    }
}

}

namespace MyBRep
{
namespace Operation
{
namespace Split
{

Topology_Wire replaceWireEdge(const Topology_Wire& wire, std::size_t edgeIndex, const std::vector<Topology_Edge>& replacementEdges)
{
    MYBREP_ASSERT_MESSAGE(wire.isValid(), "WireSplit requires a valid Topology_Wire.");
    MYBREP_ASSERT_MESSAGE(edgeIndex < wire.edgeCount(), "WireSplit Edge index is out of range.");

    const Topology_Edge sourceEdge = wire.edge(edgeIndex);
    MYBREP_ASSERT_MESSAGE(isValidReplacement(sourceEdge, replacementEdges),
                          "WireSplit replacement Edges must be valid, internally connected and preserve the replaced Edge-use endpoint identities.");

    std::vector<Topology_Edge> resultEdges;
    resultEdges.reserve(wire.edgeCount() - 1 + replacementEdges.size());

    for (std::size_t index = 0; index < edgeIndex; ++index)
    {
        resultEdges.push_back(wire.edge(index));
    }

    resultEdges.insert(resultEdges.end(), replacementEdges.begin(), replacementEdges.end());

    for (std::size_t index = edgeIndex + 1; index < wire.edgeCount(); ++index)
    {
        resultEdges.push_back(wire.edge(index));
    }

    const Topology_Wire result(resultEdges);
    MYBREP_ASSERT_MESSAGE(result.isClosed() == wire.isClosed(), "WireSplit Edge replacement must preserve the source Wire open/closed state.");
    return result;
}

Topology_Wire splitWireEdge(const Topology_Wire& wire, std::size_t edgeIndex, double parameter)
{
    MYBREP_ASSERT_MESSAGE(wire.isValid(), "WireSplit requires a valid Topology_Wire.");
    MYBREP_ASSERT_MESSAGE(edgeIndex < wire.edgeCount(), "WireSplit Edge index is out of range.");
    return replaceWireEdge(wire, edgeIndex, splitEdge(wire.edge(edgeIndex), parameter));
}

Topology_Wire splitWireEdge(const Topology_Wire& wire, std::size_t edgeIndex, const std::vector<double>& parameters)
{
    MYBREP_ASSERT_MESSAGE(wire.isValid(), "WireSplit requires a valid Topology_Wire.");
    MYBREP_ASSERT_MESSAGE(edgeIndex < wire.edgeCount(), "WireSplit Edge index is out of range.");
    return replaceWireEdge(wire, edgeIndex, splitEdge(wire.edge(edgeIndex), parameters));
}

WireBoundarySplit splitClosedWireBoundary(const Topology_Wire& wire, const WireSplitLocation& firstLocation, const WireSplitLocation& secondLocation)
{
    MYBREP_ASSERT_MESSAGE(wire.isValid() && wire.isClosed(), "WireSplit boundary subdivision requires a valid closed Topology_Wire.");
    MYBREP_ASSERT_MESSAGE(isValidBoundaryLocation(wire, firstLocation) && isValidBoundaryLocation(wire, secondLocation),
                          "WireSplit boundary locations must reference valid Edge-use indices and normalized parameters in [0,1].");

    WireBoundarySplit result;
    std::vector<Topology_Edge> edges;
    edges.reserve(wire.edgeCount() + 2);

    for (std::size_t edgeIndex = 0; edgeIndex < wire.edgeCount(); ++edgeIndex)
    {
        const bool hasFirst = edgeIndex == firstLocation.edgeIndex;
        const bool hasSecond = edgeIndex == secondLocation.edgeIndex;

        if (!hasFirst && !hasSecond)
        {
            edges.push_back(wire.edge(edgeIndex));
            continue;
        }

        std::vector<Topology_Edge> splitEdges;
        splitSourceEdge(wire.edge(edgeIndex), hasFirst, firstLocation.parameter, hasSecond, secondLocation.parameter,
                        splitEdges, result.firstVertex, result.secondVertex);
        edges.insert(edges.end(), splitEdges.begin(), splitEdges.end());
    }

    result.wire = Topology_Wire(edges);
    MYBREP_ASSERT_MESSAGE(result.wire.isClosed(), "WireSplit boundary subdivision must preserve the closed Wire state.");
    MYBREP_ASSERT_MESSAGE(result.firstVertex.isValid() && result.secondVertex.isValid() && !result.firstVertex.isSame(result.secondVertex),
                          "WireSplit boundary locations must resolve to two different valid Topology_Vertex identities.");
    return result;
}

std::vector<Topology_Wire> splitClosedWireByEdge(const Topology_Wire& wire, const Topology_Edge& splittingEdge)
{
    MYBREP_ASSERT_MESSAGE(wire.isValid() && wire.isClosed(), "WireSplit region split requires a valid closed Topology_Wire.");
    MYBREP_ASSERT_MESSAGE(splittingEdge.isValid(), "WireSplit region split requires a valid splitting Edge.");
    MYBREP_ASSERT_MESSAGE(!splittingEdge.startVertex().isSame(splittingEdge.endVertex()),
                          "WireSplit region splitting Edge endpoints must be different Topology_Vertex identities.");
    MYBREP_ASSERT_MESSAGE(!containsEdgeIdentity(wire, splittingEdge),
                          "WireSplit region splitting Edge must not already be an Edge-use of the source Wire.");

    std::size_t startOccurrenceCount = 0;
    std::size_t endOccurrenceCount = 0;
    const std::size_t startIndex = findUniqueStartVertexIndex(wire, splittingEdge.startVertex(), startOccurrenceCount);
    const std::size_t endIndex = findUniqueStartVertexIndex(wire, splittingEdge.endVertex(), endOccurrenceCount);

    MYBREP_ASSERT_MESSAGE(startOccurrenceCount == 1 && endOccurrenceCount == 1,
                          "WireSplit region splitting Edge endpoints must each occur exactly once as boundary Vertex identities.");
    MYBREP_ASSERT_MESSAGE(startIndex != endIndex, "WireSplit region splitting Edge endpoints must resolve to different boundary vertices.");

    std::vector<Topology_Edge> firstEdges = collectPath(wire, startIndex, endIndex);
    std::vector<Topology_Edge> secondEdges = collectPath(wire, endIndex, startIndex);
    MYBREP_ASSERT_MESSAGE(!firstEdges.empty() && !secondEdges.empty(),
                          "WireSplit region split requires two non-empty boundary paths between the splitting Edge endpoints.");

    firstEdges.push_back(splittingEdge.reversed());
    secondEdges.push_back(splittingEdge);

    std::vector<Topology_Wire> results;
    results.reserve(2);
    results.push_back(Topology_Wire(firstEdges));
    results.push_back(Topology_Wire(secondEdges));

    MYBREP_ASSERT_MESSAGE(results[0].isClosed() && results[1].isClosed(),
                          "WireSplit region split must produce two closed Topology_Wires.");
    return results;
}

}
}
}
