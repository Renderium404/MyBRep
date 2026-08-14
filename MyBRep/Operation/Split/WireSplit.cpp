#include "WireSplit.h"

#include "MyBRep/Foundation/Diagnostic.h"
#include "MyBRep/Operation/Split/EdgeSplit.h"

namespace
{

// 判断替换Edge序列是否非空、有效、内部连续，并保持原Edge-use的起终Topology_Vertex身份。
bool isValidReplacement(const MyBRep::Topology_Edge& sourceEdge, const std::vector<MyBRep::Topology_Edge>& replacementEdges)
{
    if (replacementEdges.empty())
    {
        return false;
    }

    if (!replacementEdges.front().isValid() || !replacementEdges.back().isValid())
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

}
}
}