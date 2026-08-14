#include "Topology_TShell.h"

#include <vector>

#include "MyBRep/Foundation/Diagnostic.h"

namespace
{

// 保存一个共享Edge身份在Shell全部Face边界中的有向使用统计。
struct EdgeUseCount
{
    EdgeUseCount(const MyBRep::Topology_Edge& sourceEdge)
        : edge(sourceEdge)
        , forwardCount(0)
        , reversedCount(0)
    {
    }

    MyBRep::Topology_Edge edge; // 用于识别共享Topology_TEdge身份的代表句柄。
    std::size_t forwardCount; // Forward方向使用次数。
    std::size_t reversedCount; // Reversed方向使用次数。
};

// 判断两个Face是否至少共享一个Topology_TEdge身份。
bool shareEdge(const MyBRep::Topology_Face& first,
               const MyBRep::Topology_Face& second)
{
    for (std::size_t firstWireIndex = 0; firstWireIndex < first.wireCount(); ++firstWireIndex)
    {
        const MyBRep::Topology_Wire firstWire = first.wire(firstWireIndex);

        for (std::size_t firstEdgeIndex = 0; firstEdgeIndex < firstWire.edgeCount(); ++firstEdgeIndex)
        {
            const MyBRep::Topology_Edge firstEdge = firstWire.edge(firstEdgeIndex);

            for (std::size_t secondWireIndex = 0; secondWireIndex < second.wireCount(); ++secondWireIndex)
            {
                const MyBRep::Topology_Wire secondWire = second.wire(secondWireIndex);

                for (std::size_t secondEdgeIndex = 0; secondEdgeIndex < secondWire.edgeCount(); ++secondEdgeIndex)
                {
                    if (firstEdge.isSame(secondWire.edge(secondEdgeIndex)))
                    {
                        return true;
                    }
                }
            }
        }
    }

    return false;
}

// 判断全部Face是否通过共享Topology_TEdge身份组成单一连通分量。
bool isConnectedFaceSet(const std::vector<MyBRep::Topology_Face>& faces)
{
    if (faces.empty())
    {
        return false;
    }

    if (faces.size() == 1)
    {
        return true;
    }

    std::vector<bool> visited(faces.size(), false);
    std::vector<std::size_t> pending;
    pending.push_back(0);
    visited[0] = true;
    std::size_t visitedCount = 0;

    while (!pending.empty())
    {
        const std::size_t current = pending.back();
        pending.pop_back();
        ++visitedCount;

        for (std::size_t candidate = 0; candidate < faces.size(); ++candidate)
        {
            if (visited[candidate])
            {
                continue;
            }

            if (shareEdge(faces[current], faces[candidate]))
            {
                visited[candidate] = true;
                pending.push_back(candidate);
            }
        }
    }

    return visitedCount == faces.size();
}

// 查找指定共享Edge身份对应的使用统计，不存在时创建新记录。
EdgeUseCount& edgeUseCount(std::vector<EdgeUseCount>& counts,
                           const MyBRep::Topology_Edge& edge)
{
    for (std::size_t index = 0; index < counts.size(); ++index)
    {
        if (counts[index].edge.isSame(edge))
        {
            return counts[index];
        }
    }

    counts.push_back(EdgeUseCount(edge));
    return counts.back();
}

// 统计全部Face边界中的共享Edge有向使用，并验证二维流形Edge约束。
std::vector<EdgeUseCount> collectEdgeUseCounts(
    const std::vector<MyBRep::Topology_Face>& faces)
{
    std::vector<EdgeUseCount> counts;

    for (std::size_t faceIndex = 0; faceIndex < faces.size(); ++faceIndex)
    {
        const MyBRep::Topology_Face& face = faces[faceIndex];

        for (std::size_t wireIndex = 0; wireIndex < face.wireCount(); ++wireIndex)
        {
            const MyBRep::Topology_Wire wire = face.wire(wireIndex);

            for (std::size_t edgeIndex = 0; edgeIndex < wire.edgeCount(); ++edgeIndex)
            {
                const MyBRep::Topology_Edge edge = wire.edge(edgeIndex);
                EdgeUseCount& count = edgeUseCount(counts, edge);

                if (edge.isForward())
                {
                    ++count.forwardCount;
                }
                else
                {
                    ++count.reversedCount;
                }

                const std::size_t totalCount =
                    count.forwardCount + count.reversedCount;

                MYBREP_ASSERT_MESSAGE(totalCount <= 2,
                                      "Topology_TShell does not allow a shared Edge to be used more than twice.");

                if (totalCount == 2)
                {
                    MYBREP_ASSERT_MESSAGE(count.forwardCount == 1 &&
                                          count.reversedCount == 1,
                                          "Topology_TShell requires two uses of a shared Edge to have opposite orientations.");
                }
            }
        }
    }

    return counts;
}

// 判断Edge使用统计是否表示无边界闭合Shell。
bool isClosedEdgeUseSet(const std::vector<EdgeUseCount>& counts)
{
    if (counts.empty())
    {
        return false;
    }

    for (std::size_t index = 0; index < counts.size(); ++index)
    {
        if (counts[index].forwardCount != 1 ||
            counts[index].reversedCount != 1)
        {
            return false;
        }
    }

    return true;
}

}

namespace MyBRep
{

Topology_TShell::Topology_TShell(const std::vector<Topology_Face>& faces)
    : m_faces(faces)
    , m_closed(false)
{
    MYBREP_ASSERT_MESSAGE(!m_faces.empty(),
                          "Topology_TShell requires at least one Topology_Face.");

    for (std::size_t index = 0; index < m_faces.size(); ++index)
    {
        MYBREP_ASSERT_MESSAGE(m_faces[index].isValid(),
                              "Topology_TShell faces must be valid.");
    }

    MYBREP_ASSERT_MESSAGE(isConnectedFaceSet(m_faces),
                          "Topology_TShell faces must form one Edge-connected component.");

    const std::vector<EdgeUseCount> edgeUseCounts =
        collectEdgeUseCounts(m_faces);

    m_closed = isClosedEdgeUseSet(edgeUseCounts);
}

/// Face集合

std::size_t Topology_TShell::faceCount() const
{
    return m_faces.size();
}

const Topology_Face& Topology_TShell::face(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(index < m_faces.size(),
                          "Topology_TShell face index is out of range.");

    return m_faces[index];
}

const std::vector<Topology_Face>& Topology_TShell::faces() const
{
    return m_faces;
}

/// 拓扑状态

bool Topology_TShell::isClosed() const
{
    return m_closed;
}

}
