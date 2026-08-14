#include "Wire.h"

#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

Wire::Wire()
    : Instance_Object()
{
}

Wire::Wire(const Topology_Wire& topology)
    : Instance_Object()
    , m_topology(topology)
{
    MYBREP_ASSERT_MESSAGE(topology.isValid(),
                          "Wire topology must be valid.");
}

Wire::Wire(const Topology_Wire& topology,
           const MyMath::Matrix4& localToWorld)
    : Instance_Object(localToWorld)
    , m_topology(topology)
{
    MYBREP_ASSERT_MESSAGE(topology.isValid(),
                          "Wire topology must be valid.");
}

/// 状态判断

bool Wire::isValid() const
{
    return m_topology.isValid();
}

bool Wire::isNull() const
{
    return m_topology.isNull();
}

Wire::operator bool() const
{
    return isValid();
}

bool Wire::isClosed() const
{
    return isValid() && m_topology.isClosed();
}

/// 局部拓扑

const Topology_Wire& Wire::topology() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access the topology of an invalid Wire.");

    return m_topology;
}

std::size_t Wire::edgeCount() const
{
    return m_topology.edgeCount();
}

Topology_Edge Wire::topologyEdge(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access an edge of an invalid Wire.");

    return m_topology.edge(index);
}

std::vector<Topology_Edge> Wire::topologyEdges() const
{
    return m_topology.edges();
}

/// 子实例

Edge Wire::edge(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access an Edge instance of an invalid Wire.");
    MYBREP_ASSERT_MESSAGE(index < edgeCount(),
                          "Wire Edge instance index is out of range.");

    return Edge(m_topology.edge(index), localToWorld());
}

/// 有向端点

MyMath::Vector3 Wire::localStartPoint() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access the local start point of an invalid Wire.");

    return m_topology.startVertex().point();
}

MyMath::Vector3 Wire::localEndPoint() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access the local end point of an invalid Wire.");

    return m_topology.endVertex().point();
}

MyMath::Vector3 Wire::worldStartPoint() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access the world start point of an invalid Wire.");

    return localToWorld().transformPoint(localStartPoint());
}

MyMath::Vector3 Wire::worldEndPoint() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access the world end point of an invalid Wire.");

    return localToWorld().transformPoint(localEndPoint());
}

/// 方向操作

Wire Wire::reversed() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot reverse an invalid Wire.");

    return Wire(m_topology.reversed(), localToWorld());
}

}
