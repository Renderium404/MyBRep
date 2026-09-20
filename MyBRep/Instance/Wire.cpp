#include "Wire.h"

#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

Wire::Wire()
{
}

Wire::Wire(const Topology_Wire& topology)
    : Instance(topology)
{
}

Wire::Wire(const Topology_Wire& topology, const MyMath::Matrix4& localToWorld)
    : Instance(topology, localToWorld)
{
}

Topology_Wire Wire::topology() const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot access the topology of an invalid Wire.");
    return Topology_Wire(topologyObject());
}

bool Wire::isClosed() const
{
    return isValid() && topology().isClosed();
}

std::size_t Wire::edgeCount() const
{
    return isValid() ? topology().edgeCount() : 0;
}

Topology_Edge Wire::topologyEdge(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot access an edge of an invalid Wire.");
    return topology().edge(index);
}

std::vector<Topology_Edge> Wire::topologyEdges() const
{
    return isValid() ? topology().edges() : std::vector<Topology_Edge>();
}

Edge Wire::edge(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot access an Edge instance of an invalid Wire.");
    MYBREP_ASSERT_MESSAGE(index < edgeCount(), "Wire Edge instance index is out of range.");

    return Edge(topologyEdge(index), localToWorld());
}

MyMath::Vector3 Wire::localStartPoint() const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot access the local start point of an invalid Wire.");
    return topology().startVertex().point();
}

MyMath::Vector3 Wire::localEndPoint() const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot access the local end point of an invalid Wire.");
    return topology().endVertex().point();
}

MyMath::Vector3 Wire::worldStartPoint() const
{
    return localToWorld().transformPoint(localStartPoint());
}

MyMath::Vector3 Wire::worldEndPoint() const
{
    return localToWorld().transformPoint(localEndPoint());
}

Wire Wire::reversed() const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot reverse an invalid Wire.");
    return Wire(topology().reversed(), localToWorld());
}

}