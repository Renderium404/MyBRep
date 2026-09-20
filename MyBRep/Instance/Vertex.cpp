#include "Vertex.h"

#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

Vertex::Vertex()
{
}

Vertex::Vertex(const Topology_Vertex& topology)
    : Instance(topology)
{
}

Vertex::Vertex(const Topology_Vertex& topology, const MyMath::Matrix4& localToWorld)
    : Instance(topology, localToWorld)
{
}

Topology_Vertex Vertex::topology() const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot access the topology of an invalid Vertex.");
    return Topology_Vertex(topologyObject());
}

const MyMath::Vector3& Vertex::localPoint() const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Vertex.");
    return static_cast<const Topology_TVertex*>(topologyObject().tObject().get())->point();
}

MyMath::Vector3 Vertex::worldPoint() const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Vertex.");
    return localToWorld().transformPoint(localPoint());
}

}