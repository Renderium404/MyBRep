#include "Vertex.h"

#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

Vertex::Vertex()
    : Instance_Object()
{
}

Vertex::Vertex(const Topology_Vertex& topology)
    : Instance_Object()
    , m_topology(topology)
{
    MYBREP_ASSERT_MESSAGE(topology.isValid(),
                          "Vertex topology must be valid.");
}

Vertex::Vertex(const Topology_Vertex& topology,
               const MyMath::Matrix4& localToWorld)
    : Instance_Object(localToWorld)
    , m_topology(topology)
{
    MYBREP_ASSERT_MESSAGE(topology.isValid(),
                          "Vertex topology must be valid.");
}

/// 状态判断

bool Vertex::isValid() const
{
    return m_topology.isValid();
}

bool Vertex::isNull() const
{
    return m_topology.isNull();
}

Vertex::operator bool() const
{
    return isValid();
}

/// 局部拓扑

const Topology_Vertex& Vertex::topology() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access the topology of an invalid Vertex.");

    return m_topology;
}

/// 空间查询

const MyMath::Vector3& Vertex::localPoint() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot query an invalid Vertex.");

    return m_topology.point();
}

MyMath::Vector3 Vertex::worldPoint() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot query an invalid Vertex.");

    return localToWorld().transformPoint(m_topology.point());
}

}
