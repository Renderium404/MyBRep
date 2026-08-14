#include "Solid.h"

#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

Solid::Solid()
    : Instance_Object()
{
}

Solid::Solid(const Topology_Solid& topology)
    : Instance_Object()
    , m_topology(topology)
{
    MYBREP_ASSERT_MESSAGE(topology.isValid(),
                          "Solid topology must be valid.");
}

Solid::Solid(const Topology_Solid& topology,
             const MyMath::Matrix4& localToWorld)
    : Instance_Object(localToWorld)
    , m_topology(topology)
{
    MYBREP_ASSERT_MESSAGE(topology.isValid(),
                          "Solid topology must be valid.");
}

/// 状态判断

bool Solid::isValid() const
{
    return m_topology.isValid();
}

bool Solid::isNull() const
{
    return m_topology.isNull();
}

Solid::operator bool() const
{
    return isValid();
}

/// 局部拓扑

const Topology_Solid& Solid::topology() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access the topology of an invalid Solid.");

    return m_topology;
}

std::size_t Solid::shellCount() const
{
    return m_topology.shellCount();
}

Topology_Shell Solid::topologyShell(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access a shell of an invalid Solid.");

    return m_topology.shell(index);
}

std::vector<Topology_Shell> Solid::topologyShells() const
{
    return m_topology.shells();
}

/// 子实例

Shell Solid::shell(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access a Shell instance of an invalid Solid.");
    MYBREP_ASSERT_MESSAGE(index < shellCount(),
                          "Solid Shell instance index is out of range.");

    return Shell(m_topology.shell(index), localToWorld());
}

/// 方向操作

Solid Solid::reversed() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot reverse an invalid Solid.");

    return Solid(m_topology.reversed(), localToWorld());
}

}
