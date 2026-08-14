#include "Shell.h"

#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

Shell::Shell()
    : Instance_Object()
{
}

Shell::Shell(const Topology_Shell& topology)
    : Instance_Object()
    , m_topology(topology)
{
    MYBREP_ASSERT_MESSAGE(topology.isValid(),
                          "Shell topology must be valid.");
}

Shell::Shell(const Topology_Shell& topology,
             const MyMath::Matrix4& localToWorld)
    : Instance_Object(localToWorld)
    , m_topology(topology)
{
    MYBREP_ASSERT_MESSAGE(topology.isValid(),
                          "Shell topology must be valid.");
}

/// 状态判断

bool Shell::isValid() const
{
    return m_topology.isValid();
}

bool Shell::isNull() const
{
    return m_topology.isNull();
}

Shell::operator bool() const
{
    return isValid();
}

bool Shell::isClosed() const
{
    return isValid() && m_topology.isClosed();
}

/// 局部拓扑

const Topology_Shell& Shell::topology() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access the topology of an invalid Shell.");

    return m_topology;
}

std::size_t Shell::faceCount() const
{
    return m_topology.faceCount();
}

Topology_Face Shell::topologyFace(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access a face of an invalid Shell.");

    return m_topology.face(index);
}

std::vector<Topology_Face> Shell::topologyFaces() const
{
    return m_topology.faces();
}

/// 子实例

Face Shell::face(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access a Face instance of an invalid Shell.");
    MYBREP_ASSERT_MESSAGE(index < faceCount(),
                          "Shell Face instance index is out of range.");

    return Face(m_topology.face(index), localToWorld());
}

/// 方向操作

Shell Shell::reversed() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot reverse an invalid Shell.");

    return Shell(m_topology.reversed(), localToWorld());
}

}
