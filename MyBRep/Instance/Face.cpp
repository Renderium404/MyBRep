#include "Face.h"

#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

Face::Face()
    : Instance_Object()
{
}

Face::Face(const Topology_Face& topology)
    : Instance_Object()
    , m_topology(topology)
{
    MYBREP_ASSERT_MESSAGE(topology.isValid(),
                          "Face topology must be valid.");
}

Face::Face(const Topology_Face& topology,
           const MyMath::Matrix4& localToWorld)
    : Instance_Object(localToWorld)
    , m_topology(topology)
{
    MYBREP_ASSERT_MESSAGE(topology.isValid(),
                          "Face topology must be valid.");
}

/// 状态判断

bool Face::isValid() const
{
    return m_topology.isValid();
}

bool Face::isNull() const
{
    return m_topology.isNull();
}

Face::operator bool() const
{
    return isValid();
}

bool Face::sharesTopologyWith(const Face& other) const
{
    return m_topology.isValid() &&
           other.m_topology.isValid() &&
           m_topology.isSame(other.m_topology);
}

bool Face::sharesGeometryWith(const Face& other) const
{
    const Geometry_Surface* currentGeometry = geometryPointer();
    const Geometry_Surface* otherGeometry = other.geometryPointer();

    return currentGeometry && currentGeometry == otherGeometry;
}

/// 局部拓扑与曲面几何

const Topology_Face& Face::topology() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access the topology of an invalid Face.");

    return m_topology;
}

const Geometry_Surface& Face::geometry() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access the geometry of an invalid Face.");

    return m_topology.geometry();
}

const Geometry_Surface* Face::geometryPointer() const
{
    return m_topology.isValid() ? &m_topology.geometry() : 0;
}

SurfaceKind Face::kind() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access the kind of an invalid Face.");

    return geometry().kind();
}

/// 裁剪Wire

std::size_t Face::wireCount() const
{
    return m_topology.wireCount();
}

Topology_Wire Face::topologyWire(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access a wire of an invalid Face.");

    return m_topology.wire(index);
}

std::vector<Topology_Wire> Face::topologyWires() const
{
    return m_topology.wires();
}

Wire Face::wire(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access a Wire instance of an invalid Face.");
    MYBREP_ASSERT_MESSAGE(index < wireCount(),
                          "Face Wire instance index is out of range.");

    return Wire(m_topology.wire(index), localToWorld());
}

/// 方向操作

Face Face::reversed() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot reverse an invalid Face.");

    return Face(m_topology.reversed(), localToWorld());
}

}
