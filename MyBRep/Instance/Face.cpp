#include "Face.h"

#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

Face::Face()
{
}

Face::Face(const Topology_Face& topology)
    : Instance(topology)
{
}

Face::Face(const Topology_Face& topology, const MyMath::Matrix4& localToWorld)
    : Instance(topology, localToWorld)
{
}

Topology_Face Face::topology() const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot access the topology of an invalid Face.");
    return Topology_Face(topologyObject());
}

bool Face::sharesGeometryWith(const Face& other) const
{
    const Geometry_Surface* currentGeometry = geometryPointer();
    const Geometry_Surface* otherGeometry = other.geometryPointer();
    return currentGeometry && currentGeometry == otherGeometry;
}

const Geometry_Surface& Face::geometry() const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot access the geometry of an invalid Face.");
    return topology().geometry();
}

const Geometry_Surface* Face::geometryPointer() const
{
    return isValid() ? &geometry() : 0;
}

SurfaceKind Face::kind() const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot access the kind of an invalid Face.");
    return geometry().kind();
}

std::size_t Face::wireCount() const
{
    return isValid() ? topology().wireCount() : 0;
}

Topology_Wire Face::topologyWire(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot access a wire of an invalid Face.");
    return topology().wire(index);
}

std::vector<Topology_Wire> Face::topologyWires() const
{
    return isValid() ? topology().wires() : std::vector<Topology_Wire>();
}

Wire Face::wire(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot access a Wire instance of an invalid Face.");
    MYBREP_ASSERT_MESSAGE(index < wireCount(), "Face Wire instance index is out of range.");

    return Wire(topologyWire(index), localToWorld());
}

Face Face::reversed() const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot reverse an invalid Face.");
    return Face(topology().reversed(), localToWorld());
}

}