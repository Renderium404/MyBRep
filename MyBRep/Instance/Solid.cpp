#include "Solid.h"

#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

Solid::Solid()
{
}

Solid::Solid(const Topology_Solid& topology)
    : Instance(topology)
{
}

Solid::Solid(const Topology_Solid& topology, const MyMath::Matrix4& localToWorld)
    : Instance(topology, localToWorld)
{
}

Topology_Solid Solid::topology() const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot access the topology of an invalid Solid.");
    return Topology_Solid(topologyObject());
}

std::size_t Solid::shellCount() const
{
    return isValid() ? topology().shellCount() : 0;
}

Topology_Shell Solid::topologyShell(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot access a shell of an invalid Solid.");
    return topology().shell(index);
}

std::vector<Topology_Shell> Solid::topologyShells() const
{
    return isValid() ? topology().shells() : std::vector<Topology_Shell>();
}

Shell Solid::shell(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot access a Shell instance of an invalid Solid.");
    MYBREP_ASSERT_MESSAGE(index < shellCount(), "Solid Shell instance index is out of range.");

    return Shell(topologyShell(index), localToWorld());
}

Solid Solid::reversed() const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot reverse an invalid Solid.");
    return Solid(topology().reversed(), localToWorld());
}

}