#include "Shell.h"

#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

Shell::Shell()
{
}

Shell::Shell(const Topology_Shell& topology)
    : Instance(topology)
{
}

Shell::Shell(const Topology_Shell& topology, const MyMath::Matrix4& localToWorld)
    : Instance(topology, localToWorld)
{
}

Topology_Shell Shell::topology() const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot access the topology of an invalid Shell.");
    return Topology_Shell(topologyObject());
}

bool Shell::isClosed() const
{
    return isValid() && topology().isClosed();
}

std::size_t Shell::faceCount() const
{
    return isValid() ? topology().faceCount() : 0;
}

Topology_Face Shell::topologyFace(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot access a face of an invalid Shell.");
    return topology().face(index);
}

std::vector<Topology_Face> Shell::topologyFaces() const
{
    return isValid() ? topology().faces() : std::vector<Topology_Face>();
}

Face Shell::face(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot access a Face instance of an invalid Shell.");
    MYBREP_ASSERT_MESSAGE(index < faceCount(), "Shell Face instance index is out of range.");

    return Face(topologyFace(index), localToWorld());
}

Shell Shell::reversed() const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot reverse an invalid Shell.");
    return Shell(topology().reversed(), localToWorld());
}

}