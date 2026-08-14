#include "ShellModeling.h"

namespace MyBRep
{
namespace Modeling
{

/// 局部Topology_Shell创建

Topology_Shell createShell(
    const std::vector<Topology_Face>& faces)
{
    return Topology_Shell(faces);
}

/// 空间Shell实例创建

Shell makeShell(
    const std::vector<Topology_Face>& faces)
{
    return Shell(createShell(faces));
}

Shell makeShell(
    const std::vector<Topology_Face>& faces,
    const MyMath::Matrix4& localToWorld)
{
    return Shell(createShell(faces),
                 localToWorld);
}

}
}
