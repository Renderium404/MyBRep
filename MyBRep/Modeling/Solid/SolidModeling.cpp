#include "SolidModeling.h"

namespace MyBRep
{
namespace Modeling
{

/// 局部Topology_Solid创建

Topology_Solid createSolid(
    const Topology_Shell& shell)
{
    return Topology_Solid(shell);
}

Topology_Solid createSolid(
    const std::vector<Topology_Shell>& shells)
{
    return Topology_Solid(shells);
}

/// 空间Solid实例创建

Solid makeSolid(
    const Topology_Shell& shell)
{
    return Solid(createSolid(shell));
}

Solid makeSolid(
    const Topology_Shell& shell,
    const MyMath::Matrix4& localToWorld)
{
    return Solid(createSolid(shell),
                 localToWorld);
}

Solid makeSolid(
    const std::vector<Topology_Shell>& shells)
{
    return Solid(createSolid(shells));
}

Solid makeSolid(
    const std::vector<Topology_Shell>& shells,
    const MyMath::Matrix4& localToWorld)
{
    return Solid(createSolid(shells),
                 localToWorld);
}

}
}
