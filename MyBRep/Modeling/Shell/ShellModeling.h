#ifndef MYBREP_MODELING_SHELL_SHELLMODELING_H
#define MYBREP_MODELING_SHELL_SHELLMODELING_H

#include <vector>

#include "MyMath/Matrix4.h"
#include "MyBRep/Instance/Shell.h"
#include "MyBRep/Topology/Face/Topology_Face.h"
#include "MyBRep/Topology/Shell/Topology_Shell.h"

namespace MyBRep
{
namespace Modeling
{

/// 局部Topology_Shell创建

// 使用通过共享Topology_TEdge身份连通的有向Face集合创建开放或闭合Topology_Shell。
Topology_Shell createShell(
    const std::vector<Topology_Face>& faces);

/// 空间Shell实例创建

// 使用单位变换创建Shell实例。
Shell makeShell(
    const std::vector<Topology_Face>& faces);

// 使用指定可逆仿射变换创建Shell实例。
Shell makeShell(
    const std::vector<Topology_Face>& faces,
    const MyMath::Matrix4& localToWorld);

}
}

#endif // MYBREP_MODELING_SHELL_SHELLMODELING_H
