#ifndef MYBREP_MODELING_SOLID_SOLIDMODELING_H
#define MYBREP_MODELING_SOLID_SOLIDMODELING_H

#include <vector>

#include "MyMath/Matrix4.h"
#include "MyBRep/Instance/Solid.h"
#include "MyBRep/Topology/Shell/Topology_Shell.h"
#include "MyBRep/Topology/Solid/Topology_Solid.h"

namespace MyBRep
{
namespace Modeling
{

/// 局部Topology_Solid创建

// 使用一个有效闭合Topology_Shell创建完整B-Rep Topology_Solid。
Topology_Solid createSolid(const Topology_Shell& shell);

// 使用一个或多个有效闭合Topology_Shell创建完整B-Rep Topology_Solid。
Topology_Solid createSolid(const std::vector<Topology_Shell>& shells);

/// 空间Solid实例创建

// 使用单位变换创建单Shell Solid实例。
Solid makeSolid(const Topology_Shell& shell);

// 使用指定可逆仿射变换创建单Shell Solid实例。
Solid makeSolid(const Topology_Shell& shell,const MyMath::Matrix4& localToWorld);

// 使用单位变换创建多Shell Solid实例。
Solid makeSolid(const std::vector<Topology_Shell>& shells);

// 使用指定可逆仿射变换创建多Shell Solid实例。
Solid makeSolid(const std::vector<Topology_Shell>& shells,const MyMath::Matrix4& localToWorld);

}
}

#endif // MYBREP_MODELING_SOLID_SOLIDMODELING_H
