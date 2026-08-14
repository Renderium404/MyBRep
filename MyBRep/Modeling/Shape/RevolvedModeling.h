#ifndef MYBREP_MODELING_SHAPE_REVOLVEDMODELING_H
#define MYBREP_MODELING_SHAPE_REVOLVEDMODELING_H

#include "MyMath/Matrix4.h"
#include "MyBRep/Instance/Shape.h"
#include "MyBRep/Instance/Wire.h"
#include "MyBRep/Topology/Shape/Topology_Shape.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

namespace MyBRep
{
namespace Modeling
{

/// 局部Topology_Shape创建

// 使用局部XY平面闭合Topology_Wire作为有限母线，创建绕局部Z轴完整旋转的Topology_Shape。
Topology_Shape createRevolved(const Topology_Wire& profile, double profileTolerance = MyMath::Vector3::DefaultEpsilon);

/// 空间Shape实例创建

// 使用单位变换创建完整回转Shape实例。
Shape makeRevolved(const Topology_Wire& profile, double profileTolerance = MyMath::Vector3::DefaultEpsilon);

// 使用指定可逆仿射变换创建完整回转Shape实例。
Shape makeRevolved(const Topology_Wire& profile, const MyMath::Matrix4& localToWorld, double profileTolerance = MyMath::Vector3::DefaultEpsilon);

// 使用Wire实例自身空间放置创建完整回转Shape实例。
Shape makeRevolved(const Wire& profile, double profileTolerance = MyMath::Vector3::DefaultEpsilon);

}
}

#endif // MYBREP_MODELING_SHAPE_REVOLVEDMODELING_H
