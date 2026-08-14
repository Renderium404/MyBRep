#ifndef MYBREP_OPERATION_SPLIT_WIRESPLIT_H
#define MYBREP_OPERATION_SPLIT_WIRESPLIT_H

#include <cstddef>
#include <vector>

#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

namespace MyBRep
{
namespace Operation
{
namespace Split
{

/// Wire Edge-use替换

// 使用一组按遍历方向连续连接的新Edge-use替换Wire中指定Edge-use，返回保持输入Wire当前遍历方向的新Topology_Wire。
Topology_Wire replaceWireEdge(const Topology_Wire& wire, std::size_t edgeIndex, const std::vector<Topology_Edge>& replacementEdges);

/// Wire Edge-use切分

// 在指定Edge-use当前方向规范化参数(0,1)处切分，并返回保持输入Wire当前遍历方向的新Topology_Wire。
Topology_Wire splitWireEdge(const Topology_Wire& wire, std::size_t edgeIndex, double parameter);

// 在指定Edge-use当前方向的一组严格递增规范化参数处切分，并返回保持输入Wire当前遍历方向的新Topology_Wire。
Topology_Wire splitWireEdge(const Topology_Wire& wire, std::size_t edgeIndex, const std::vector<double>& parameters);

}
}
}

#endif // MYBREP_OPERATION_SPLIT_WIRESPLIT_H