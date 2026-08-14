#ifndef MYBREP_OPERATION_SPLIT_EDGESPLIT_H
#define MYBREP_OPERATION_SPLIT_EDGESPLIT_H

#include <vector>

#include "MyBRep/Topology/Edge/Topology_Edge.h"

namespace MyBRep
{
namespace Operation
{
namespace Split
{

// 在当前Edge使用方向规范化参数(0,1)处切分有限Topology_Edge，返回沿原Edge遍历方向排列的两条新Edge。
std::vector<Topology_Edge> splitEdge(const Topology_Edge& edge, double parameter);

// 在当前Edge使用方向的一组严格递增规范化参数处切分有限Topology_Edge，返回沿原Edge遍历方向排列的全部新Edge。
std::vector<Topology_Edge> splitEdge(const Topology_Edge& edge, const std::vector<double>& parameters);

}
}
}

#endif // MYBREP_OPERATION_SPLIT_EDGESPLIT_H
