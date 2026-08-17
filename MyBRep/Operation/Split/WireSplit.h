#ifndef MYBREP_OPERATION_SPLIT_WIRESPLIT_H
#define MYBREP_OPERATION_SPLIT_WIRESPLIT_H

#include <cstddef>
#include <vector>

#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

namespace MyBRep
{
namespace Operation
{
namespace Split
{

// 描述当前Wire某个Edge-use规范化参数[0,1]上的边界位置。
struct WireSplitLocation
{
    WireSplitLocation(std::size_t sourceEdgeIndex, double sourceParameter)
        : edgeIndex(sourceEdgeIndex), parameter(sourceParameter)
    {
    }

    std::size_t edgeIndex; // 当前Wire使用方向下的Edge-use索引。
    double parameter; // 当前Edge-use使用方向下的规范化参数。
};

// 保存闭合Wire在两个指定边界位置完成必要Edge细分后的拓扑结果。
struct WireBoundarySplit
{
    Topology_Wire wire; // 保持原Wire当前遍历方向的细分结果。
    Topology_Vertex firstVertex; // 第一输入位置对应的共享边界Vertex。
    Topology_Vertex secondVertex; // 第二输入位置对应的共享边界Vertex。
};

/// Wire Edge-use替换

// 使用一组按遍历方向连续连接的新Edge-use替换Wire中指定Edge-use，返回保持输入Wire当前遍历方向的新Topology_Wire。
Topology_Wire replaceWireEdge(const Topology_Wire& wire, std::size_t edgeIndex, const std::vector<Topology_Edge>& replacementEdges);

/// Wire Edge-use切分

// 在指定Edge-use当前方向规范化参数(0,1)处切分，并返回保持输入Wire当前遍历方向的新Topology_Wire。
Topology_Wire splitWireEdge(const Topology_Wire& wire, std::size_t edgeIndex, double parameter);

// 在指定Edge-use当前方向的一组严格递增规范化参数处切分，并返回保持输入Wire当前遍历方向的新Topology_Wire。
Topology_Wire splitWireEdge(const Topology_Wire& wire, std::size_t edgeIndex, const std::vector<double>& parameters);

// 在闭合Wire的两个边界位置建立真实共享Vertex；位置可落在既有Vertex或Edge内部。
WireBoundarySplit splitClosedWireBoundary(const Topology_Wire& wire, const WireSplitLocation& firstLocation, const WireSplitLocation& secondLocation);

/// 闭合Wire区域切分

// 使用连接两个既有边界Vertex的内部splitting Edge切分单个闭合Wire，返回沿原Wire方向构造的两个闭合Topology_Wire。
std::vector<Topology_Wire> splitClosedWireByEdge(const Topology_Wire& wire, const Topology_Edge& splittingEdge);

}
}
}

#endif // MYBREP_OPERATION_SPLIT_WIRESPLIT_H
