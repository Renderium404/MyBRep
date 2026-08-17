#ifndef MYBREP_OPERATION_SPLIT_FACESPLIT_H
#define MYBREP_OPERATION_SPLIT_FACESPLIT_H

#include <cstddef>
#include <vector>

#include "MyBRep/Operation/Split/WireSplit.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Face/Topology_Face.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

namespace MyBRep
{
namespace Operation
{
namespace Split
{

/// Face裁剪Wire替换

// 使用新的闭合Wire替换Face中指定裁剪Wire，返回共享原Geometry_Surface并保持输入Face当前使用方向的新Topology_Face。
Topology_Face replaceFaceWire(const Topology_Face& face, std::size_t wireIndex, const Topology_Wire& replacementWire);

/// Face边界Edge-use切分

// 在指定裁剪Wire的指定Edge-use当前方向规范化参数(0,1)处切分边界Edge，并重建保持同一几何区域的新Topology_Face。
Topology_Face splitFaceBoundaryEdge(const Topology_Face& face, std::size_t wireIndex, std::size_t edgeIndex, double parameter);

// 在指定裁剪Wire的指定Edge-use当前方向的一组严格递增规范化参数处切分边界Edge，并重建保持同一几何区域的新Topology_Face。
Topology_Face splitFaceBoundaryEdge(const Topology_Face& face, std::size_t wireIndex, std::size_t edgeIndex, const std::vector<double>& parameters);

/// Face区域切分

// 使用连接单一裁剪Wire两个既有Vertex的内部splitting Edge把Face切分为两个新Face。
std::vector<Topology_Face> splitFaceByEdge(const Topology_Face& face, const Topology_Edge& splittingEdge);

// 允许splitting Edge起终点分别落在单一裁剪Wire指定Edge-use的内部或端点；操作自动建立共享边界Vertex并重建splitting Edge拓扑端点。
//
// firstLocation对应splittingEdge当前使用方向起点，secondLocation对应其终点；两位置均按face当前使用方向下的Wire/Edge-use解释。
// 当前第一阶段要求Face只有一个裁剪Wire，splitting Edge必须已经具有当前Face Surface上的Curve-on-Surface表示，且其内部几何合法性由调用者保证。
std::vector<Topology_Face> splitFaceByEdge(const Topology_Face& face, const WireSplitLocation& firstLocation,
                                           const WireSplitLocation& secondLocation, const Topology_Edge& splittingEdge,
                                           double tolerance = MyMath::Vector3::DefaultEpsilon);

}
}
}

#endif // MYBREP_OPERATION_SPLIT_FACESPLIT_H
