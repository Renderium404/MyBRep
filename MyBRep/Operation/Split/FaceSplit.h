#ifndef MYBREP_OPERATION_SPLIT_FACESPLIT_H
#define MYBREP_OPERATION_SPLIT_FACESPLIT_H

#include <cstddef>
#include <vector>

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

}
}
}

#endif // MYBREP_OPERATION_SPLIT_FACESPLIT_H