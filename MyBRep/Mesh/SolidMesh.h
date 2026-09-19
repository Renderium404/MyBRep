#ifndef MYBREP_MESH_SOLIDMESH_H
#define MYBREP_MESH_SOLIDMESH_H

#include <cstddef>
#include <vector>

#include "MyBRep/Mesh/FaceMesh.h"
#include "MyBRep/Topology/Solid/Topology_Solid.h"

namespace MyBRep
{

class SolidMesher;

// Solid离散后的分Face三角网格。
// 当前保持Topology_Solid原有Shell/Face层次，不合并不同Face的顶点和索引，便于后续显示、拾取及局部网格处理。
class SolidMesh
{
public:
    // 构造不包含任何Topology_Solid和FaceMesh的空网格。
    SolidMesh();

    /// 状态

    // 判断当前SolidMesh是否没有任何Shell网格。
    bool isEmpty() const;
    // 判断当前SolidMesh是否持有有效Topology_Solid，且每个Shell/Face均存在对应的有效FaceMesh。
    bool isValid() const;

    /// 拓扑来源

    // 返回当前网格对应的Topology_Solid。
    const Topology_Solid& topology() const;

    /// 网格层次

    // 返回Shell网格数量。
    std::size_t shellCount() const;
    // 返回指定Shell包含的FaceMesh数量。
    std::size_t faceCount(std::size_t shellIndex) const;
    // 返回全部Shell包含的FaceMesh总数量。
    std::size_t faceCount() const;
    // 返回指定Shell、Face位置对应的FaceMesh。
    const FaceMesh& faceMesh(std::size_t shellIndex, std::size_t faceIndex) const;

private:
    friend class SolidMesher;

    Topology_Solid m_topology;                         // 当前离散结果对应的完整B-Rep实体。
    std::vector<std::vector<FaceMesh> > m_shellMeshes; // 按Solid Shell/Face顺序保存的FaceMesh集合。
};

}

#endif // MYBREP_MESH_SOLIDMESH_H