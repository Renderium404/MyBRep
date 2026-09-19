#ifndef MYBREP_MESH_SOLIDMESHER_H
#define MYBREP_MESH_SOLIDMESHER_H

#include "MyBRep/Mesh/FaceMesher.h"
#include "MyBRep/Mesh/SolidMesh.h"
#include "MyBRep/Topology/Solid/Topology_Solid.h"

namespace MyBRep
{

// Topology_Solid统一三角化入口。
// 当前按Solid原有Shell/Face层次逐Face调用FaceMesher，不在本层执行跨Face顶点焊接或共享Edge离散。
class SolidMesher
{
public:
    // 判断Solid中的全部Face是否均存在已支持且满足前置条件的Face Mesher。
    static bool canMesh(const Topology_Solid& solid);

    // 按Shell/Face顺序离散整个Solid；任意Face网格化失败时返回空SolidMesh。
    static SolidMesh mesh(const Topology_Solid& solid, const FaceMeshOptions& options = FaceMeshOptions());
};

}

#endif // MYBREP_MESH_SOLIDMESHER_H
