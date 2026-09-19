#ifndef MYBREP_MESH_CONICALFACEMESHER_H
#define MYBREP_MESH_CONICALFACEMESHER_H

#include "MyBRep/Mesh/FaceMesh.h"
#include "MyBRep/Mesh/ParametricFaceMesherCore.h"
#include "MyBRep/Topology/Face/Topology_Face.h"

namespace MyBRep
{

// 圆锥Surface Face三角化参数。
struct ConicalFaceMeshOptions : public ParametricFaceMeshOptions
{
    ConicalFaceMeshOptions();
};

// U周期圆锥Surface Face Mesher。
// 使用ParametricFaceMesherCore执行边界采样、周期展开、区域三角化和曲面细分，并由本类定义V=0顶点奇点语义。
// apex处保留不同U参数的同位置FaceMesh顶点，以保持各自不同的极限法向。
class ConicalFaceMesher
{
public:
    // 判断当前Face是否满足圆锥三角化的基础前置条件。
    static bool canMesh(const Topology_Face& face);

    // 三角化圆锥Face；前置条件、周期展开、奇点处理或曲面细分失败时返回空FaceMesh。
    static FaceMesh mesh(const Topology_Face& face, const ConicalFaceMeshOptions& options = ConicalFaceMeshOptions());
};

}

#endif // MYBREP_MESH_CONICALFACEMESHER_H
