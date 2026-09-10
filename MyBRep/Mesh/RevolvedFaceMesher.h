#ifndef MYBREP_MESH_REVOLVEDFACEMESHER_H
#define MYBREP_MESH_REVOLVEDFACEMESHER_H

#include "MyBRep/Mesh/FaceMesh.h"
#include "MyBRep/Mesh/ParametricFaceMesherCore.h"
#include "MyBRep/Topology/Face/Topology_Face.h"

namespace MyBRep
{

// 旋转Surface Face三角化参数。
struct RevolvedFaceMeshOptions : public ParametricFaceMeshOptions
{
    RevolvedFaceMeshOptions();
};

// U周期旋转Surface Face Mesher。
// v2支持普通面片、U/V周期、完整周期带以及母线以一阶非退化方式触碰旋转轴的边界参数奇点。
// 轴奇点处保留不同U参数的同位置FaceMesh顶点，并使用母线一阶切向解析计算极限法向。
class RevolvedFaceMesher
{
public:
    static bool canMesh(const Topology_Face& face);
    static FaceMesh mesh(const Topology_Face& face, const RevolvedFaceMeshOptions& options = RevolvedFaceMeshOptions());
};

}

#endif // MYBREP_MESH_REVOLVEDFACEMESHER_H
