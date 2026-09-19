#ifndef MYBREP_MESH_SPHERICALFACEMESHER_H
#define MYBREP_MESH_SPHERICALFACEMESHER_H

#include "MyBRep/Mesh/FaceMesh.h"
#include "MyBRep/Mesh/ParametricFaceMesherCore.h"
#include "MyBRep/Topology/Face/Topology_Face.h"

namespace MyBRep
{

// 球面Surface Face三角化参数。
struct SphericalFaceMeshOptions : public ParametricFaceMeshOptions
{
    SphericalFaceMeshOptions();
};

// U周期球面Surface Face Mesher。
// 使用ParametricFaceMesherCore执行边界采样、周期展开、区域三角化和曲面细分，并由本类定义南北极参数奇点语义。
// Core阶段允许不同U参数表示同一极点，最终输出阶段将同一极点合并为单个FaceMesh顶点。
class SphericalFaceMesher
{
public:
    // 判断当前Face是否满足球面三角化的基础前置条件。
    static bool canMesh(const Topology_Face& face);

    // 三角化球面Face；前置条件、周期展开、奇点处理或曲面细分失败时返回空FaceMesh。
    static FaceMesh mesh(const Topology_Face& face, const SphericalFaceMeshOptions& options = SphericalFaceMeshOptions());
};

}

#endif // MYBREP_MESH_SPHERICALFACEMESHER_H
