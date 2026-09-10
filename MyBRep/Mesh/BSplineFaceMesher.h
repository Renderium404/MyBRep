#ifndef MYBREP_MESH_BSPLINEFACEMESHER_H
#define MYBREP_MESH_BSPLINEFACEMESHER_H

#include "MyBRep/Mesh/FaceMesh.h"
#include "MyBRep/Mesh/ParametricFaceMesherCore.h"
#include "MyBRep/Topology/Face/Topology_Face.h"

namespace MyBRep
{

// B-Spline Surface Face三角化参数。
struct BSplineFaceMeshOptions : public ParametricFaceMeshOptions
{
    BSplineFaceMeshOptions();
};

// 有限、非周期、正则B-Spline Surface Face Mesher。
// v1支持一阶导数唯一的参数区域；未知参数奇点继续拒绝。
class BSplineFaceMesher
{
public:
    static bool canMesh(const Topology_Face& face);
    static FaceMesh mesh(const Topology_Face& face, const BSplineFaceMeshOptions& options = BSplineFaceMeshOptions());
};

}

#endif // MYBREP_MESH_BSPLINEFACEMESHER_H