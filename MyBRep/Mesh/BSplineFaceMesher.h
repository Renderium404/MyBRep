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

// 有限、非周期B-Spline Surface Face Mesher。
// 一次×一次且trimming为完整自然参数域矩形时按节点Span独立三角化，支持C0内部节点并保留节点线两侧独立法向。
// 其余B-Spline Face继续使用通用参数曲面Mesher，当前要求实际网格采样点处一阶导数唯一。
class BSplineFaceMesher
{
public:
    static bool canMesh(const Topology_Face& face);
    static FaceMesh mesh(const Topology_Face& face, const BSplineFaceMeshOptions& options = BSplineFaceMeshOptions());
};

}

#endif // MYBREP_MESH_BSPLINEFACEMESHER_H