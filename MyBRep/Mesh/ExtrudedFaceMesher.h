#ifndef MYBREP_MESH_EXTRUDEDFACEMESHER_H
#define MYBREP_MESH_EXTRUDEDFACEMESHER_H

#include "MyBRep/Mesh/FaceMesh.h"
#include "MyBRep/Mesh/ParametricFaceMesherCore.h"
#include "MyBRep/Topology/Face/Topology_Face.h"

namespace MyBRep
{

// 拉伸Surface Face三角化参数。
struct ExtrudedFaceMeshOptions : public ParametricFaceMeshOptions
{
    ExtrudedFaceMeshOptions();
};

// 非周期正则拉伸Surface Face Mesher。
// v1要求U/V均非周期且整个trimming区域不存在参数退化。
class ExtrudedFaceMesher
{
public:
    static bool canMesh(const Topology_Face& face);
    static FaceMesh mesh(const Topology_Face& face, const ExtrudedFaceMeshOptions& options = ExtrudedFaceMeshOptions());
};

}

#endif // MYBREP_MESH_EXTRUDEDFACEMESHER_H
