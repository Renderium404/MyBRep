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
// v1支持普通面片、跨seam面片和完整2π参数带；触碰旋转轴的参数退化区域留给v2处理。
class RevolvedFaceMesher
{
public:
    static bool canMesh(const Topology_Face& face);
    static FaceMesh mesh(const Topology_Face& face, const RevolvedFaceMeshOptions& options = RevolvedFaceMeshOptions());
};

}

#endif // MYBREP_MESH_REVOLVEDFACEMESHER_H
