#ifndef MYBREP_MESH_FACEMESHER_H
#define MYBREP_MESH_FACEMESHER_H

#include "MyBRep/Mesh/ConicalFaceMesher.h"
#include "MyBRep/Mesh/CylindricalFaceMesher.h"
#include "MyBRep/Mesh/ExtrudedFaceMesher.h"
#include "MyBRep/Mesh/FaceMesh.h"
#include "MyBRep/Mesh/PlanarFaceMesher.h"
#include "MyBRep/Mesh/RevolvedFaceMesher.h"
#include "MyBRep/Mesh/SphericalFaceMesher.h"
#include "MyBRep/Topology/Face/Topology_Face.h"

namespace MyBRep
{

// Face统一三角化参数。
// 各Surface类型保留自己的离散参数，避免把不同参数度量、曲率、周期和奇点控制强行合并为一组公共字段。
struct FaceMeshOptions
{
    FaceMeshOptions();

    // 判断当前全部已支持Face Mesher参数是否有效。
    bool isValid() const;

    PlanarFaceMeshOptions planar;             // 平面Face三角化参数。
    CylindricalFaceMeshOptions cylindrical;   // 圆柱Face三角化参数。
    SphericalFaceMeshOptions spherical;       // 球面Face三角化参数。
    ConicalFaceMeshOptions conical;           // 圆锥Face三角化参数。
    ExtrudedFaceMeshOptions extruded;         // 拉伸Face三角化参数。
    RevolvedFaceMeshOptions revolved;         // 旋转Face三角化参数。
};

// Topology_Face统一三角化入口。
// 上层只依赖FaceMesher，由本类根据SurfaceKind分派到具体Surface Mesher。
class FaceMesher
{
public:
    // 判断当前Face是否存在已支持且满足前置条件的具体Mesher。
    static bool canMesh(const Topology_Face& face);

    // 根据SurfaceKind三角化Face；当前不支持的Surface类型返回空FaceMesh。
    static FaceMesh mesh(const Topology_Face& face, const FaceMeshOptions& options = FaceMeshOptions());
};

}

#endif // MYBREP_MESH_FACEMESHER_H
