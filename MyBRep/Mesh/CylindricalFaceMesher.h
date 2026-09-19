#ifndef MYBREP_MESH_CYLINDRICALFACEMESHER_H
#define MYBREP_MESH_CYLINDRICALFACEMESHER_H

#include "MyBRep/Mesh/FaceMesh.h"
#include "MyBRep/Mesh/ParametricFaceMesherCore.h"
#include "MyBRep/Topology/Face/Topology_Face.h"

namespace MyBRep
{

// 圆柱Surface Face三角化参数。
struct CylindricalFaceMeshOptions : public ParametricFaceMeshOptions
{
    CylindricalFaceMeshOptions();
};

// U周期正则圆柱Surface Face Mesher。
// 使用ParametricFaceMesherCore执行边界采样、周期展开、区域三角化和曲面细分。
// 继续限制单个曲面参数边最多跨越90°，避免过大弦误差配置产生跨越圆柱背面的宽三角形。
class CylindricalFaceMesher
{
public:
    // 判断当前Face是否满足圆柱三角化的基础前置条件。
    static bool canMesh(const Topology_Face& face);

    // 三角化圆柱Face；前置条件、周期展开或曲面细分失败时返回空FaceMesh。
    static FaceMesh mesh(const Topology_Face& face, const CylindricalFaceMeshOptions& options = CylindricalFaceMeshOptions());
};

}

#endif // MYBREP_MESH_CYLINDRICALFACEMESHER_H