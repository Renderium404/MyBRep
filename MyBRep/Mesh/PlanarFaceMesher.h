#ifndef MYBREP_MESH_PLANARFACEMESHER_H
#define MYBREP_MESH_PLANARFACEMESHER_H

#include "MyBRep/Mesh/FaceMesh.h"
#include "MyBRep/Topology/Face/Topology_Face.h"

namespace MyBRep
{

// 平面Face边界离散与三角化参数。
struct PlanarFaceMeshOptions
{
    PlanarFaceMeshOptions();

    // 判断当前参数是否可用于平面Face三角化。
    bool isValid() const;

    double chordTolerance;       // P-Curve离散到参数空间折线时允许的最大弦误差。
    double geometricTolerance;   // UV点合并、共线和相交判断使用的二维几何容差。
    int minimumSubdivisionDepth; // 非直线P-Curve至少执行的二分深度。
    int maximumSubdivisionDepth; // P-Curve自适应细分允许的最大二分深度。
};

// 将具有显式闭合trimming Wire的平面Topology_Face离散为三角网格。
// Face的全部Wire按even-odd规则解释，Wire方向不决定外环或孔洞语义。
class PlanarFaceMesher
{
public:
    // 判断当前Face是否满足平面三角化的基础前置条件。
    static bool canMesh(const Topology_Face& face);

    // 三角化平面Face；前置条件或三角化过程失败时返回空FaceMesh。
    static FaceMesh mesh(const Topology_Face& face, const PlanarFaceMeshOptions& options = PlanarFaceMeshOptions());
};

}

#endif // MYBREP_MESH_PLANARFACEMESHER_H
