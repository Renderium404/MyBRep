#ifndef MYBREP_MESH_SPHERICALFACEMESHER_H
#define MYBREP_MESH_SPHERICALFACEMESHER_H

#include "MyBRep/Mesh/FaceMesh.h"
#include "MyBRep/Topology/Face/Topology_Face.h"

namespace MyBRep
{

// 球面Face边界离散、周期参数展开和曲面三角化参数。
struct SphericalFaceMeshOptions
{
    SphericalFaceMeshOptions();

    // 判断当前参数是否可用于球面Face三角化。
    bool isValid() const;

    double boundaryChordTolerance;       // trimming P-Curve映射到三维球面后允许的最大边界弦误差。
    double surfaceChordTolerance;        // 球面Surface被三角形近似时允许的最大共享边三维弦高误差。
    double geometricTolerance;           // UV点连接、合并、共线、相交、周期对齐和极点判定使用的参数空间几何容差。
    int minimumBoundarySubdivisionDepth; // 非线性或周期边界至少执行的二分深度。
    int maximumBoundarySubdivisionDepth; // trimming边界自适应细分允许的最大二分深度。
    int maximumSurfaceSubdivisionRounds; // 球面Surface共享边一致细分允许的最大迭代轮数。
};

// 将具有显式闭合trimming Wire的球面Topology_Face离散为三角网格。
// Face全部Wire继续按even-odd规则解释；U周期方向会展开到一个连续参数图中，南北极参数奇点会在最终三维网格中合并。
// 当前要求全部Wire能够共同落入跨度不超过一个U周期的连续展开图；允许trimming边界接触一个或两个V参数极点。
class SphericalFaceMesher
{
public:
    // 判断当前Face是否满足球面三角化的基础拓扑前置条件；允许Edge端点位于南北极。
    static bool canMesh(const Topology_Face& face);

    // 三角化球面Face；前置条件、周期展开或曲面细分失败时返回空FaceMesh。
    static FaceMesh mesh(const Topology_Face& face, const SphericalFaceMeshOptions& options = SphericalFaceMeshOptions());
};

}

#endif // MYBREP_MESH_SPHERICALFACEMESHER_H
