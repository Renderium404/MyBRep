#ifndef MYBREP_MESH_CONICALFACEMESHER_H
#define MYBREP_MESH_CONICALFACEMESHER_H

#include "MyBRep/Mesh/FaceMesh.h"
#include "MyBRep/Topology/Face/Topology_Face.h"

namespace MyBRep
{

// 圆锥Face边界离散、周期参数展开和曲面三角化参数。
struct ConicalFaceMeshOptions
{
    ConicalFaceMeshOptions();

    // 判断当前参数是否可用于圆锥Face三角化。
    bool isValid() const;

    double boundaryChordTolerance;       // trimming P-Curve映射到三维圆锥面后允许的最大边界弦误差。
    double surfaceChordTolerance;        // 圆锥Surface被三角形近似时允许的最大共享边三维弦高误差。
    double geometricTolerance;           // UV点连接、合并、共线、相交、周期对齐和圆锥顶点判定使用的参数空间几何容差。
    int minimumBoundarySubdivisionDepth; // 非线性或周期边界至少执行的二分深度。
    int maximumBoundarySubdivisionDepth; // trimming边界自适应细分允许的最大二分深度。
    int maximumSurfaceSubdivisionRounds; // 圆锥Surface共享边一致细分允许的最大迭代轮数。
};

// 将具有显式闭合trimming Wire的圆锥Topology_Face离散为三角网格，支持trimming边界触碰V=0圆锥顶点。
// Face全部Wire继续按even-odd规则解释；U周期方向会展开到一个连续参数图中。
// 当前要求全部Wire能够共同落入跨度不超过一个U周期的连续展开图；apex处允许多个同位置FaceMesh顶点保留各自U方向的极限法向。
class ConicalFaceMesher
{
public:
    // 判断当前Face是否满足圆锥三角化的基础拓扑前置条件；允许合法trimming边界端点位于V=0圆锥顶点。
    static bool canMesh(const Topology_Face& face);

    // 三角化圆锥Face；前置条件、周期展开或曲面细分失败时返回空FaceMesh。
    static FaceMesh mesh(const Topology_Face& face, const ConicalFaceMeshOptions& options = ConicalFaceMeshOptions());
};

}

#endif // MYBREP_MESH_CONICALFACEMESHER_H
