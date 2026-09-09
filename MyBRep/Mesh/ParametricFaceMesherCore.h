#ifndef MYBREP_MESH_PARAMETRICFACEMESHERCORE_H
#define MYBREP_MESH_PARAMETRICFACEMESHERCORE_H

#include "MyBRep/Mesh/FaceMesh.h"
#include "MyBRep/Topology/Face/Topology_Face.h"

namespace MyBRep
{

// 通用正则参数曲面Face三角化参数。
// 该结构只描述Mesher算法约束，不规定具体SurfaceKind。
struct ParametricFaceMeshOptions
{
    ParametricFaceMeshOptions();

    // 判断当前参数是否满足通用参数曲面三角化约束。
    bool isValid() const;

    double boundaryChordTolerance;       // trimming P-Curve映射到三维Surface后允许的最大边界弦误差。
    double surfaceChordTolerance;        // Surface被三角形近似时允许的最大共享边三维弦高误差。
    double geometricTolerance;           // UV连接、合并、共线、相交和周期对齐使用的参数空间几何容差。
    int minimumBoundarySubdivisionDepth; // trimming Edge至少执行的二分深度。
    int maximumBoundarySubdivisionDepth; // trimming Edge自适应细分允许的最大二分深度。
    int maximumSurfaceSubdivisionRounds; // Surface共享边一致细分允许的最大迭代轮数。
};

// 通用参数曲面Mesher使用的参数域规则。
struct ParametricFaceMeshPolicy
{
    ParametricFaceMeshPolicy();

    bool periodicU;                 // U方向是否按Surface周期展开。
    bool periodicV;                 // V方向是否按Surface周期展开。
    bool rejectSingularParameters;  // 是否拒绝dS/du × dS/dv退化的参数点。
};

// 为Extruded/Revolved等正则参数曲面提供共享三角化核心，支持U/V单周期和双周期参数面。
// 本类不判断SurfaceKind；具体Mesher必须先完成曲面类型和周期语义检查。
class ParametricFaceMesherCore
{
public:
    // 判断Face的Wire、P-Curve和参数域是否满足共享核心的基础前置条件。
    static bool canMesh(const Topology_Face& face, const ParametricFaceMeshPolicy& policy);

    // 执行P-Curve采样、UV even-odd三角化和三维共享边自适应细分。
    static FaceMesh mesh(const Topology_Face& face,
                         const ParametricFaceMeshOptions& options,
                         const ParametricFaceMeshPolicy& policy);
};

}

#endif // MYBREP_MESH_PARAMETRICFACEMESHERCORE_H
