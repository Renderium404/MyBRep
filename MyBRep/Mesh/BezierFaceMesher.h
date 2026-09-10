#ifndef MYBREP_MESH_BEZIERFACEMESHER_H
#define MYBREP_MESH_BEZIERFACEMESHER_H

#include "MyBRep/Mesh/FaceMesh.h"
#include "MyBRep/Mesh/ParametricFaceMesherCore.h"
#include "MyBRep/Topology/Face/Topology_Face.h"

namespace MyBRep
{

// Bezier Surface Face三角化参数。
struct BezierFaceMeshOptions : public ParametricFaceMeshOptions
{
    BezierFaceMeshOptions();
};

// 有限、非周期、正则Bezier Surface Face Mesher。
// v1直接复用ParametricFaceMesherCore，参数奇点继续按未知奇点拒绝。
class BezierFaceMesher
{
public:
    static bool canMesh(const Topology_Face& face);
    static FaceMesh mesh(const Topology_Face& face, const BezierFaceMeshOptions& options = BezierFaceMeshOptions());
};

}

#endif // MYBREP_MESH_BEZIERFACEMESHER_H
