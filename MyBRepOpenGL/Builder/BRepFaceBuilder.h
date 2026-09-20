#ifndef MYBREPOPENGL_BUILDER_BREPFACEBUILDER_H
#define MYBREPOPENGL_BUILDER_BREPFACEBUILDER_H

#include <QString>

#include "MyBRep/Mesh/FaceMesher.h"
#include "MyBRep/Topology/Face/Topology_Face.h"

class BufferGeometry;

namespace MyBRep
{
namespace Display
{

// 控制Topology_Face离散为MyOpenGL三角形Geometry时使用的网格参数。
struct BRepFaceBuildOptions
{
    BRepFaceBuildOptions();

    bool isValid() const;

    PlanarFaceMeshOptions meshing;
    CylindricalFaceMeshOptions cylindricalMeshing;
    SphericalFaceMeshOptions sphericalMeshing;
    ConicalFaceMeshOptions conicalMeshing;
    ExtrudedFaceMeshOptions extrudedMeshing;
    RevolvedFaceMeshOptions revolvedMeshing;
    BezierFaceMeshOptions bezierMeshing;
    BSplineFaceMeshOptions bsplineMeshing;
};

// 将Topology_Face离散为局部坐标下包含Position和Normal的三角形Geometry。
// Builder不处理Instance变换，生成的Geometry始终保持Topology局部坐标。
class BRepFaceBuilder
{
public:
    static BufferGeometry* build(const Topology_Face& face, const QString& name,
                                 const BRepFaceBuildOptions& options = BRepFaceBuildOptions());
};

}
}

#endif // MYBREPOPENGL_BUILDER_BREPFACEBUILDER_H
