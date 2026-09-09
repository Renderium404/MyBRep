#ifndef MYBREPOPENGL_BUILDER_BREPFACEBUILDER_H
#define MYBREPOPENGL_BUILDER_BREPFACEBUILDER_H

#include <QString>

#include "MyMath/Matrix4.h"
#include "MyBRep/Mesh/FaceMesher.h"
#include "MyBRep/Topology/Face/Topology_Face.h"

class BufferGeometry;

namespace MyBRep
{
namespace Display
{

// 控制Topology_Face离散为MyOpenGL三角形Geometry时使用的Face网格参数。
struct BRepFaceBuildOptions
{
    BRepFaceBuildOptions();

    // 判断当前Face显示构建参数是否有效。
    bool isValid() const;

    PlanarFaceMeshOptions meshing;                   // 既有平面Face三角化参数，保留原字段名以兼容现有调用代码。
    CylindricalFaceMeshOptions cylindricalMeshing;   // 圆柱Face三角化参数。
    SphericalFaceMeshOptions sphericalMeshing;       // 球面Face三角化参数。
    ConicalFaceMeshOptions conicalMeshing;           // 圆锥Face三角化参数。
    ExtrudedFaceMeshOptions extrudedMeshing;         // 拉伸Face三角化参数。
    RevolvedFaceMeshOptions revolvedMeshing;         // 旋转Face三角化参数。
};

// 将MyBRep Topology_Face转换为包含Position和Normal的MyOpenGL三角形BufferGeometry。
// 具体Surface离散由MyBRep::FaceMesher统一分派；localToWorld直接烘焙进顶点和法向，因此支持一般可逆仿射放置。
class BRepFaceBuilder
{
public:
    // 使用单位放置构建Face表面Geometry，当前不支持的Surface类型返回空指针。
    static BufferGeometry* build(const Topology_Face& face, const QString& name, const BRepFaceBuildOptions& options = BRepFaceBuildOptions());

    // 使用指定可逆仿射放置构建Face表面Geometry，不支持的Surface类型或非法放置返回空指针。
    static BufferGeometry* build(const Topology_Face& face, const MyMath::Matrix4& localToWorld, const QString& name, const BRepFaceBuildOptions& options = BRepFaceBuildOptions());
};

}
}

#endif // MYBREPOPENGL_BUILDER_BREPFACEBUILDER_H
