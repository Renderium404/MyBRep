#ifndef MYBREPOPENGL_BUILDER_BREPSOLIDBUILDER_H
#define MYBREPOPENGL_BUILDER_BREPSOLIDBUILDER_H

#include <QString>

#include "MyMath/Matrix4.h"
#include "MyBRep/Topology/Solid/Topology_Solid.h"
#include "MyBRepOpenGL/Builder/BRepFaceBuilder.h"
#include "MyBRepOpenGL/Builder/BRepWireframeBuilder.h"

class BufferGeometry;

namespace MyBRep
{
namespace Display
{

// 控制Topology_Solid转换为MyOpenGL表面和边界Geometry时使用的离散参数。
struct BRepSolidBuildOptions
{
    BRepSolidBuildOptions();

    // 判断当前Solid显示构建参数是否有效。
    bool isValid() const;

    BRepFaceBuildOptions surface;         // Solid全部Shell中各Face的三角网格构建参数。
    BRepWireframeBuildOptions wireframe; // Solid全部唯一Edge的边界离散参数。
};

// 将完整B-Rep Topology_Solid转换为MyOpenGL显示Geometry。
// Surface按Shell/Face独立离散后合并，不跨Face焊接顶点；Boundary按共享Topology_Edge身份全局去重。
class BRepSolidBuilder
{
public:
    /// Surface

    // 使用单位放置构建Solid统一三角表面Geometry；任一Face不支持时返回空指针。
    static BufferGeometry* buildSurface(const Topology_Solid& solid, const QString& name,
                                        const BRepSolidBuildOptions& options = BRepSolidBuildOptions());

    // 使用指定可逆仿射放置构建Solid统一三角表面Geometry；任一Face不支持或放置非法时返回空指针。
    static BufferGeometry* buildSurface(const Topology_Solid& solid, const MyMath::Matrix4& localToWorld, const QString& name,
                                        const BRepSolidBuildOptions& options = BRepSolidBuildOptions());

    /// Boundary

    // 使用单位放置构建Solid全局去重边界Geometry。
    static BufferGeometry* buildBoundary(const Topology_Solid& solid, const QString& name,
                                         const BRepSolidBuildOptions& options = BRepSolidBuildOptions());

    // 使用指定可逆仿射放置构建Solid全局去重边界Geometry。
    static BufferGeometry* buildBoundary(const Topology_Solid& solid, const MyMath::Matrix4& localToWorld, const QString& name,
                                         const BRepSolidBuildOptions& options = BRepSolidBuildOptions());
};

}
}

#endif // MYBREPOPENGL_BUILDER_BREPSOLIDBUILDER_H
