#ifndef MYBREPOPENGL_BUILDER_BREPSHELLBUILDER_H
#define MYBREPOPENGL_BUILDER_BREPSHELLBUILDER_H

#include <QString>

#include "MyMath/Matrix4.h"
#include "MyBRep/Topology/Shell/Topology_Shell.h"
#include "MyBRepOpenGL/Builder/BRepFaceBuilder.h"
#include "MyBRepOpenGL/Builder/BRepWireframeBuilder.h"

class BufferGeometry;

namespace MyBRep
{
namespace Display
{

// 控制Topology_Shell转换为MyOpenGL表面和边界Geometry时使用的离散参数。
struct BRepShellBuildOptions
{
    BRepShellBuildOptions();

    // 判断当前Shell显示构建参数是否有效。
    bool isValid() const;

    BRepFaceBuildOptions surface;         // Shell中各Face的三角网格构建参数。
    BRepWireframeBuildOptions wireframe; // Shell全部唯一Edge的边界离散参数。
};

// 将Topology_Shell转换为MyOpenGL显示Geometry。
// Surface按Face独立离散后合并，不跨Face焊接顶点；Boundary按共享Topology_Edge身份去重。
class BRepShellBuilder
{
public:
    /// Surface

    // 使用单位放置构建Shell统一三角表面Geometry；任一Face不支持时返回空指针。
    static BufferGeometry* buildSurface(const Topology_Shell& shell, const QString& name,
                                        const BRepShellBuildOptions& options = BRepShellBuildOptions());

    // 使用指定可逆仿射放置构建Shell统一三角表面Geometry；任一Face不支持或放置非法时返回空指针。
    static BufferGeometry* buildSurface(const Topology_Shell& shell, const MyMath::Matrix4& localToWorld, const QString& name,
                                        const BRepShellBuildOptions& options = BRepShellBuildOptions());

    /// Boundary

    // 使用单位放置构建Shell去重边界Geometry。
    static BufferGeometry* buildBoundary(const Topology_Shell& shell, const QString& name,
                                         const BRepShellBuildOptions& options = BRepShellBuildOptions());

    // 使用指定可逆仿射放置构建Shell去重边界Geometry。
    static BufferGeometry* buildBoundary(const Topology_Shell& shell, const MyMath::Matrix4& localToWorld, const QString& name,
                                         const BRepShellBuildOptions& options = BRepShellBuildOptions());
};

}
}

#endif // MYBREPOPENGL_BUILDER_BREPSHELLBUILDER_H
