#ifndef MYBREPOPENGL_BUILDER_BREPWIREFRAMEBUILDER_H
#define MYBREPOPENGL_BUILDER_BREPWIREFRAMEBUILDER_H

#include <QString>

#include "MyMath/Matrix4.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Face/Topology_Face.h"
#include "MyBRep/Topology/Shell/Topology_Shell.h"
#include "MyBRep/Topology/Solid/Topology_Solid.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

class BufferGeometry;

namespace MyBRep
{
namespace Display
{

// 控制B-Rep拓扑边界离散为MyOpenGL线段Geometry时的精度和显示宽度。
struct BRepWireframeBuildOptions
{
    BRepWireframeBuildOptions();

    bool isValid() const;

    double chordTolerance;       // 世界空间弦误差容差。
    int minimumSubdivisionDepth; // 非直线Edge至少执行的二分深度。
    int maximumSubdivisionDepth; // 自适应细分允许的最大二分深度。
    float lineWidth;             // MyOpenGL Lines的屏幕Pixel宽度。
};

// 将MyBRep拓扑边界离散为MyOpenGL BufferGeometry。
// localToWorld会直接烘焙进生成顶点，因此支持MyBRep的一般可逆仿射放置，不依赖MyOpenGL的TRS Transform。
class BRepWireframeBuilder
{
public:
    /// Edge
    static BufferGeometry* build(const Topology_Edge& edge, const QString& name, const BRepWireframeBuildOptions& options = BRepWireframeBuildOptions());
    static BufferGeometry* build(const Topology_Edge& edge, const MyMath::Matrix4& localToWorld, const QString& name, const BRepWireframeBuildOptions& options = BRepWireframeBuildOptions());

    /// Wire
    static BufferGeometry* build(const Topology_Wire& wire, const QString& name, const BRepWireframeBuildOptions& options = BRepWireframeBuildOptions());
    static BufferGeometry* build(const Topology_Wire& wire, const MyMath::Matrix4& localToWorld, const QString& name, const BRepWireframeBuildOptions& options = BRepWireframeBuildOptions());

    /// Face Boundary
    static BufferGeometry* build(const Topology_Face& face, const QString& name, const BRepWireframeBuildOptions& options = BRepWireframeBuildOptions());
    static BufferGeometry* build(const Topology_Face& face, const MyMath::Matrix4& localToWorld, const QString& name, const BRepWireframeBuildOptions& options = BRepWireframeBuildOptions());

    /// Shell Boundary
    static BufferGeometry* build(const Topology_Shell& shell, const QString& name, const BRepWireframeBuildOptions& options = BRepWireframeBuildOptions());
    static BufferGeometry* build(const Topology_Shell& shell, const MyMath::Matrix4& localToWorld, const QString& name, const BRepWireframeBuildOptions& options = BRepWireframeBuildOptions());

    /// Solid Boundary
    static BufferGeometry* build(const Topology_Solid& solid, const QString& name, const BRepWireframeBuildOptions& options = BRepWireframeBuildOptions());
    static BufferGeometry* build(const Topology_Solid& solid, const MyMath::Matrix4& localToWorld, const QString& name, const BRepWireframeBuildOptions& options = BRepWireframeBuildOptions());
};

}
}

#endif // MYBREPOPENGL_BUILDER_BREPWIREFRAMEBUILDER_H
