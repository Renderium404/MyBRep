#ifndef MYBREPOPENGL_BUILDER_BREPEDGEBUILDER_H
#define MYBREPOPENGL_BUILDER_BREPEDGEBUILDER_H

#include <QString>

#include "MyBRep/Topology/Edge/Topology_Edge.h"

class BufferGeometry;

namespace MyBRep
{
namespace Display
{

// 控制Topology_Edge离散为线段Geometry时使用的精度参数。
struct BRepEdgeBuildOptions
{
    BRepEdgeBuildOptions();

    bool isValid() const;

    double chordTolerance;
    int minimumSubdivisionDepth;
    int maximumSubdivisionDepth;
};

// 将Topology_Edge离散为局部坐标下的Lines Geometry。
// Builder不负责Wire/Face/Shell/Solid遍历，也不处理Instance变换。
class BRepEdgeBuilder
{
public:
    static BufferGeometry* build(const Topology_Edge& edge, const QString& name,
                                 const BRepEdgeBuildOptions& options = BRepEdgeBuildOptions());
};

}
}

#endif // MYBREPOPENGL_BUILDER_BREPEDGEBUILDER_H