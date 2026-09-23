#ifndef MYBREPOPENGL_BUILDER_BREPSHAPEBUILDER_H
#define MYBREPOPENGL_BUILDER_BREPSHAPEBUILDER_H

#include <QString>

#include "MyBRep/Topology/Shape/Topology_Shape.h"

class BufferGeometry;

namespace MyBRep
{
namespace Display
{

// 控制Topology_Shape离散为MyOpenGL三角形Geometry时使用的精度参数。
// 当前只支持Geometry_Revolved。
struct BRepShapeBuildOptions
{
    BRepShapeBuildOptions();

    bool isValid() const;

    double profileChordTolerance;       // 二维母线曲线离散允许的最大弦误差。
    double revolutionChordTolerance;    // 绕Z轴圆周离散允许的最大弦高误差。
    int minimumProfileSubdivisionDepth; // 非直线母线至少执行的二分深度。
    int maximumProfileSubdivisionDepth; // 母线自适应细分允许的最大二分深度。
    int minimumRevolutionSegments;      // 完整2*pi旋转至少使用的圆周段数。
    int maximumRevolutionSegments;      // 完整2*pi旋转允许的最大圆周段数。
};

// 将Topology_Shape离散为局部坐标下包含Position和Normal的三角形Geometry。
// 当前第一阶段只接受ShapeKind::Revolved；Builder不处理Instance变换。
class BRepShapeBuilder
{
public:
    static bool canBuild(const Topology_Shape& shape);

    static BufferGeometry* build(const Topology_Shape& shape, const QString& name,
                                 const BRepShapeBuildOptions& options = BRepShapeBuildOptions());
};

}
}

#endif // MYBREPOPENGL_BUILDER_BREPSHAPEBUILDER_H
