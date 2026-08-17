#ifndef MYBREP_TOOL_QUERY_FACECLASSIFIER2D_H
#define MYBREP_TOOL_QUERY_FACECLASSIFIER2D_H

#include "MyMath/Vector2.h"
#include "MyBRep/Topology/Face/Topology_Face.h"

namespace MyBRep
{

// 描述Surface参数空间中的点相对于Face trimming区域的位置。
enum class FaceUVClassification
{
    Outside,
    Boundary,
    Inside
};

// 在单一非周期UV参数图中使用全部trimming Wire的even-odd规则分类指定UV点；无显式Wire时整个Surface自然参数域视为Face区域。
FaceUVClassification classifyFaceUV(const Topology_Face& face, const MyMath::Vector2& parameter,
                                    double tolerance = MyMath::Vector2::DefaultEpsilon);

}

#endif // MYBREP_TOOL_QUERY_FACECLASSIFIER2D_H