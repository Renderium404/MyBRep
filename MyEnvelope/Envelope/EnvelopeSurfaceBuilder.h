#ifndef MYENVELOPE_ENVELOPE_ENVELOPESURFACEBUILDER_H
#define MYENVELOPE_ENVELOPE_ENVELOPESURFACEBUILDER_H

#include <vector>

#include "MyBRep/Geometry/Construction/Geometry_Revolved.h"
#include "MyEnvelope/Geometry/RevolvedEnvelopeSurface.h"
#include "MyEnvelope/Motion/MotionSegment.h"

namespace MyEnvelope
{

// 构造回转体连续运动产生的候选包络几何分支，不负责Face裁剪、Shell连接或Solid构造。
class EnvelopeSurfaceBuilder
{
public:
    // 为Geometry_Revolved每个有限母线段生成Plus/Minus两条候选包络分支。
    // 候选分支并不保证在整个[0,1]x[0,1]参数域内都有定义，调用者需使用isDefinedAt/isRegularAt裁剪有效区域。
    static void build(const MyBRep::Geometry_Revolved& body,
                      const MotionSegment& motion,
                      std::vector<RevolvedEnvelopeSurface>& result,
                      double epsilon = MyMath::Vector3::DefaultEpsilon);
};

}

#endif // MYENVELOPE_ENVELOPE_ENVELOPESURFACEBUILDER_H
