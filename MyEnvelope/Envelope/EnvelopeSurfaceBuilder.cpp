#include "MyEnvelope/Envelope/EnvelopeSurfaceBuilder.h"

namespace MyEnvelope
{

void EnvelopeSurfaceBuilder::build(const MyBRep::Geometry_Revolved& body,
                                   const MotionSegment& motion,
                                   std::vector<RevolvedEnvelopeSurface>& result,
                                   double epsilon)
{
    result.clear();
    result.reserve(body.profileSegmentCount() * 2);

    for (std::size_t index = 0; index < body.profileSegmentCount(); ++index)
    {
        const MyBRep::Geometry_Revolved::ProfileSegment& segment = body.profileSegment(index);
        result.push_back(RevolvedEnvelopeSurface(segment, body.radialSign(), motion, RevolvedEnvelopeBranch::Plus, epsilon));
        result.push_back(RevolvedEnvelopeSurface(segment, body.radialSign(), motion, RevolvedEnvelopeBranch::Minus, epsilon));
    }
}

}
