#include "RevolvedModeling.h"

#include <limits>
#include <vector>

#include "MyBRep/Foundation/Diagnostic.h"
#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Construction/Geometry_Revolved.h"
#include "MyBRep/Geometry/Shape/Geometry_Shape.h"

namespace
{

// 判断标量是否为有限非负数。
bool isFiniteNonNegative(double value)
{
    const double infinity = (std::numeric_limits<double>::infinity)();

    return value == value && value != infinity && value != -infinity && value >= 0.0;
}

// 将有向Topology_Wire无损转换为Geometry_Revolved需要的有限母线段序列。
std::vector<MyBRep::Geometry_Revolved::ProfileSegment> profileSegments(const MyBRep::Topology_Wire& profile)
{
    MYBREP_ASSERT_MESSAGE(profile.isValid() && profile.isClosed() && profile.edgeCount() > 0, "Revolved modeling requires a valid non-empty closed Topology_Wire.");

    std::vector<MyBRep::Geometry_Revolved::ProfileSegment> segments;
    segments.reserve(profile.edgeCount());

    for (std::size_t index = 0; index < profile.edgeCount(); ++index)
    {
        const MyBRep::Topology_Edge edge = profile.edge(index);

        segments.push_back(MyBRep::Geometry_Revolved::ProfileSegment(edge.geometryResource(), edge.firstParameter(), edge.lastParameter()));
    }

    return segments;
}

}

namespace MyBRep
{
namespace Modeling
{

/// 局部Topology_Shape创建

Topology_Shape createRevolved(const Topology_Wire& profile, double profileTolerance)
{
    MYBREP_ASSERT_MESSAGE(profile.isValid() && profile.isClosed() && profile.edgeCount() > 0, "Revolved modeling requires a valid non-empty closed Topology_Wire.");
    MYBREP_ASSERT_MESSAGE(isFiniteNonNegative(profileTolerance), "Revolved modeling profile tolerance must be finite and non-negative.");

    const std::vector<Geometry_Revolved::ProfileSegment> segments = profileSegments(profile);

    const Foundation::RefPtr<const Geometry_Shape> geometry(new Geometry_Revolved(segments, profileTolerance));

    return Topology_Shape(geometry);
}

/// 空间Shape实例创建

Shape makeRevolved(const Topology_Wire& profile, double profileTolerance)
{
    return Shape(createRevolved(profile, profileTolerance));
}

Shape makeRevolved(const Topology_Wire& profile, const MyMath::Matrix4& localToWorld, double profileTolerance)
{
    return Shape(createRevolved(profile, profileTolerance), localToWorld);
}

Shape makeRevolved(const Wire& profile, double profileTolerance)
{
    MYBREP_ASSERT_MESSAGE(profile.isValid(), "Revolved modeling requires a valid Wire instance.");

    return Shape(createRevolved(profile.topology(), profileTolerance), profile.localToWorld());
}

}
}
