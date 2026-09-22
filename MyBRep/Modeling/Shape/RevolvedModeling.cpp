#include "RevolvedModeling.h"

#include <cmath>
#include <limits>
#include <vector>

#include "MyBRep/Foundation/Diagnostic.h"
#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Construction/Geometry_Revolved.h"
#include "MyBRep/Geometry/Curve/Geometry_Circle.h"
#include "MyBRep/Geometry/Curve/Geometry_Curve.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Circle2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Curve2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Line2D.h"
#include "MyBRep/Geometry/Shape/Geometry_Shape.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795;
const double TwoPi = Pi * 2.0;

// 判断标量是否为有限非负数。
bool isFiniteNonNegative(double value)
{
    const double infinity = (std::numeric_limits<double>::infinity)();
    return value == value && value != infinity && value != -infinity && value >= 0.0;
}

// 将允许profileTolerance平面误差的三维点转换为严格二维母线点。
MyMath::Vector2 profilePoint(const MyMath::Vector3& point, double profileTolerance)
{
    MYBREP_ASSERT_MESSAGE(point.isFinite(), "Revolved modeling profile points must be finite.");
    MYBREP_ASSERT_MESSAGE(std::fabs(point.z()) <= profileTolerance,
                          "Revolved modeling profile points must lie in the local XY plane.");
    return MyMath::Vector2(point.x(), point.y());
}

// 将三维Line Edge按当前有向起终点重建为二维单位参数直线段。
MyBRep::Geometry_Revolved::ProfileSegment createLineProfileSegment(const MyBRep::Topology_Edge& edge, double profileTolerance)
{
    const MyMath::Vector2 start = profilePoint(edge.startVertex().point(), profileTolerance);
    const MyMath::Vector2 end = profilePoint(edge.endVertex().point(), profileTolerance);
    const MyMath::Vector2 direction = end - start;

    MYBREP_ASSERT_MESSAGE(direction.isVector(0.0), "Revolved modeling profile Line Edge must be non-degenerate.");

    const double length = direction.length();
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> curve(new MyBRep::Geometry_Line2D(start, direction));
    return MyBRep::Geometry_Revolved::ProfileSegment(curve, 0.0, length);
}

// 将局部XY平面三维Circle Edge无损转换为具有相同角参数方向的二维Circle区间。
MyBRep::Geometry_Revolved::ProfileSegment createCircleProfileSegment(const MyBRep::Topology_Edge& edge, double profileTolerance)
{
    const MyBRep::Geometry_Circle& circle = static_cast<const MyBRep::Geometry_Circle&>(edge.geometry());
    const MyMath::Vector3 normal = circle.normal();
    const double epsilon = MyMath::Vector3::DefaultEpsilon;
    const double sweep = std::fabs(edge.lastParameter() - edge.firstParameter());

    profilePoint(edge.startVertex().point(), profileTolerance);
    profilePoint(edge.endVertex().point(), profileTolerance);

    MYBREP_ASSERT_MESSAGE(std::fabs(circle.center().z()) <= profileTolerance,
                          "Revolved modeling profile Circle center must lie in the local XY plane.");
    MYBREP_ASSERT_MESSAGE(std::fabs(normal.x()) <= epsilon && std::fabs(normal.y()) <= epsilon &&
                          std::fabs(std::fabs(normal.z()) - 1.0) <= epsilon,
                          "Revolved modeling profile Circle must lie in the local XY plane.");
    MYBREP_ASSERT_MESSAGE(sweep <= TwoPi + epsilon,
                          "Revolved modeling profile Circle interval must not exceed one complete period.");

    const MyMath::Vector2 center(circle.center().x(), circle.center().y());
    const MyMath::Vector2 xDir(circle.xDir().x(), circle.xDir().y());
    const MyMath::Vector2 yDir(circle.yDir().x(), circle.yDir().y());

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> curve(
        new MyBRep::Geometry_Circle2D(center, circle.radius(), xDir, yDir));

    return MyBRep::Geometry_Revolved::ProfileSegment(curve, edge.firstParameter(), edge.lastParameter());
}

// 将单个三维Topology_Edge转换为Geometry_Revolved使用的二维母线段。
MyBRep::Geometry_Revolved::ProfileSegment createProfileSegment(const MyBRep::Topology_Edge& edge, double profileTolerance)
{
    MYBREP_ASSERT_MESSAGE(edge.isValid(), "Revolved modeling requires valid profile Edges.");

    if (edge.geometry().kind() == MyBRep::CurveKind::Line) return createLineProfileSegment(edge, profileTolerance);
    if (edge.geometry().kind() == MyBRep::CurveKind::Circle) return createCircleProfileSegment(edge, profileTolerance);

    MYBREP_ASSERT_MESSAGE(false, "Revolved modeling currently supports only Line and Circle profile Edges.");
    return MyBRep::Geometry_Revolved::ProfileSegment(MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D>(), 0.0, 1.0);
}

// 将有向三维Topology_Wire转换为Geometry_Revolved需要的有序二维母线段序列。
std::vector<MyBRep::Geometry_Revolved::ProfileSegment> profileSegments(const MyBRep::Topology_Wire& profile, double profileTolerance)
{
    MYBREP_ASSERT_MESSAGE(profile.isValid() && profile.isClosed() && profile.edgeCount() > 0,
                          "Revolved modeling requires a valid non-empty closed Topology_Wire.");

    std::vector<MyBRep::Geometry_Revolved::ProfileSegment> segments;
    segments.reserve(profile.edgeCount());

    for (std::size_t index = 0; index < profile.edgeCount(); ++index)
        segments.push_back(createProfileSegment(profile.edge(index), profileTolerance));

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
    MYBREP_ASSERT_MESSAGE(profile.isValid() && profile.isClosed() && profile.edgeCount() > 0,
                          "Revolved modeling requires a valid non-empty closed Topology_Wire.");
    MYBREP_ASSERT_MESSAGE(isFiniteNonNegative(profileTolerance),
                          "Revolved modeling profile tolerance must be finite and non-negative.");

    const std::vector<Geometry_Revolved::ProfileSegment> segments = profileSegments(profile, profileTolerance);
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
