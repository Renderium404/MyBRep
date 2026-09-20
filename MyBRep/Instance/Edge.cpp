#include "Edge.h"

#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

Edge::Edge()
{
}

Edge::Edge(const Topology_Edge& topology)
    : Instance(topology)
{
}

Edge::Edge(const Topology_Edge& topology, const MyMath::Matrix4& localToWorld)
    : Instance(topology, localToWorld)
{
}

Topology_Edge Edge::topology() const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot access the topology of an invalid Edge.");
    return Topology_Edge(topologyObject());
}

bool Edge::sharesGeometryWith(const Edge& other) const
{
    const Geometry_Curve* currentGeometry = geometryPointer();
    const Geometry_Curve* otherGeometry = other.geometryPointer();
    return currentGeometry && currentGeometry == otherGeometry;
}

const Geometry_Curve& Edge::geometry() const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot access the geometry of an invalid Edge.");
    return topology().geometry();
}

const Geometry_Curve* Edge::geometryPointer() const
{
    return isValid() ? &geometry() : 0;
}

CurveKind Edge::kind() const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot access the kind of an invalid Edge.");
    return geometry().kind();
}

MyMath::Vector3 Edge::localStartPoint() const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot access the local start point of an invalid Edge.");
    return topology().startVertex().point();
}

MyMath::Vector3 Edge::localEndPoint() const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot access the local end point of an invalid Edge.");
    return topology().endVertex().point();
}

MyMath::Vector3 Edge::worldStartPoint() const
{
    return localToWorld().transformPoint(localStartPoint());
}

MyMath::Vector3 Edge::worldEndPoint() const
{
    return localToWorld().transformPoint(localEndPoint());
}

MyMath::Vector3 Edge::pointAt(double parameter) const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Edge.");
    return topology().pointAt(parameter);
}

MyMath::Vector3 Edge::tangentAt(double parameter) const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Edge.");
    return topology().tangentAt(parameter);
}

MyMath::Vector3 Edge::worldPointAt(double parameter) const
{
    return localToWorld().transformPoint(pointAt(parameter));
}

MyMath::Vector3 Edge::worldTangentAt(double parameter) const
{
    const MyMath::Vector3 worldTangent = localToWorld().transformVector(tangentAt(parameter));

    MYBREP_ASSERT_MESSAGE(worldTangent.isVector(0.0),
                          "Edge world tangent must remain non-zero under an invertible affine transform.");

    return worldTangent.normalized(0.0);
}

Edge Edge::reversed() const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot reverse an invalid Edge.");
    return Edge(topology().reversed(), localToWorld());
}

}