#include "Edge.h"

#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

Edge::Edge()
    : Instance_Object()
{
}

Edge::Edge(const Topology_Edge& topology)
    : Instance_Object()
    , m_topology(topology)
{
    MYBREP_ASSERT_MESSAGE(topology.isValid(),
                          "Edge topology must be valid.");
}

Edge::Edge(const Topology_Edge& topology,
           const MyMath::Matrix4& localToWorld)
    : Instance_Object(localToWorld)
    , m_topology(topology)
{
    MYBREP_ASSERT_MESSAGE(topology.isValid(),
                          "Edge topology must be valid.");
}

/// 状态判断

bool Edge::isValid() const
{
    return m_topology.isValid();
}

bool Edge::isNull() const
{
    return m_topology.isNull();
}

Edge::operator bool() const
{
    return isValid();
}

bool Edge::sharesTopologyWith(const Edge& other) const
{
    return m_topology.isValid() &&
           other.m_topology.isValid() &&
           m_topology.isSame(other.m_topology);
}

bool Edge::sharesGeometryWith(const Edge& other) const
{
    const Geometry_Curve* currentGeometry = geometryPointer();
    const Geometry_Curve* otherGeometry = other.geometryPointer();

    return currentGeometry && currentGeometry == otherGeometry;
}

/// 局部拓扑与几何资源

const Topology_Edge& Edge::topology() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access the topology of an invalid Edge.");

    return m_topology;
}

const Geometry_Curve& Edge::geometry() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access the geometry of an invalid Edge.");

    return m_topology.geometry();
}

const Geometry_Curve* Edge::geometryPointer() const
{
    return m_topology.isValid() ? &m_topology.geometry() : 0;
}

CurveKind Edge::kind() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access the kind of an invalid Edge.");

    return geometry().kind();
}

/// 有向端点

MyMath::Vector3 Edge::localStartPoint() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access the local start point of an invalid Edge.");

    return m_topology.startVertex().point();
}

MyMath::Vector3 Edge::localEndPoint() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access the local end point of an invalid Edge.");

    return m_topology.endVertex().point();
}

MyMath::Vector3 Edge::worldStartPoint() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access the world start point of an invalid Edge.");

    return localToWorld().transformPoint(localStartPoint());
}

MyMath::Vector3 Edge::worldEndPoint() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access the world end point of an invalid Edge.");

    return localToWorld().transformPoint(localEndPoint());
}

/// 局部空间查询

MyMath::Vector3 Edge::pointAt(double parameter) const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot query an invalid Edge.");

    return m_topology.pointAt(parameter);
}

MyMath::Vector3 Edge::tangentAt(double parameter) const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot query an invalid Edge.");

    return m_topology.tangentAt(parameter);
}

/// 世界空间查询

MyMath::Vector3 Edge::worldPointAt(double parameter) const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot query an invalid Edge.");

    return localToWorld().transformPoint(m_topology.pointAt(parameter));
}

MyMath::Vector3 Edge::worldTangentAt(double parameter) const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot query an invalid Edge.");

    const MyMath::Vector3 worldTangent =
        localToWorld().transformVector(m_topology.tangentAt(parameter));

    MYBREP_ASSERT_MESSAGE(worldTangent.isVector(0.0),
                          "Edge world tangent must remain non-zero under an invertible affine transform.");

    return worldTangent.normalized(0.0);
}

/// 方向操作

Edge Edge::reversed() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot reverse an invalid Edge.");

    return Edge(m_topology.reversed(), localToWorld());
}

}
