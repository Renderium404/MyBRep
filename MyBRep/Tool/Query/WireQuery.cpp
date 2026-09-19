#include "WireQuery.h"

#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

WireQuery::WireQuery(const Topology_Wire& topology)
    : m_localToWorld(MyMath::Matrix4::identity())
    , m_queryToWorld(MyMath::Matrix4::identity())
{
    initialize(topology, MyMath::Matrix4::identity(), MyMath::Matrix4::identity());
}

WireQuery::WireQuery(const Topology_Wire& topology, const MyMath::Matrix4& localToWorld, const MyMath::Matrix4& queryToWorld)
    : m_localToWorld(MyMath::Matrix4::identity())
    , m_queryToWorld(MyMath::Matrix4::identity())
{
    initialize(topology, localToWorld, queryToWorld);
}

WireQuery::WireQuery(const Wire& wire)
    : m_localToWorld(MyMath::Matrix4::identity())
    , m_queryToWorld(MyMath::Matrix4::identity())
{
    MYBREP_ASSERT_MESSAGE(wire.isValid(), "WireQuery requires a valid Wire.");
    initialize(wire.topology(), wire.localToWorld(), MyMath::Matrix4::identity());
}

WireQuery::WireQuery(const Wire& wire, const MyMath::Matrix4& queryToWorld)
    : m_localToWorld(MyMath::Matrix4::identity())
    , m_queryToWorld(MyMath::Matrix4::identity())
{
    MYBREP_ASSERT_MESSAGE(wire.isValid(), "WireQuery requires a valid Wire.");
    initialize(wire.topology(), wire.localToWorld(), queryToWorld);
}

/// 查询对象与空间数据

const Topology_Wire& WireQuery::topology() const
{
    return m_topology;
}

const Bounds3& WireQuery::queryBounds() const
{
    return m_queryBounds;
}

/// Edge查询

std::size_t WireQuery::edgeCount() const
{
    return m_topology.edgeCount();
}

EdgeQuery WireQuery::edgeQuery(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(index < edgeCount(), "WireQuery Edge index is out of range.");
    return EdgeQuery(m_topology.edge(index), m_localToWorld, m_queryToWorld);
}

/// 线性度量

double WireQuery::length(double absoluteTolerance, unsigned int maxSubdivisionDepth) const
{
    double totalLength = 0.0;

    for (std::size_t index = 0; index < edgeCount(); ++index)
    {
        totalLength += edgeQuery(index).length(absoluteTolerance, maxSubdivisionDepth);
    }

    return totalLength;
}

/// 初始化

void WireQuery::initialize(const Topology_Wire& topology, const MyMath::Matrix4& localToWorld, const MyMath::Matrix4& queryToWorld)
{
    MYBREP_ASSERT_MESSAGE(topology.isValid(), "WireQuery requires a valid Topology_Wire.");
    MYBREP_ASSERT_MESSAGE(localToWorld.isAffine(), "WireQuery local-to-world transform must be affine.");
    MYBREP_ASSERT_MESSAGE(queryToWorld.isAffine(), "WireQuery query-to-world transform must be affine.");

    MyMath::Matrix4 inverse;
    MYBREP_ASSERT_MESSAGE(localToWorld.inverted(inverse), "WireQuery local-to-world transform must be invertible.");
    MYBREP_ASSERT_MESSAGE(queryToWorld.inverted(inverse), "WireQuery query-to-world transform must be invertible.");

    m_topology = topology;
    m_localToWorld = localToWorld;
    m_queryToWorld = queryToWorld;
    m_queryBounds.clear();

    for (std::size_t index = 0; index < m_topology.edgeCount(); ++index)
    {
        m_queryBounds.include(EdgeQuery(m_topology.edge(index), m_localToWorld, m_queryToWorld).queryBounds());
    }

    MYBREP_ASSERT_MESSAGE(m_queryBounds.isValid(), "WireQuery bounds must be valid.");
}

}
