#include "WireQuery.h"

#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

WireQuery::WireQuery(const Topology_Wire& topology)
    : m_queryToWorld(MyMath::Matrix4::identity())
{
    MYBREP_ASSERT_MESSAGE(topology.isValid(),
                          "WireQuery requires a valid Topology_Wire.");

    initialize(Wire(topology), MyMath::Matrix4::identity());
}

WireQuery::WireQuery(const Wire& wire)
    : m_queryToWorld(MyMath::Matrix4::identity())
{
    initialize(wire, MyMath::Matrix4::identity());
}

WireQuery::WireQuery(const Wire& wire,
                     const MyMath::Matrix4& queryToWorld)
    : m_queryToWorld(MyMath::Matrix4::identity())
{
    initialize(wire, queryToWorld);
}

/// 查询对象与空间数据

const Wire& WireQuery::wire() const
{
    return m_wire;
}

const Topology_Wire& WireQuery::topology() const
{
    return m_wire.topology();
}

const Bounds3& WireQuery::queryBounds() const
{
    return m_queryBounds;
}

/// Edge查询

std::size_t WireQuery::edgeCount() const
{
    return m_wire.edgeCount();
}

EdgeQuery WireQuery::edgeQuery(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(index < edgeCount(),
                          "WireQuery Edge index is out of range.");

    return EdgeQuery(m_wire.edge(index),
                     m_queryToWorld);
}

/// 线性度量

double WireQuery::length(double absoluteTolerance,
                         unsigned int maxSubdivisionDepth) const
{
    double totalLength = 0.0;

    for (std::size_t index = 0;
         index < edgeCount();
         ++index)
    {
        totalLength +=
            edgeQuery(index).length(absoluteTolerance,
                                    maxSubdivisionDepth);
    }

    return totalLength;
}

/// 初始化

void WireQuery::initialize(const Wire& wire,
                           const MyMath::Matrix4& queryToWorld)
{
    MYBREP_ASSERT_MESSAGE(wire.isValid(),
                          "WireQuery requires a valid Wire.");
    MYBREP_ASSERT_MESSAGE(queryToWorld.isAffine(),
                          "WireQuery query-to-world transform must be affine.");

    MyMath::Matrix4 worldToQuery;
    const bool invertible =
        queryToWorld.inverted(worldToQuery);

    MYBREP_ASSERT_MESSAGE(invertible,
                          "WireQuery query-to-world transform must be invertible.");

    m_wire = wire;
    m_queryToWorld = queryToWorld;
    m_queryBounds.clear();

    for (std::size_t index = 0;
         index < m_wire.edgeCount();
         ++index)
    {
        m_queryBounds.include(
            EdgeQuery(m_wire.edge(index),
                      m_queryToWorld).queryBounds());
    }

    MYBREP_ASSERT_MESSAGE(m_queryBounds.isValid(),
                          "WireQuery bounds must be valid.");
}

}
