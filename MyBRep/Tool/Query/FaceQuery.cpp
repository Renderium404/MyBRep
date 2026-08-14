#include "FaceQuery.h"

#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

FaceQuery::FaceQuery(const Topology_Face& topology)
    : m_queryToWorld(MyMath::Matrix4::identity())
    , m_queryToLocal(MyMath::Matrix4::identity())
    , m_localToQuery(MyMath::Matrix4::identity())
{
    MYBREP_ASSERT_MESSAGE(topology.isValid(),
                          "FaceQuery requires a valid Topology_Face.");

    initialize(Face(topology),
               MyMath::Matrix4::identity());
}

FaceQuery::FaceQuery(const Face& face)
    : m_queryToWorld(MyMath::Matrix4::identity())
    , m_queryToLocal(MyMath::Matrix4::identity())
    , m_localToQuery(MyMath::Matrix4::identity())
{
    initialize(face, MyMath::Matrix4::identity());
}

FaceQuery::FaceQuery(const Face& face,
                     const MyMath::Matrix4& queryToWorld)
    : m_queryToWorld(MyMath::Matrix4::identity())
    , m_queryToLocal(MyMath::Matrix4::identity())
    , m_localToQuery(MyMath::Matrix4::identity())
{
    initialize(face, queryToWorld);
}

/// 查询对象与空间数据

const Face& FaceQuery::face() const
{
    return m_face;
}

const Topology_Face& FaceQuery::topology() const
{
    return m_face.topology();
}

const Geometry_Surface& FaceQuery::geometry() const
{
    return m_face.geometry();
}

const MyMath::Matrix4& FaceQuery::queryToLocal() const
{
    return m_queryToLocal;
}

const MyMath::Matrix4& FaceQuery::localToQuery() const
{
    return m_localToQuery;
}

/// 曲面参数查询

MyMath::Vector3 FaceQuery::pointAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(geometry().isParameterInDomain(u, v),
                          "FaceQuery parameters are outside the Geometry_Surface natural parameter domain.");

    return m_localToQuery.transformPoint(
        geometry().pointAt(u, v));
}

MyMath::Vector3 FaceQuery::normalAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(geometry().isParameterInDomain(u, v),
                          "FaceQuery normal parameters are outside the Geometry_Surface natural parameter domain.");

    const MyMath::Vector3 queryUDerivative =
        m_localToQuery.transformVector(
            geometry().firstDerivativeUAt(u, v));
    const MyMath::Vector3 queryVDerivative =
        m_localToQuery.transformVector(
            geometry().firstDerivativeVAt(u, v));

    MyMath::Vector3 queryNormal =
        MyMath::Vector3::cross(queryUDerivative,
                               queryVDerivative);

    MYBREP_ASSERT_MESSAGE(queryNormal.isVector(0.0),
                          "FaceQuery normal requires a regular transformed surface parameter.");

    queryNormal = queryNormal.normalized(0.0);

    return topology().isForward()
               ? queryNormal
               : queryNormal * -1.0;
}

/// 裁剪边界查询

std::size_t FaceQuery::wireCount() const
{
    return topology().wireCount();
}

std::size_t FaceQuery::edgeCount(std::size_t wireIndex) const
{
    MYBREP_ASSERT_MESSAGE(wireIndex < wireCount(),
                          "FaceQuery Wire index is out of range.");

    return topology().wire(wireIndex).edgeCount();
}

EdgeQuery FaceQuery::edgeQuery(std::size_t wireIndex,
                               std::size_t edgeIndex) const
{
    MYBREP_ASSERT_MESSAGE(wireIndex < wireCount(),
                          "FaceQuery Wire index is out of range.");
    MYBREP_ASSERT_MESSAGE(edgeIndex < edgeCount(wireIndex),
                          "FaceQuery Edge index is out of range.");

    return EdgeQuery(
        m_face.wire(wireIndex).edge(edgeIndex),
        m_queryToWorld);
}

MyMath::Vector2 FaceQuery::surfaceParameterAt(std::size_t wireIndex,
                                              std::size_t edgeIndex,
                                              double parameter) const
{
    MYBREP_ASSERT_MESSAGE(wireIndex < wireCount(),
                          "FaceQuery Wire index is out of range.");
    MYBREP_ASSERT_MESSAGE(edgeIndex < edgeCount(wireIndex),
                          "FaceQuery Edge index is out of range.");

    const Topology_Edge edge =
        topology().wire(wireIndex).edge(edgeIndex);

    return edge.surfaceParameterAt(geometry(),
                                   parameter);
}

Bounds3 FaceQuery::boundaryBounds() const
{
    Bounds3 result;

    for (std::size_t wireIndex = 0;
         wireIndex < wireCount();
         ++wireIndex)
    {
        for (std::size_t edgeIndex = 0;
             edgeIndex < edgeCount(wireIndex);
             ++edgeIndex)
        {
            result.include(
                edgeQuery(wireIndex,
                          edgeIndex).queryBounds());
        }
    }

    return result;
}

/// 初始化

void FaceQuery::initialize(const Face& face,
                           const MyMath::Matrix4& queryToWorld)
{
    MYBREP_ASSERT_MESSAGE(face.isValid(),
                          "FaceQuery requires a valid Face.");
    MYBREP_ASSERT_MESSAGE(queryToWorld.isAffine(),
                          "FaceQuery query-to-world transform must be affine.");

    MyMath::Matrix4 worldToQuery;
    const bool invertible =
        queryToWorld.inverted(worldToQuery);

    MYBREP_ASSERT_MESSAGE(invertible,
                          "FaceQuery query-to-world transform must be invertible.");

    m_face = face;
    m_queryToWorld = queryToWorld;
    m_queryToLocal =
        face.worldToLocal() * queryToWorld;
    m_localToQuery =
        worldToQuery * face.localToWorld();
}

}
