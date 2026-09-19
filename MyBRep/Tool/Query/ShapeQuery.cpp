#include "ShapeQuery.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

const unsigned int ShapeQuery::XOctantMask = 1;
const unsigned int ShapeQuery::YOctantMask = 2;
const unsigned int ShapeQuery::ZOctantMask = 4;
const double ShapeQuery::SignedDistanceMetricToleranceScale = 256.0;

/// 构造

ShapeQuery::ShapeQuery(const Topology_Shape& topology)
    : m_queryToLocal(MyMath::Matrix4::identity())
    , m_localToQuery(MyMath::Matrix4::identity())
    , m_absoluteQueryToLocalRowX(MyMath::Vector3::unitX())
    , m_absoluteQueryToLocalRowY(MyMath::Vector3::unitY())
    , m_absoluteQueryToLocalRowZ(MyMath::Vector3::unitZ())
    , m_localDistancePerQueryUnit(1.0)
    , m_signedDistanceMetricValid(false)
    , m_identityQuery(true)
{
    initialize(topology, MyMath::Matrix4::identity(), MyMath::Matrix4::identity());
}

ShapeQuery::ShapeQuery(const Topology_Shape& topology, const MyMath::Matrix4& localToWorld, const MyMath::Matrix4& queryToWorld)
    : m_queryToLocal(MyMath::Matrix4::identity())
    , m_localToQuery(MyMath::Matrix4::identity())
    , m_absoluteQueryToLocalRowX(MyMath::Vector3::unitX())
    , m_absoluteQueryToLocalRowY(MyMath::Vector3::unitY())
    , m_absoluteQueryToLocalRowZ(MyMath::Vector3::unitZ())
    , m_localDistancePerQueryUnit(1.0)
    , m_signedDistanceMetricValid(false)
    , m_identityQuery(false)
{
    initialize(topology, localToWorld, queryToWorld);
}

ShapeQuery::ShapeQuery(const Shape& shape)
    : m_queryToLocal(MyMath::Matrix4::identity())
    , m_localToQuery(MyMath::Matrix4::identity())
    , m_absoluteQueryToLocalRowX(MyMath::Vector3::unitX())
    , m_absoluteQueryToLocalRowY(MyMath::Vector3::unitY())
    , m_absoluteQueryToLocalRowZ(MyMath::Vector3::unitZ())
    , m_localDistancePerQueryUnit(1.0)
    , m_signedDistanceMetricValid(false)
    , m_identityQuery(false)
{
    MYBREP_ASSERT_MESSAGE(shape.isValid(), "ShapeQuery requires a valid Shape.");
    initialize(shape.topology(), shape.localToWorld(), MyMath::Matrix4::identity());
}

ShapeQuery::ShapeQuery(const Shape& shape, const MyMath::Matrix4& queryToWorld)
    : m_queryToLocal(MyMath::Matrix4::identity())
    , m_localToQuery(MyMath::Matrix4::identity())
    , m_absoluteQueryToLocalRowX(MyMath::Vector3::unitX())
    , m_absoluteQueryToLocalRowY(MyMath::Vector3::unitY())
    , m_absoluteQueryToLocalRowZ(MyMath::Vector3::unitZ())
    , m_localDistancePerQueryUnit(1.0)
    , m_signedDistanceMetricValid(false)
    , m_identityQuery(false)
{
    MYBREP_ASSERT_MESSAGE(shape.isValid(), "ShapeQuery requires a valid Shape.");
    initialize(shape.topology(), shape.localToWorld(), queryToWorld);
}

ShapeQuery::~ShapeQuery()
{
}

/// 查询能力

bool ShapeQuery::isIdentityQuery() const
{
    return m_identityQuery;
}

bool ShapeQuery::supportsSignedDistance() const
{
    return m_signedDistanceMetricValid &&
           geometry().supportsSignedDistance();
}

/// 查询对象与空间数据

const Topology_Shape& ShapeQuery::topology() const
{
    return m_topology;
}

const Geometry_Shape& ShapeQuery::geometry() const
{
    return m_topology.geometry();
}

const MyMath::Matrix4& ShapeQuery::queryToLocal() const
{
    return m_queryToLocal;
}

const MyMath::Matrix4& ShapeQuery::localToQuery() const
{
    return m_localToQuery;
}

const Bounds3& ShapeQuery::queryBounds() const
{
    return m_queryBounds;
}

/// 标准空间查询

bool ShapeQuery::containsPoint(const MyMath::Vector3& point) const
{
    MYBREP_ASSERT_MESSAGE(point.isFinite(),
                          "ShapeQuery point must be finite.");

    if (!m_queryBounds.contains(point))
    {
        return false;
    }

    if (m_identityQuery)
    {
        return geometry().containsLocalPoint(point);
    }

    return geometry().containsLocalPoint(
        m_queryToLocal.transformPoint(point));
}

double ShapeQuery::signedDistanceToPoint(const MyMath::Vector3& point) const
{
    MYBREP_ASSERT_MESSAGE(point.isFinite(),
                          "ShapeQuery signed-distance point must be finite.");
    MYBREP_ASSERT_MESSAGE(supportsSignedDistance(),
                          "ShapeQuery signed distance requires supported geometry and a rigid or uniform-scale query transform.");

    const MyMath::Vector3 localPoint =
        m_identityQuery
            ? point
            : m_queryToLocal.transformPoint(point);

    const double localDistance =
        geometry().signedDistanceLocalPoint(localPoint);

    return localDistance / m_localDistancePerQueryUnit;
}

ShapeRelation ShapeQuery::classifyBounds(const Bounds3& bounds) const
{
    MYBREP_ASSERT_MESSAGE(bounds.isValid(),
                          "ShapeQuery bounds must be valid.");

    if (!m_queryBounds.intersects(bounds))
    {
        return ShapeRelation::Outside;
    }

    if (m_identityQuery)
    {
        return geometry().classifyLocalBounds(bounds);
    }

    // 标准路径显式变换查询AABB八个角点并建立局部保守AABB，作为快速路径的独立正确性基准。
    return geometry().classifyLocalBounds(
        bounds.transformed(m_queryToLocal));
}

/// 快速空间查询

ShapeRelation ShapeQuery::classifyBoundsFast(
    const MyMath::Vector3& center,
    const MyMath::Vector3& extent) const
{
    MYBREP_ASSERT_MESSAGE(center.isFinite(),
                          "ShapeQuery bounds center must be finite.");
    MYBREP_ASSERT_MESSAGE(isValidExtent(extent),
                          "ShapeQuery bounds extent must be finite and non-negative.");

    if (!intersectsQueryBounds(center, extent))
    {
        return ShapeRelation::Outside;
    }

    return classifyBoundsFastImpl(center, extent);
}

void ShapeQuery::classifyOctantBoundsFast(
    const MyMath::Vector3& parentCenter,
    const MyMath::Vector3& childExtent,
    std::array<ShapeRelation, OctantCount>& results) const
{
    MYBREP_ASSERT_MESSAGE(parentCenter.isFinite(),
                          "ShapeQuery parent center must be finite.");
    MYBREP_ASSERT_MESSAGE(isValidExtent(childExtent),
                          "ShapeQuery child extent must be finite and non-negative.");

    classifyOctantBoundsFastImpl(parentCenter,
                                 childExtent,
                                 results);
}

/// 特殊Shape快速查询扩展

ShapeRelation ShapeQuery::classifyBoundsFastImpl(
    const MyMath::Vector3& center,
    const MyMath::Vector3& extent) const
{
    if (m_identityQuery)
    {
        return geometry().classifyLocalBoundsFast(center, extent);
    }

    MyMath::Vector3 localCenter;
    MyMath::Vector3 localExtent;

    transformBoundsToLocalFast(center,
                               extent,
                               localCenter,
                               localExtent);

    return geometry().classifyLocalBoundsFast(localCenter,
                                           localExtent);
}

void ShapeQuery::classifyOctantBoundsFastImpl(
    const MyMath::Vector3& parentCenter,
    const MyMath::Vector3& childExtent,
    std::array<ShapeRelation, OctantCount>& results) const
{
    if (m_identityQuery)
    {
        for (unsigned int octantIndex = 0;
             octantIndex < static_cast<unsigned int>(OctantCount);
             ++octantIndex)
        {
            const MyMath::Vector3 childCenter =
                octantCenter(parentCenter,
                              childExtent,
                              octantIndex);

            results[octantIndex] =
                intersectsQueryBounds(childCenter, childExtent)
                    ? geometry().classifyLocalBoundsFast(childCenter,
                                                      childExtent)
                    : ShapeRelation::Outside;
        }

        return;
    }

    const MyMath::Vector3 localParentCenter =
        m_queryToLocal.transformPoint(parentCenter);

    const MyMath::Vector3 localOffsetX(
        m_queryToLocal(0, 0) * childExtent.x(),
        m_queryToLocal(1, 0) * childExtent.x(),
        m_queryToLocal(2, 0) * childExtent.x());

    const MyMath::Vector3 localOffsetY(
        m_queryToLocal(0, 1) * childExtent.y(),
        m_queryToLocal(1, 1) * childExtent.y(),
        m_queryToLocal(2, 1) * childExtent.y());

    const MyMath::Vector3 localOffsetZ(
        m_queryToLocal(0, 2) * childExtent.z(),
        m_queryToLocal(1, 2) * childExtent.z(),
        m_queryToLocal(2, 2) * childExtent.z());

    const MyMath::Vector3 localExtent(
        MyMath::Vector3::dot(m_absoluteQueryToLocalRowX, childExtent),
        MyMath::Vector3::dot(m_absoluteQueryToLocalRowY, childExtent),
        MyMath::Vector3::dot(m_absoluteQueryToLocalRowZ, childExtent));

    for (unsigned int octantIndex = 0;
         octantIndex < static_cast<unsigned int>(OctantCount);
         ++octantIndex)
    {
        const MyMath::Vector3 childCenter =
            octantCenter(parentCenter,
                          childExtent,
                          octantIndex);

        if (!intersectsQueryBounds(childCenter, childExtent))
        {
            results[octantIndex] = ShapeRelation::Outside;
            continue;
        }

        const double signX =
            (octantIndex & XOctantMask) != 0 ? 1.0 : -1.0;
        const double signY =
            (octantIndex & YOctantMask) != 0 ? 1.0 : -1.0;
        const double signZ =
            (octantIndex & ZOctantMask) != 0 ? 1.0 : -1.0;

        const MyMath::Vector3 localCenter(
            localParentCenter.x() +
                signX * localOffsetX.x() +
                signY * localOffsetY.x() +
                signZ * localOffsetZ.x(),
            localParentCenter.y() +
                signX * localOffsetX.y() +
                signY * localOffsetY.y() +
                signZ * localOffsetZ.y(),
            localParentCenter.z() +
                signX * localOffsetX.z() +
                signY * localOffsetY.z() +
                signZ * localOffsetZ.z());

        results[octantIndex] =
            geometry().classifyLocalBoundsFast(localCenter,
                                            localExtent);
    }
}

/// 通用快速查询辅助

void ShapeQuery::transformBoundsToLocalFast(
    const MyMath::Vector3& center,
    const MyMath::Vector3& extent,
    MyMath::Vector3& localCenter,
    MyMath::Vector3& localExtent) const
{
    localCenter = m_queryToLocal.transformPoint(center);

    localExtent.set(
        MyMath::Vector3::dot(m_absoluteQueryToLocalRowX, extent),
        MyMath::Vector3::dot(m_absoluteQueryToLocalRowY, extent),
        MyMath::Vector3::dot(m_absoluteQueryToLocalRowZ, extent));
}

bool ShapeQuery::intersectsQueryBounds(
    const MyMath::Vector3& center,
    const MyMath::Vector3& extent) const
{
    return center.x() + extent.x() >= m_queryBounds.minimum().x() &&
           center.x() - extent.x() <= m_queryBounds.maximum().x() &&
           center.y() + extent.y() >= m_queryBounds.minimum().y() &&
           center.y() - extent.y() <= m_queryBounds.maximum().y() &&
           center.z() + extent.z() >= m_queryBounds.minimum().z() &&
           center.z() - extent.z() <= m_queryBounds.maximum().z();
}

/// 初始化

/// 初始化

void ShapeQuery::initialize(const Topology_Shape& topology, const MyMath::Matrix4& localToWorld, const MyMath::Matrix4& queryToWorld)
{
    MYBREP_ASSERT_MESSAGE(topology.isValid(), "ShapeQuery requires a valid Topology_Shape.");
    MYBREP_ASSERT_MESSAGE(localToWorld.isAffine(), "ShapeQuery local-to-world transform must be affine.");
    MYBREP_ASSERT_MESSAGE(queryToWorld.isAffine(), "ShapeQuery query-to-world transform must be affine.");

    MyMath::Matrix4 worldToLocal;
    MyMath::Matrix4 worldToQuery;
    const bool localInvertible = localToWorld.inverted(worldToLocal);
    const bool queryInvertible = queryToWorld.inverted(worldToQuery);

    MYBREP_ASSERT_MESSAGE(localInvertible, "ShapeQuery local-to-world transform must be invertible.");
    MYBREP_ASSERT_MESSAGE(queryInvertible, "ShapeQuery query-to-world transform must be invertible.");

    m_topology = topology;
    m_queryToLocal = worldToLocal * queryToWorld;
    m_localToQuery = worldToQuery * localToWorld;
    m_identityQuery = m_queryToLocal.isIdentity(0.0);

    updateAbsoluteQueryToLocalRows();
    updateSignedDistanceMetric();

    m_queryBounds = geometry().localBounds().transformed(m_localToQuery);

    MYBREP_ASSERT_MESSAGE(m_queryBounds.isValid(), "ShapeQuery query bounds must be valid.");
    MYBREP_ASSERT_MESSAGE(m_queryBounds.hasVolume(), "ShapeQuery requires Geometry_Shape query bounds with positive volume.");
}

/// 内部辅助

bool ShapeQuery::isValidExtent(const MyMath::Vector3& extent)
{
    return extent.isFinite() && extent.x() >= 0.0 && extent.y() >= 0.0 && extent.z() >= 0.0;
}

MyMath::Vector3 ShapeQuery::octantCenter(const MyMath::Vector3& parentCenter, const MyMath::Vector3& childExtent,
                                        unsigned int octantIndex)
{
    return MyMath::Vector3(
        parentCenter.x() + ((octantIndex & XOctantMask) != 0 ? childExtent.x() : -childExtent.x()),
        parentCenter.y() + ((octantIndex & YOctantMask) != 0 ? childExtent.y() : -childExtent.y()),
        parentCenter.z() + ((octantIndex & ZOctantMask) != 0 ? childExtent.z() : -childExtent.z()));
}

void ShapeQuery::updateAbsoluteQueryToLocalRows()
{
    m_absoluteQueryToLocalRowX.set(
        std::fabs(m_queryToLocal(0, 0)),
        std::fabs(m_queryToLocal(0, 1)),
        std::fabs(m_queryToLocal(0, 2)));

    m_absoluteQueryToLocalRowY.set(
        std::fabs(m_queryToLocal(1, 0)),
        std::fabs(m_queryToLocal(1, 1)),
        std::fabs(m_queryToLocal(1, 2)));

    m_absoluteQueryToLocalRowZ.set(
        std::fabs(m_queryToLocal(2, 0)),
        std::fabs(m_queryToLocal(2, 1)),
        std::fabs(m_queryToLocal(2, 2)));
}

void ShapeQuery::updateSignedDistanceMetric()
{
    const MyMath::Vector3 columnX(
        m_queryToLocal(0, 0),
        m_queryToLocal(1, 0),
        m_queryToLocal(2, 0));

    const MyMath::Vector3 columnY(
        m_queryToLocal(0, 1),
        m_queryToLocal(1, 1),
        m_queryToLocal(2, 1));

    const MyMath::Vector3 columnZ(
        m_queryToLocal(0, 2),
        m_queryToLocal(1, 2),
        m_queryToLocal(2, 2));

    const double lengthSquaredX =
        MyMath::Vector3::dot(columnX, columnX);
    const double lengthSquaredY =
        MyMath::Vector3::dot(columnY, columnY);
    const double lengthSquaredZ =
        MyMath::Vector3::dot(columnZ, columnZ);

    const double maximumLengthSquared =
        (std::max)(1.0,
                   (std::max)(lengthSquaredX,
                              (std::max)(lengthSquaredY,
                                         lengthSquaredZ)));

    const double tolerance =
        std::numeric_limits<double>::epsilon() *
        SignedDistanceMetricToleranceScale *
        maximumLengthSquared;

    const bool equalScale =
        std::fabs(lengthSquaredX - lengthSquaredY) <= tolerance &&
        std::fabs(lengthSquaredX - lengthSquaredZ) <= tolerance;

    const bool orthogonal =
        std::fabs(MyMath::Vector3::dot(columnX, columnY)) <= tolerance &&
        std::fabs(MyMath::Vector3::dot(columnX, columnZ)) <= tolerance &&
        std::fabs(MyMath::Vector3::dot(columnY, columnZ)) <= tolerance;

    m_signedDistanceMetricValid =
        equalScale &&
        orthogonal &&
        lengthSquaredX > 0.0;

    m_localDistancePerQueryUnit =
        m_signedDistanceMetricValid
            ? std::sqrt(
                  (lengthSquaredX +
                   lengthSquaredY +
                   lengthSquaredZ) /
                  3.0)
            : 1.0;
}


}
