#include "Shape.h"

#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

Shape::Shape()
    : Instance_Object<Topology_Shape>()
{
}

Shape::Shape(const Topology_Shape& topology)
    : Instance_Object<Topology_Shape>(topology)
{
    initialize();
}

Shape::Shape(const Topology_Shape& topology, const MyMath::Matrix4& localToWorld)
    : Instance_Object<Topology_Shape>(topology, localToWorld)
{
    initialize();
}

/// 状态判断

bool Shape::isValid() const
{
    return Instance_Object<Topology_Shape>::isValid() && m_worldBounds.isValid();
}

Shape::operator bool() const
{
    return isValid();
}

bool Shape::sharesGeometryWith(const Shape& other) const
{
    const Geometry_Shape* currentGeometry = geometryPointer();
    const Geometry_Shape* otherGeometry = other.geometryPointer();
    return currentGeometry && currentGeometry == otherGeometry;
}

/// 几何内核

const Geometry_Shape& Shape::geometry() const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot access the geometry of an invalid Shape.");
    return topology().geometry();
}

const Geometry_Shape* Shape::geometryPointer() const
{
    return Instance_Object<Topology_Shape>::isValid() ? &topology().geometry() : 0;
}

ShapeKind Shape::kind() const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot access the kind of an invalid Shape.");
    return geometry().kind();
}

/// 空间放置

bool Shape::setLocalToWorld(const MyMath::Matrix4& localToWorld)
{
    if (!Instance_Object<Topology_Shape>::setLocalToWorld(localToWorld))
    {
        return false;
    }

    if (Instance_Object<Topology_Shape>::isValid())
    {
        initialize();
    }

    return true;
}

/// 空间范围

const Bounds3& Shape::localBounds() const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot access the local bounds of an invalid Shape.");
    return geometry().localBounds();
}

const Bounds3& Shape::worldBounds() const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot access the world bounds of an invalid Shape.");
    return m_worldBounds;
}

/// 局部空间查询

bool Shape::containsLocalPoint(const MyMath::Vector3& point) const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Shape.");
    return geometry().containsLocalPoint(point);
}

ShapeRelation Shape::classifyLocalBounds(const Bounds3& bounds) const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Shape.");
    return geometry().classifyLocalBounds(bounds);
}

ShapeRelation Shape::classifyLocalBoundsFast(const MyMath::Vector3& center, const MyMath::Vector3& extent) const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Shape.");
    return geometry().classifyLocalBoundsFast(center, extent);
}

/// 世界空间查询

bool Shape::containsWorldPoint(const MyMath::Vector3& point) const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Shape.");
    MYBREP_ASSERT_MESSAGE(point.isFinite(), "Shape world query point must be finite.");

    if (!m_worldBounds.contains(point))
    {
        return false;
    }

    return geometry().containsLocalPoint(worldToLocal().transformPoint(point));
}

ShapeRelation Shape::classifyWorldBounds(const Bounds3& bounds) const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Shape.");
    MYBREP_ASSERT_MESSAGE(bounds.isValid(), "Shape world query bounds must be valid.");

    if (!m_worldBounds.intersects(bounds))
    {
        return ShapeRelation::Outside;
    }

    return geometry().classifyLocalBounds(bounds.transformed(worldToLocal()));
}

/// 方向操作

Shape Shape::reversed() const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot reverse an invalid Shape.");
    return Shape(topology().reversed(), localToWorld());
}

/// 初始化

void Shape::initialize()
{
    MYBREP_ASSERT_MESSAGE(Instance_Object<Topology_Shape>::isValid(), "Shape topology must be valid.");
    m_worldBounds = topology().geometry().localBounds().transformed(localToWorld());
    MYBREP_ASSERT_MESSAGE(m_worldBounds.isValid(), "Shape transformed world bounds must be valid.");
}

}
