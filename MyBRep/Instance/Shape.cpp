#include "Shape.h"

#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

Shape::Shape()
{
}

Shape::Shape(const Topology_Shape& topology)
    : Instance(topology)
{
    updateWorldBounds();
}

Shape::Shape(const Topology_Shape& topology, const MyMath::Matrix4& localToWorld)
    : Instance(topology, localToWorld)
{
    updateWorldBounds();
}

Topology_Shape Shape::topology() const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot access the topology of an invalid Shape.");
    return Topology_Shape(topologyObject());
}

bool Shape::sharesGeometryWith(const Shape& other) const
{
    const Geometry_Shape* currentGeometry = geometryPointer();
    const Geometry_Shape* otherGeometry = other.geometryPointer();
    return currentGeometry && currentGeometry == otherGeometry;
}

const Geometry_Shape& Shape::geometry() const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot access the geometry of an invalid Shape.");
    return topology().geometry();
}

const Geometry_Shape* Shape::geometryPointer() const
{
    return isValid() ? &geometry() : 0;
}

ShapeKind Shape::kind() const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot access the kind of an invalid Shape.");
    return geometry().kind();
}

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

bool Shape::containsWorldPoint(const MyMath::Vector3& point) const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Shape.");
    MYBREP_ASSERT_MESSAGE(point.isFinite(), "Shape world query point must be finite.");

    if (!m_worldBounds.contains(point)) return false;

    return geometry().containsLocalPoint(worldToLocal().transformPoint(point));
}

ShapeRelation Shape::classifyWorldBounds(const Bounds3& bounds) const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot query an invalid Shape.");
    MYBREP_ASSERT_MESSAGE(bounds.isValid(), "Shape world query bounds must be valid.");

    if (!m_worldBounds.intersects(bounds)) return ShapeRelation::Outside;

    return geometry().classifyLocalBounds(bounds.transformed(worldToLocal()));
}

Shape Shape::reversed() const
{
    MYBREP_ASSERT_MESSAGE(isValid(), "Cannot reverse an invalid Shape.");
    return Shape(topology().reversed(), localToWorld());
}

void Shape::onInstanceChanged()
{
    updateWorldBounds();
}

void Shape::updateWorldBounds()
{
    m_worldBounds.clear();

    if (!isValid()) return;

    m_worldBounds = geometry().localBounds().transformed(localToWorld());

    MYBREP_ASSERT_MESSAGE(m_worldBounds.isValid(),
                          "Shape transformed world bounds must be valid.");
}

}