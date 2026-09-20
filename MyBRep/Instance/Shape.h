#ifndef MYBREP_INSTANCE_SHAPE_H
#define MYBREP_INSTANCE_SHAPE_H

#include "MyMath/Vector3.h"
#include "MyBRep/Base/Bounds3.h"
#include "MyBRep/Geometry/Shape/Geometry_Shape.h"
#include "MyBRep/Instance/Instance.h"
#include "MyBRep/Topology/Shape/Topology_Shape.h"

namespace MyBRep
{

class Shape : public Instance
{
public:
    Shape();
    explicit Shape(const Topology_Shape& topology);
    Shape(const Topology_Shape& topology, const MyMath::Matrix4& localToWorld);

    Shape(const Shape&) = default;
    Shape& operator=(const Shape&) = default;

    /// Topology与Geometry

    Topology_Shape topology() const;

    bool sharesGeometryWith(const Shape& other) const;

    const Geometry_Shape& geometry() const;
    const Geometry_Shape* geometryPointer() const;
    ShapeKind kind() const;

    /// 空间范围

    const Bounds3& localBounds() const;
    const Bounds3& worldBounds() const;

    /// 局部空间查询

    bool containsLocalPoint(const MyMath::Vector3& point) const;
    ShapeRelation classifyLocalBounds(const Bounds3& bounds) const;
    ShapeRelation classifyLocalBoundsFast(const MyMath::Vector3& center, const MyMath::Vector3& extent) const;

    /// 世界空间查询

    bool containsWorldPoint(const MyMath::Vector3& point) const;
    ShapeRelation classifyWorldBounds(const Bounds3& bounds) const;

    /// 方向操作

    Shape reversed() const;

protected:
    void onInstanceChanged() override;

private:
    void updateWorldBounds();

private:
    Bounds3 m_worldBounds;
};

}

#endif // MYBREP_INSTANCE_SHAPE_H
