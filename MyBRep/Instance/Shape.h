#ifndef MYBREP_INSTANCE_SHAPE_H
#define MYBREP_INSTANCE_SHAPE_H

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyBRep/Base/Bounds3.h"
#include "MyBRep/Geometry/Shape/Geometry_Shape.h"
#include "MyBRep/Instance/Instance_Object.h"
#include "MyBRep/Topology/Shape/Topology_Shape.h"

namespace MyBRep
{

// 表示Topology_Shape连续体内核在世界空间中的一次不可变放置，并缓存对应的世界轴对齐包围盒。
//
// Shape与Solid明确区分：Shape直接包装Geometry_Shape连续体，Solid则实例化完整B-Rep Topology_Solid。
class Shape : public Instance_Object
{
public:
    // 构造不包含局部Topology_Shape的空实例。
    Shape();
    // 使用单位变换放置局部Topology_Shape。
    explicit Shape(const Topology_Shape& topology);
    // 使用指定可逆仿射变换放置局部Topology_Shape。
    Shape(const Topology_Shape& topology, const MyMath::Matrix4& localToWorld);
    Shape(const Shape&) = default;
    Shape& operator=(const Shape&) = default;

    /// 状态判断

    // 判断当前Shape是否包含有效局部Topology_Shape和有效世界包围盒。
    bool isValid() const;
    // 判断当前Shape是否未包含局部Topology_Shape。
    bool isNull() const;
    // 判断当前Shape是否包含完整有效数据。
    explicit operator bool() const;
    // 判断当前Shape是否与另一个Shape引用同一个Topology_TShape身份。
    bool sharesTopologyWith(const Shape& other) const;
    // 判断当前Shape是否与另一个Shape引用同一个Geometry_Shape资源。
    bool sharesGeometryWith(const Shape& other) const;

    /// 局部拓扑与几何内核

    // 返回当前Shape持有的局部Topology_Shape。
    const Topology_Shape& topology() const;
    // 返回当前Shape直接引用的不可变连续实体几何内核。
    const Geometry_Shape& geometry() const;
    // 返回当前Shape直接引用的不可变连续实体几何普通指针，空实例返回空指针。
    const Geometry_Shape* geometryPointer() const;
    // 返回当前Shape的标准连续体几何类型。
    ShapeKind kind() const;

    /// 空间范围

    // 返回当前Shape在局部坐标系中的有限轴对齐包围盒。
    const Bounds3& localBounds() const;
    // 返回当前Shape在世界坐标系中的有限轴对齐包围盒。
    const Bounds3& worldBounds() const;

    /// 局部空间查询

    // 判断指定局部坐标点是否位于Shape内部或边界上。
    bool containsLocalPoint(const MyMath::Vector3& point) const;
    // 返回指定局部轴对齐包围盒与Shape之间的保守空间关系。
    ShapeRelation classifyLocalBounds(const Bounds3& bounds) const;
    // 使用已经计算好的局部包围盒中心和半尺寸执行保守分类。
    ShapeRelation classifyLocalBoundsFast(const MyMath::Vector3& center,
                                          const MyMath::Vector3& extent) const;

    /// 世界空间查询

    // 判断指定世界坐标点是否位于Shape内部或边界上。
    bool containsWorldPoint(const MyMath::Vector3& point) const;
    // 返回指定世界轴对齐包围盒与Shape之间的保守空间关系。
    ShapeRelation classifyWorldBounds(const Bounds3& bounds) const;

    /// 方向操作

    // 返回共享同一Topology_TShape身份、保持同一空间放置但整体方向相反的新Shape实例。
    Shape reversed() const;

private:
    // 根据当前Topology_Shape和空间放置建立世界轴对齐包围盒。
    void initialize();

private:
    Topology_Shape m_topology; // 当前Shape实例持有的局部Topology_Shape。
    Bounds3 m_worldBounds; // 当前Shape在世界坐标系中的有限轴对齐包围盒。
};

}

#endif // MYBREP_INSTANCE_SHAPE_H
