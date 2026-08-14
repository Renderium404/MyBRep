#ifndef MYBREP_TOPOLOGY_SHAPE_TOPOLOGY_SHAPE_H
#define MYBREP_TOPOLOGY_SHAPE_TOPOLOGY_SHAPE_H

#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Shape/Geometry_Shape.h"
#include "MyBRep/Topology/Topology_Object.h"
#include "Topology_TShape.h"

namespace MyBRep
{

// 作为连续Geometry_Shape几何内核的轻量拓扑形体句柄，不建立完整B-Rep边界子拓扑。
//
// Topology_Shape与Topology_Solid职责明确区分：Topology_Shape直接持有Geometry_Shape，
// Topology_Solid则由Shell、Face、Wire、Edge和Vertex等完整子拓扑组成。
class Topology_Shape : public Topology_Object
{
public:
    // 构造不引用任何共享Topology_TShape实体的空句柄。
    Topology_Shape();
    // 使用指定连续实体几何内核创建新的Forward拓扑形体身份。
    explicit Topology_Shape(const Foundation::RefPtr<const Geometry_Shape>& geometry);
    Topology_Shape(const Topology_Shape&) = default;
    Topology_Shape& operator=(const Topology_Shape&) = default;

    /// 几何内核

    // 返回当前拓扑形体对应的连续实体几何内核。
    const Geometry_Shape& geometry() const;
    // 返回当前拓扑形体共享的不可变连续实体几何内核资源。
    const Foundation::RefPtr<const Geometry_Shape>& geometryResource() const;

    /// 方向操作

    // 返回共享同一Topology_TShape身份但整体使用方向相反的新句柄，Geometry_Shape资源本身不发生修改。
    Topology_Shape reversed() const;

private:
    // 使用已有共享Topology_TShape实体和明确方向构造拓扑形体句柄。
    Topology_Shape(const Foundation::RefPtr<Topology_TObject>& object,
                   Topology_Orientation orientation);

    // 返回当前句柄引用的强类型共享拓扑形体实体。
    const Topology_TShape& tShape() const;
};

}

#endif // MYBREP_TOPOLOGY_SHAPE_TOPOLOGY_SHAPE_H