#ifndef MYBREP_TOPOLOGY_SHAPE_TOPOLOGY_TSHAPE_H
#define MYBREP_TOPOLOGY_SHAPE_TOPOLOGY_TSHAPE_H

#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Shape/Geometry_Shape.h"
#include "MyBRep/Topology/Topology_TObject.h"

namespace MyBRep
{

class Topology_Shape;

// 保存一个共享拓扑形体身份及其连续Geometry_Shape几何内核。
//
// Topology_TShape不建立Vertex、Edge、Wire、Face、Shell或Solid子拓扑，完整连续体语义直接由Geometry_Shape提供。
class Topology_TShape : public Topology_TObject
{
    friend class Topology_Shape;

public:
    /// 几何内核

    // 返回当前拓扑形体对应的连续实体几何内核。
    const Geometry_Shape& geometry() const;
    // 返回当前拓扑形体共享的不可变连续实体几何内核资源。
    const Foundation::RefPtr<const Geometry_Shape>& geometryResource() const;

protected:
    // 使用指定非空连续实体几何内核创建共享拓扑形体实体。
    explicit Topology_TShape(const Foundation::RefPtr<const Geometry_Shape>& geometry);
    // 通过Topology_Shape持有的最终共享引用释放拓扑形体实体。
    ~Topology_TShape() override = default;

private:
    Foundation::RefPtr<const Geometry_Shape> m_geometry; // 当前拓扑形体对应的不可变连续实体几何内核。
};

}

#endif // MYBREP_TOPOLOGY_SHAPE_TOPOLOGY_TSHAPE_H