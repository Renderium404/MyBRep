#ifndef MYBREP_TOPOLOGY_FACE_TOPOLOGY_TFACE_H
#define MYBREP_TOPOLOGY_FACE_TOPOLOGY_TFACE_H

#include <cstddef>
#include <vector>

#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Surface/Geometry_Surface.h"
#include "MyBRep/Topology/Topology_TObject.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

namespace MyBRep
{

class Topology_Face;

// 保存一个共享拓扑面身份、完整底层参数曲面以及规范Forward方向下的闭合裁剪Wire序列。
//
// Topology_TFace不复制Edge的P-Curve；每条边在该Surface上的二维几何表示统一保存在共享Topology_TEdge中。
// wires允许为空，此时Face表示完整Surface的自然区域。
class Topology_TFace : public Topology_TObject
{
    friend class Topology_Face;

public:
    /// 曲面几何

    // 返回当前Face引用的完整不可变参数曲面。
    const Geometry_Surface& geometry() const;
    // 返回当前Face引用的完整不可变参数曲面资源。
    const Foundation::RefPtr<const Geometry_Surface>& geometryResource() const;

    /// 裁剪Wire

    // 返回规范Forward Face保存的闭合Wire数量。
    std::size_t wireCount() const;
    // 返回规范Forward Face指定位置的有向闭合Wire。
    const Topology_Wire& wire(std::size_t index) const;
    // 返回规范Forward Face保存的全部有向闭合Wire。
    const std::vector<Topology_Wire>& wires() const;

protected:
    // 使用完整Surface和零个或多个闭合裁剪Wire创建共享Face实体。
    Topology_TFace(const Foundation::RefPtr<const Geometry_Surface>& geometry,const std::vector<Topology_Wire>& wires);
    // 通过Topology_Face持有的最终共享引用释放拓扑面实体。
    ~Topology_TFace() override = default;
private:
    Foundation::RefPtr<const Geometry_Surface> m_geometry; // 当前Face引用的完整不可变参数曲面。
    std::vector<Topology_Wire> m_wires; // 规范Forward Face保存的全部有向闭合裁剪Wire。
};

}

#endif // MYBREP_TOPOLOGY_FACE_TOPOLOGY_TFACE_H
