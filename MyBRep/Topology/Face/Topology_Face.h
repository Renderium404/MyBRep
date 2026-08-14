#ifndef MYBREP_TOPOLOGY_FACE_TOPOLOGY_FACE_H
#define MYBREP_TOPOLOGY_FACE_TOPOLOGY_FACE_H

#include <cstddef>
#include <vector>

#include "MyMath/Vector3.h"
#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Surface/Geometry_Surface.h"
#include "MyBRep/Topology/Topology_Object.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"
#include "Topology_TFace.h"

namespace MyBRep
{

// 作为共享拓扑面实体的轻量公开句柄，使用方向决定Face法向和裁剪Wire的整体遍历方向。
//
// Forward Face采用Geometry_Surface自身的参数法向；Reversed Face取相反法向并反转每个裁剪Wire。
// Face只保存完整Surface与Wire拓扑，Edge在Surface上的P-Curve统一由共享Topology_TEdge管理。
class Topology_Face : public Topology_Object
{
public:
    // 构造空拓扑面句柄。
    Topology_Face();

    // 使用完整参数曲面和零个或多个闭合裁剪Wire创建新的Forward拓扑面身份。
    Topology_Face(const Foundation::RefPtr<const Geometry_Surface>& geometry,
                  const std::vector<Topology_Wire>& wires);

    // 使用完整参数曲面创建不含显式裁剪Wire的Forward拓扑面身份。
    explicit Topology_Face(const Foundation::RefPtr<const Geometry_Surface>& geometry);

    /// 曲面几何

    // 返回当前Face引用的完整不可变参数曲面。
    const Geometry_Surface& geometry() const;
    // 返回当前Face引用的完整不可变参数曲面资源。
    const Foundation::RefPtr<const Geometry_Surface>& geometryResource() const;
    // 返回当前Face使用方向在指定Surface参数处对应的单位法向。
    MyMath::Vector3 normalAt(double u, double v) const;

    /// 裁剪Wire

    // 返回当前Face包含的裁剪Wire数量，空句柄返回零。
    std::size_t wireCount() const;
    // 返回当前Face使用方向下指定位置的闭合Wire，Reversed Face返回对应Wire的反向句柄。
    Topology_Wire wire(std::size_t index) const;
    // 返回当前Face使用方向下的全部闭合裁剪Wire。
    std::vector<Topology_Wire> wires() const;

    /// 方向操作

    // 返回共享同一Topology_TFace身份但使用方向相反的新句柄。
    Topology_Face reversed() const;

private:
    // 使用已有共享Topology_TFace实体和明确方向构造拓扑面句柄。
    Topology_Face(const Foundation::RefPtr<Topology_TObject>& object,
                  Topology_Orientation orientation);

    // 返回当前句柄引用的强类型共享拓扑面实体。
    const Topology_TFace& tFace() const;
};

}

#endif // MYBREP_TOPOLOGY_FACE_TOPOLOGY_FACE_H
