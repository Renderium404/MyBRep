#ifndef MYBREP_INSTANCE_FACE_H
#define MYBREP_INSTANCE_FACE_H

#include <cstddef>
#include <vector>

#include "MyMath/Matrix4.h"
#include "MyBRep/Geometry/Surface/Geometry_Surface.h"
#include "MyBRep/Instance/Instance_Object.h"
#include "MyBRep/Instance/Wire.h"
#include "MyBRep/Topology/Face/Topology_Face.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

namespace MyBRep
{

// 表示Topology_Face在世界空间中的一次不可变放置。
//
// Face实例不复制曲面或P-Curve，所有局部B-Rep几何关系继续由共享Topology_Face和Topology_Edge提供。
// 当前核心实例层不缓存裁剪Face包围盒，有限Face空间范围由后续B-Rep查询层统一提供。
class Face : public Instance_Object
{
public:
    // 构造不包含局部Topology_Face的空实例。
    Face();
    // 使用单位变换放置局部Topology_Face。
    explicit Face(const Topology_Face& topology);
    // 使用指定可逆仿射变换放置局部Topology_Face。
    Face(const Topology_Face& topology, const MyMath::Matrix4& localToWorld);
    Face(const Face&) = default;
    Face& operator=(const Face&) = default;

    /// 状态判断

    // 判断当前Face是否包含有效局部拓扑面。
    bool isValid() const;
    // 判断当前Face是否未包含局部拓扑面。
    bool isNull() const;
    // 判断当前Face是否包含有效局部拓扑面。
    explicit operator bool() const;
    // 判断当前Face是否与另一个Face引用同一个Topology_TFace身份。
    bool sharesTopologyWith(const Face& other) const;
    // 判断当前Face是否与另一个Face引用同一个完整Geometry_Surface资源。
    bool sharesGeometryWith(const Face& other) const;

    /// 局部拓扑与曲面几何

    // 返回当前Face持有的局部Topology_Face。
    const Topology_Face& topology() const;
    // 返回当前Face引用的完整不可变参数曲面。
    const Geometry_Surface& geometry() const;
    // 返回当前Face引用的完整不可变参数曲面普通指针，空实例返回空指针。
    const Geometry_Surface* geometryPointer() const;
    // 返回当前Face引用的标准曲面类型。
    SurfaceKind kind() const;

    /// 裁剪Wire

    // 返回当前Face使用方向下的裁剪Wire数量，空实例返回零。
    std::size_t wireCount() const;
    // 返回当前Face使用方向下指定位置的局部Topology_Wire。
    Topology_Wire topologyWire(std::size_t index) const;
    // 返回当前Face使用方向下的全部局部Topology_Wire。
    std::vector<Topology_Wire> topologyWires() const;
    // 返回指定裁剪Wire在当前Face空间放置下对应的Wire实例。
    Wire wire(std::size_t index) const;

    /// 方向操作

    // 返回共享同一Topology_TFace身份、保持同一空间放置但使用方向相反的新Face实例。
    Face reversed() const;

private:
    Topology_Face m_topology; // 当前Face实例持有的局部拓扑面。
};

}

#endif // MYBREP_INSTANCE_FACE_H
