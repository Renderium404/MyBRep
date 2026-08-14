#ifndef MYBREP_INSTANCE_EDGE_H
#define MYBREP_INSTANCE_EDGE_H

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyBRep/Geometry/Curve/Geometry_Curve.h"
#include "MyBRep/Instance/Instance_Object.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"

namespace MyBRep
{

// 表示有限Topology_Edge在世界空间中的一次不可变放置。
//
// Edge实例引用完整Geometry_Curve资源，但有限范围始终由Topology_Edge参数区间决定。
// 当前核心实例层不缓存Edge包围盒或弧长，这些有限几何查询由后续B-Rep查询层统一提供。
class Edge : public Instance_Object
{
public:
    // 构造不包含局部Topology_Edge的空实例。
    Edge();
    // 使用单位变换放置局部Topology_Edge。
    explicit Edge(const Topology_Edge& topology);
    // 使用指定可逆仿射变换放置局部Topology_Edge。
    Edge(const Topology_Edge& topology, const MyMath::Matrix4& localToWorld);
    Edge(const Edge&) = default;
    Edge& operator=(const Edge&) = default;

    /// 状态判断

    // 判断当前Edge是否包含有效局部拓扑边。
    bool isValid() const;
    // 判断当前Edge是否未包含局部拓扑边。
    bool isNull() const;
    // 判断当前Edge是否包含有效局部拓扑边。
    explicit operator bool() const;
    // 判断当前Edge是否与另一个Edge引用同一个Topology_TEdge身份。
    bool sharesTopologyWith(const Edge& other) const;
    // 判断当前Edge是否与另一个Edge引用同一个完整Geometry_Curve资源。
    bool sharesGeometryWith(const Edge& other) const;

    /// 局部拓扑与几何资源

    // 返回当前Edge持有的局部Topology_Edge。
    const Topology_Edge& topology() const;
    // 返回当前Edge引用的完整不可变三维参数曲线。
    const Geometry_Curve& geometry() const;
    // 返回当前Edge引用的完整不可变三维参数曲线普通指针，空实例返回空指针。
    const Geometry_Curve* geometryPointer() const;
    // 返回当前Edge引用的标准曲线几何类型。
    CurveKind kind() const;

    /// 有向端点

    // 返回当前拓扑使用方向中的局部起点。
    MyMath::Vector3 localStartPoint() const;
    // 返回当前拓扑使用方向中的局部终点。
    MyMath::Vector3 localEndPoint() const;
    // 返回当前拓扑使用方向中的世界起点。
    MyMath::Vector3 worldStartPoint() const;
    // 返回当前拓扑使用方向中的世界终点。
    MyMath::Vector3 worldEndPoint() const;

    /// 局部空间查询

    // 返回当前拓扑使用方向规范化参数[0,1]对应的局部Edge点。
    MyMath::Vector3 pointAt(double parameter) const;
    // 返回当前拓扑使用方向规范化参数[0,1]对应的局部单位切向量。
    MyMath::Vector3 tangentAt(double parameter) const;

    /// 世界空间查询

    // 返回当前拓扑使用方向规范化参数[0,1]对应的世界Edge点。
    MyMath::Vector3 worldPointAt(double parameter) const;
    // 返回当前拓扑使用方向规范化参数[0,1]对应的世界单位切向量。
    MyMath::Vector3 worldTangentAt(double parameter) const;

    /// 方向操作

    // 返回共享同一Topology_TEdge身份、保持同一空间放置但使用方向相反的新Edge实例。
    Edge reversed() const;

private:
    Topology_Edge m_topology; // 当前Edge实例持有的局部拓扑边。
};

}

#endif // MYBREP_INSTANCE_EDGE_H
