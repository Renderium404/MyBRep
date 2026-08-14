#ifndef MYBREP_INSTANCE_WIRE_H
#define MYBREP_INSTANCE_WIRE_H

#include <cstddef>
#include <vector>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyBRep/Instance/Edge.h"
#include "MyBRep/Instance/Instance_Object.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

namespace MyBRep
{

// 表示Topology_Wire在世界空间中的一次不可变放置。
class Wire : public Instance_Object
{
public:
    // 构造不包含局部Topology_Wire的空实例。
    Wire();
    // 使用单位变换放置局部Topology_Wire。
    explicit Wire(const Topology_Wire& topology);
    // 使用指定可逆仿射变换放置局部Topology_Wire。
    Wire(const Topology_Wire& topology, const MyMath::Matrix4& localToWorld);
    Wire(const Wire&) = default;
    Wire& operator=(const Wire&) = default;

    /// 状态判断

    // 判断当前Wire是否包含有效局部拓扑。
    bool isValid() const;
    // 判断当前Wire是否未包含局部拓扑。
    bool isNull() const;
    // 判断当前Wire是否包含有效局部拓扑。
    explicit operator bool() const;
    // 判断当前Wire拓扑是否首尾闭合。
    bool isClosed() const;

    /// 局部拓扑

    // 返回当前Wire持有的局部Topology_Wire。
    const Topology_Wire& topology() const;
    // 返回当前Wire包含的Edge数量，空实例返回零。
    std::size_t edgeCount() const;
    // 返回当前Wire遍历方向下指定位置的局部Topology_Edge。
    Topology_Edge topologyEdge(std::size_t index) const;
    // 返回当前Wire遍历方向下的全部局部Topology_Edge。
    std::vector<Topology_Edge> topologyEdges() const;

    /// 子实例

    // 返回指定Topology_Edge在当前Wire空间放置下对应的Edge实例。
    Edge edge(std::size_t index) const;

    /// 有向端点

    // 返回当前Wire遍历方向下的局部起点。
    MyMath::Vector3 localStartPoint() const;
    // 返回当前Wire遍历方向下的局部终点。
    MyMath::Vector3 localEndPoint() const;
    // 返回当前Wire遍历方向下的世界起点。
    MyMath::Vector3 worldStartPoint() const;
    // 返回当前Wire遍历方向下的世界终点。
    MyMath::Vector3 worldEndPoint() const;

    /// 方向操作

    // 返回共享同一Topology_TWire身份、保持同一空间放置但遍历方向相反的新Wire实例。
    Wire reversed() const;

private:
    Topology_Wire m_topology; // 当前Wire实例持有的局部拓扑实体。
};

}

#endif // MYBREP_INSTANCE_WIRE_H
