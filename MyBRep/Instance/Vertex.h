#ifndef MYBREP_INSTANCE_VERTEX_H
#define MYBREP_INSTANCE_VERTEX_H

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyBRep/Instance/Instance_Object.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"

namespace MyBRep
{

// 表示Topology_Vertex在世界空间中的一次不可变放置。
class Vertex : public Instance_Object
{
public:
    // 构造不包含局部Topology_Vertex的空实例。
    Vertex();
    // 使用单位变换放置局部Topology_Vertex。
    explicit Vertex(const Topology_Vertex& topology);
    // 使用指定可逆仿射变换放置局部Topology_Vertex。
    Vertex(const Topology_Vertex& topology, const MyMath::Matrix4& localToWorld);
    Vertex(const Vertex&) = default;
    Vertex& operator=(const Vertex&) = default;

    /// 状态判断

    // 判断当前Vertex是否包含有效局部拓扑点。
    bool isValid() const;
    // 判断当前Vertex是否未包含局部拓扑点。
    bool isNull() const;
    // 判断当前Vertex是否包含有效局部拓扑点。
    explicit operator bool() const;

    /// 局部拓扑

    // 返回当前Vertex持有的局部Topology_Vertex。
    const Topology_Vertex& topology() const;

    /// 空间查询

    // 返回当前拓扑点的局部坐标。
    const MyMath::Vector3& localPoint() const;
    // 返回当前拓扑点的世界坐标。
    MyMath::Vector3 worldPoint() const;

private:
    Topology_Vertex m_topology; // 当前Vertex实例持有的局部拓扑点。
};

}

#endif // MYBREP_INSTANCE_VERTEX_H
