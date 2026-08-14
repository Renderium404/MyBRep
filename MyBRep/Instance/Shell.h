#ifndef MYBREP_INSTANCE_SHELL_H
#define MYBREP_INSTANCE_SHELL_H

#include <cstddef>
#include <vector>

#include "MyMath/Matrix4.h"
#include "MyBRep/Instance/Face.h"
#include "MyBRep/Instance/Instance_Object.h"
#include "MyBRep/Topology/Face/Topology_Face.h"
#include "MyBRep/Topology/Shell/Topology_Shell.h"

namespace MyBRep
{

// 表示Topology_Shell在世界空间中的一次不可变放置。
class Shell : public Instance_Object
{
public:
    // 构造不包含局部Topology_Shell的空实例。
    Shell();
    // 使用单位变换放置局部Topology_Shell。
    explicit Shell(const Topology_Shell& topology);
    // 使用指定可逆仿射变换放置局部Topology_Shell。
    Shell(const Topology_Shell& topology, const MyMath::Matrix4& localToWorld);
    Shell(const Shell&) = default;
    Shell& operator=(const Shell&) = default;

    /// 状态判断

    // 判断当前Shell是否包含有效局部拓扑壳。
    bool isValid() const;
    // 判断当前Shell是否未包含局部拓扑壳。
    bool isNull() const;
    // 判断当前Shell是否包含有效局部拓扑壳。
    explicit operator bool() const;
    // 判断当前Shell拓扑是否闭合。
    bool isClosed() const;

    /// 局部拓扑

    // 返回当前Shell持有的局部Topology_Shell。
    const Topology_Shell& topology() const;
    // 返回当前Shell使用方向下的Face数量，空实例返回零。
    std::size_t faceCount() const;
    // 返回当前Shell使用方向下指定位置的局部Topology_Face。
    Topology_Face topologyFace(std::size_t index) const;
    // 返回当前Shell使用方向下的全部局部Topology_Face。
    std::vector<Topology_Face> topologyFaces() const;

    /// 子实例

    // 返回指定Topology_Face在当前Shell空间放置下对应的Face实例。
    Face face(std::size_t index) const;

    /// 方向操作

    // 返回共享同一Topology_TShell身份、保持同一空间放置但整体方向相反的新Shell实例。
    Shell reversed() const;

private:
    Topology_Shell m_topology; // 当前Shell实例持有的局部拓扑壳。
};

}

#endif // MYBREP_INSTANCE_SHELL_H
