#ifndef MYBREP_INSTANCE_SOLID_H
#define MYBREP_INSTANCE_SOLID_H

#include <cstddef>
#include <vector>

#include "MyMath/Matrix4.h"
#include "MyBRep/Instance/Instance_Object.h"
#include "MyBRep/Instance/Shell.h"
#include "MyBRep/Topology/Shell/Topology_Shell.h"
#include "MyBRep/Topology/Solid/Topology_Solid.h"

namespace MyBRep
{

// 表示完整B-Rep Topology_Solid在世界空间中的一次不可变放置。
//
// Solid实例只负责共享完整子拓扑并施加空间放置，不额外建立Geometry_Shape连续体内核。
class Solid : public Instance_Object
{
public:
    // 构造不包含局部Topology_Solid的空实例。
    Solid();
    // 使用单位变换放置局部Topology_Solid。
    explicit Solid(const Topology_Solid& topology);
    // 使用指定可逆仿射变换放置局部Topology_Solid。
    Solid(const Topology_Solid& topology, const MyMath::Matrix4& localToWorld);
    Solid(const Solid&) = default;
    Solid& operator=(const Solid&) = default;

    /// 状态判断

    // 判断当前Solid是否包含有效局部完整B-Rep拓扑。
    bool isValid() const;
    // 判断当前Solid是否未包含局部Topology_Solid。
    bool isNull() const;
    // 判断当前Solid是否包含有效局部完整B-Rep拓扑。
    explicit operator bool() const;

    /// 局部拓扑

    // 返回当前Solid持有的局部Topology_Solid。
    const Topology_Solid& topology() const;
    // 返回当前Solid使用方向下的Shell数量，空实例返回零。
    std::size_t shellCount() const;
    // 返回当前Solid使用方向下指定位置的局部Topology_Shell。
    Topology_Shell topologyShell(std::size_t index) const;
    // 返回当前Solid使用方向下的全部局部Topology_Shell。
    std::vector<Topology_Shell> topologyShells() const;

    /// 子实例

    // 返回指定Topology_Shell在当前Solid空间放置下对应的Shell实例。
    Shell shell(std::size_t index) const;

    /// 方向操作

    // 返回共享同一Topology_TSolid身份、保持同一空间放置但整体方向相反的新Solid实例。
    Solid reversed() const;

private:
    Topology_Solid m_topology; // 当前Solid实例持有的局部完整B-Rep拓扑。
};

}

#endif // MYBREP_INSTANCE_SOLID_H
