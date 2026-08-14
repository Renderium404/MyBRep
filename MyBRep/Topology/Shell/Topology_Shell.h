#ifndef MYBREP_TOPOLOGY_SHELL_TOPOLOGY_SHELL_H
#define MYBREP_TOPOLOGY_SHELL_TOPOLOGY_SHELL_H

#include <cstddef>
#include <vector>

#include "MyBRep/Topology/Face/Topology_Face.h"
#include "MyBRep/Topology/Topology_Object.h"
#include "Topology_TShell.h"

namespace MyBRep
{

// 表示连通二维拓扑壳的轻量句柄，共享Topology_TShell身份并通过Topology_Object方向决定全部Face的使用方向。
//
// Shell允许开放或闭合；Reversed Shell不改变Face集合顺序，只翻转每个Face的使用方向。
class Topology_Shell : public Topology_Object
{
public:
    // 构造不引用任何共享拓扑Shell实体的空句柄。
    Topology_Shell();
    // 使用至少一个通过共享Topology_TEdge身份连通的有向Face创建新的Forward Shell身份。
    explicit Topology_Shell(const std::vector<Topology_Face>& faces);
    Topology_Shell(const Topology_Shell&) = default;
    Topology_Shell& operator=(const Topology_Shell&) = default;

    /// 拓扑状态

    // 判断当前Shell是否形成无边界的闭合二维流形。
    bool isClosed() const;

    /// 有向Face集合

    // 返回当前Shell包含的Face数量，空句柄返回零。
    std::size_t faceCount() const;
    // 返回当前Shell使用方向下指定位置的Topology_Face，Reversed Shell会翻转对应Face方向。
    Topology_Face face(std::size_t index) const;
    // 返回当前Shell使用方向下的全部有向Topology_Face。
    std::vector<Topology_Face> faces() const;

    /// 方向操作

    // 返回共享同一Topology_TShell身份但整体方向相反的新句柄。
    Topology_Shell reversed() const;

private:
    // 使用已有共享Topology_TShell实体和明确方向构造拓扑壳句柄。
    Topology_Shell(const Foundation::RefPtr<Topology_TObject>& object,
                   Topology_Orientation orientation);

    // 返回当前句柄共享的强类型Topology_TShell实体。
    const Topology_TShell& tShell() const;
};

}

#endif // MYBREP_TOPOLOGY_SHELL_TOPOLOGY_SHELL_H
