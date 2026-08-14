#ifndef MYBREP_TOPOLOGY_SHELL_TOPOLOGY_TSHELL_H
#define MYBREP_TOPOLOGY_SHELL_TOPOLOGY_TSHELL_H

#include <cstddef>
#include <vector>

#include "MyBRep/Topology/Face/Topology_Face.h"
#include "MyBRep/Topology/Topology_TObject.h"

namespace MyBRep
{

class Topology_Shell;

// 保存一个共享拓扑壳身份以及规范Forward方向下的连通有向Face集合。
//
// Topology_TShell允许开放或闭合；闭合状态由共享Edge在全部Face边界中的使用次数和方向自动确定。
// 对于流形Shell，同一Topology_TEdge最多使用两次；使用两次时两个Edge句柄必须方向相反。
class Topology_TShell : public Topology_TObject
{
    friend class Topology_Shell;

public:
    /// Face集合

    // 返回规范Forward Shell包含的Face数量。
    std::size_t faceCount() const;
    // 返回规范Forward Shell指定位置的有向Topology_Face。
    const Topology_Face& face(std::size_t index) const;
    // 返回规范Forward Shell保存的全部有向Topology_Face。
    const std::vector<Topology_Face>& faces() const;

    /// 拓扑状态

    // 判断当前共享Shell是否形成无边界的闭合二维流形。
    bool isClosed() const;

protected:
    // 使用至少一个按共享Edge身份连通的有效Face创建共享Shell实体。
    explicit Topology_TShell(const std::vector<Topology_Face>& faces);
    // 通过Topology_Shell持有的最终共享引用释放拓扑壳实体。
    ~Topology_TShell() override = default;

private:
    std::vector<Topology_Face> m_faces; // 规范Forward Shell保存的连通有向Face集合。
    bool m_closed; // 全部Edge是否恰好以一正一反两次使用并形成无边界流形。
};

}

#endif // MYBREP_TOPOLOGY_SHELL_TOPOLOGY_TSHELL_H
