#ifndef MYBREP_TOPOLOGY_SOLID_TOPOLOGY_TSOLID_H
#define MYBREP_TOPOLOGY_SOLID_TOPOLOGY_TSOLID_H

#include <cstddef>
#include <vector>

#include "MyBRep/Topology/Shell/Topology_Shell.h"
#include "MyBRep/Topology/Topology_TObject.h"

namespace MyBRep
{

class Topology_Solid;

// 保存一个共享完整B-Rep实体身份以及规范Forward方向下的闭合Shell集合。
//
// Topology_TSolid不区分Outer/Inner标签；外边界与内腔由各Shell自身方向表达。
// 构造阶段只验证拓扑闭合与身份唯一性，不在Topology层执行Shell之间的几何包含关系分类。
class Topology_TSolid : public Topology_TObject
{
    friend class Topology_Solid;

public:
    /// Shell集合

    // 返回规范Forward Solid包含的Shell数量。
    std::size_t shellCount() const;
    // 返回规范Forward Solid指定位置的有向闭合Topology_Shell。
    const Topology_Shell& shell(std::size_t index) const;
    // 返回规范Forward Solid保存的全部有向闭合Topology_Shell。
    const std::vector<Topology_Shell>& shells() const;

protected:
    // 使用至少一个有效闭合Shell创建共享完整B-Rep实体。
    explicit Topology_TSolid(const std::vector<Topology_Shell>& shells);
    // 通过Topology_Solid持有的最终共享引用释放拓扑实体。
    ~Topology_TSolid() override = default;

private:
    std::vector<Topology_Shell> m_shells; // 规范Forward Solid保存的闭合有向Shell集合。
};

}

#endif // MYBREP_TOPOLOGY_SOLID_TOPOLOGY_TSOLID_H
