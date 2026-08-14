#ifndef MYBREP_TOPOLOGY_SOLID_TOPOLOGY_SOLID_H
#define MYBREP_TOPOLOGY_SOLID_TOPOLOGY_SOLID_H

#include <cstddef>
#include <vector>

#include "MyBRep/Topology/Shell/Topology_Shell.h"
#include "MyBRep/Topology/Topology_Object.h"
#include "Topology_TSolid.h"

namespace MyBRep
{

// 表示由完整子拓扑构成的三维B-Rep实体轻量句柄，共享Topology_TSolid身份并保存整体使用方向。
//
// Solid由一个或多个闭合Shell组成；Reversed Solid不改变Shell集合顺序，只翻转每个Shell的使用方向。
// Topology_Solid与直接持有Geometry_Shape的Topology_Shape职责不同，当前类型始终代表完整边界拓扑实体。
class Topology_Solid : public Topology_Object
{
public:
    // 构造不引用任何共享Topology_TSolid实体的空句柄。
    Topology_Solid();
    // 使用至少一个有效闭合Shell创建新的Forward完整B-Rep实体身份。
    explicit Topology_Solid(const std::vector<Topology_Shell>& shells);
    // 使用一个有效闭合Shell创建新的Forward完整B-Rep实体身份。
    explicit Topology_Solid(const Topology_Shell& shell);
    Topology_Solid(const Topology_Solid&) = default;
    Topology_Solid& operator=(const Topology_Solid&) = default;

    /// 有向Shell集合

    // 返回当前Solid包含的Shell数量，空句柄返回零。
    std::size_t shellCount() const;
    // 返回当前Solid使用方向下指定位置的Topology_Shell，Reversed Solid会翻转对应Shell方向。
    Topology_Shell shell(std::size_t index) const;
    // 返回当前Solid使用方向下的全部有向闭合Topology_Shell。
    std::vector<Topology_Shell> shells() const;

    /// 方向操作

    // 返回共享同一Topology_TSolid身份但整体方向相反的新句柄。
    Topology_Solid reversed() const;

private:
    // 使用已有共享Topology_TSolid实体和明确方向构造完整B-Rep实体句柄。
    Topology_Solid(const Foundation::RefPtr<Topology_TObject>& object,
                   Topology_Orientation orientation);

    // 返回当前句柄共享的强类型Topology_TSolid实体。
    const Topology_TSolid& tSolid() const;
};

}

#endif // MYBREP_TOPOLOGY_SOLID_TOPOLOGY_SOLID_H
