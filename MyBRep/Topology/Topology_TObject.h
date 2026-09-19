#ifndef MYBREP_TOPOLOGY_TOPOLOGY_TOBJECT_H
#define MYBREP_TOPOLOGY_TOPOLOGY_TOBJECT_H

#include "MyBRep/Foundation/ReferenceCounted.h"
#include "Topology_Id.h"

namespace MyBRep
{

// 作为全部共享拓扑实体的内部生命周期基类，具体Topology_TObject对象本身唯一确定拓扑身份。
// 每个新建Topology_TObject都会分配一个进程内唯一且非零的TopologyId；公开拓扑句柄共享该身份。
class Topology_TObject : public Foundation::ReferenceCounted
{
public:
    /// 拓扑身份

    // 返回当前共享拓扑实体稳定且非零的TopologyId。
    TopologyId id() const{return m_id;}

protected:
    // 构造引用计数为零的共享拓扑实体，并分配新的TopologyId。
    Topology_TObject();
    // 通过公开Topology_Object句柄持有的最终共享引用释放具体拓扑实体。
    ~Topology_TObject() override;

private:
    // 分配一个进程内唯一且非零的TopologyId。
    static TopologyId allocateId();

private:
    TopologyId m_id; // 当前共享拓扑实体稳定身份。
};

}

#endif // MYBREP_TOPOLOGY_TOPOLOGY_TOBJECT_H
