#ifndef MYBREP_TOPOLOGY_TOPOLOGY_TOBJECT_H
#define MYBREP_TOPOLOGY_TOPOLOGY_TOBJECT_H

#include "MyBRep/Foundation/ReferenceCounted.h"

namespace MyBRep
{

// 作为全部共享拓扑实体的内部生命周期基类，具体Topology_TObject对象本身唯一确定拓扑身份。
class Topology_TObject : public Foundation::ReferenceCounted
{
protected:
    // 构造引用计数为零的共享拓扑实体。
    Topology_TObject();
    // 通过公开Topology_Object句柄持有的最终共享引用释放具体拓扑实体。
    ~Topology_TObject() override;
};

}

#endif // MYBREP_TOPOLOGY_TOPOLOGY_TOBJECT_H
