#include "Topology_TObject.h"

#include <atomic>

#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

Topology_TObject::Topology_TObject()
    : m_id(allocateId())
{
}

Topology_TObject::~Topology_TObject()
{
}

/// 拓扑身份

TopologyId Topology_TObject::allocateId()
{
    static std::atomic<TopologyId> nextId(1);

    const TopologyId id = nextId.fetch_add(1, std::memory_order_relaxed);
    MYBREP_ASSERT_MESSAGE(id != InvalidTopologyId, "TopologyId range exhausted.");
    return id;
}

}
