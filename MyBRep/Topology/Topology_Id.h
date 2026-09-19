#ifndef MYBREP_TOPOLOGY_TOPOLOGY_ID_H
#define MYBREP_TOPOLOGY_TOPOLOGY_ID_H

#include <cstdint>

namespace MyBRep
{

// MyBRep共享拓扑实体唯一标识类型。
// TopologyId只标识Topology_TObject实体身份，不包含Topology_Orientation。
typedef std::uint64_t TopologyId;

// 无效拓扑实体ID。
const TopologyId InvalidTopologyId = static_cast<TopologyId>(0);

}

#endif // MYBREP_TOPOLOGY_TOPOLOGY_ID_H
