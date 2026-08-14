#ifndef MYBREP_TOPOLOGY_TOPOLOGY_ORIENTATION_H
#define MYBREP_TOPOLOGY_TOPOLOGY_ORIENTATION_H

namespace MyBRep
{

// 表示拓扑句柄相对于底层共享拓扑实体的使用方向。
enum class Topology_Orientation
{
    Forward, // 按底层共享拓扑实体的标准方向使用。
    Reversed // 按底层共享拓扑实体的相反方向使用。
};

// 返回与指定拓扑使用方向相反的方向。
Topology_Orientation oppositeOrientation(Topology_Orientation orientation);

}

#endif // MYBREP_TOPOLOGY_TOPOLOGY_ORIENTATION_H
