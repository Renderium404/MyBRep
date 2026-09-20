#ifndef MYBREP_TOOL_TOPOLOGYCOLLECTOR_H
#define MYBREP_TOOL_TOPOLOGYCOLLECTOR_H

#include <vector>

#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Face/Topology_Face.h"
#include "MyBRep/Topology/Shell/Topology_Shell.h"
#include "MyBRep/Topology/Solid/Topology_Solid.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

namespace MyBRep
{
namespace Tool
{

// 从一个B-Rep拓扑对象中收集出的唯一Face和Edge实体。
// 去重只比较共享拓扑身份，Topology_Orientation不参与唯一性判断。
// 当同一拓扑实体以不同方向重复出现时，保留首次遍历到的有向句柄。
struct TopologyCollection
{
    bool empty() const { return faces.empty() && edges.empty(); }
    void clear() { faces.clear(); edges.clear(); }

    std::vector<Topology_Face> faces;
    std::vector<Topology_Edge> edges;
};

// 收集一个拓扑对象包含的唯一Face和Edge实体。
// 本类只负责拓扑遍历和身份去重，不进行几何查询、实例变换或显示构建。
class TopologyCollector
{
public:
    static TopologyCollection collect(const Topology_Edge& edge);
    static TopologyCollection collect(const Topology_Wire& wire);
    static TopologyCollection collect(const Topology_Face& face);
    static TopologyCollection collect(const Topology_Shell& shell);
    static TopologyCollection collect(const Topology_Solid& solid);
};

}
}

#endif // MYBREP_TOOL_TOPOLOGYCOLLECTOR_H