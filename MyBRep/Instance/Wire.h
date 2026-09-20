#ifndef MYBREP_INSTANCE_WIRE_H
#define MYBREP_INSTANCE_WIRE_H

#include <cstddef>
#include <vector>

#include "MyMath/Vector3.h"
#include "MyBRep/Instance/Edge.h"
#include "MyBRep/Instance/Instance.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

namespace MyBRep
{

class Wire : public Instance
{
public:
    Wire();
    explicit Wire(const Topology_Wire& topology);
    Wire(const Topology_Wire& topology, const MyMath::Matrix4& localToWorld);

    Wire(const Wire&) = default;
    Wire& operator=(const Wire&) = default;

    /// Topology

    Topology_Wire topology() const;

    bool isClosed() const;

    std::size_t edgeCount() const;
    Topology_Edge topologyEdge(std::size_t index) const;
    std::vector<Topology_Edge> topologyEdges() const;

    /// 子实例

    Edge edge(std::size_t index) const;

    /// 有向端点

    MyMath::Vector3 localStartPoint() const;
    MyMath::Vector3 localEndPoint() const;
    MyMath::Vector3 worldStartPoint() const;
    MyMath::Vector3 worldEndPoint() const;

    /// 方向操作

    Wire reversed() const;
};

}

#endif // MYBREP_INSTANCE_WIRE_H
