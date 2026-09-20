#ifndef MYBREP_INSTANCE_VERTEX_H
#define MYBREP_INSTANCE_VERTEX_H

#include "MyMath/Vector3.h"
#include "MyBRep/Instance/Instance.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"

namespace MyBRep
{

// 表示Topology_Vertex在世界空间中的一次实例。
class Vertex : public Instance
{
public:
    Vertex();
    explicit Vertex(const Topology_Vertex& topology);
    Vertex(const Topology_Vertex& topology, const MyMath::Matrix4& localToWorld);

    Vertex(const Vertex&) = default;
    Vertex& operator=(const Vertex&) = default;

    /// Topology

    Topology_Vertex topology() const;

    /// 空间查询

    const MyMath::Vector3& localPoint() const;
    MyMath::Vector3 worldPoint() const;
};

}

#endif // MYBREP_INSTANCE_VERTEX_H
