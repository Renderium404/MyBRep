#ifndef MYBREP_INSTANCE_EDGE_H
#define MYBREP_INSTANCE_EDGE_H

#include "MyMath/Vector3.h"
#include "MyBRep/Geometry/Curve/Geometry_Curve.h"
#include "MyBRep/Instance/Instance.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"

namespace MyBRep
{

class Edge : public Instance
{
public:
    Edge();
    explicit Edge(const Topology_Edge& topology);
    Edge(const Topology_Edge& topology, const MyMath::Matrix4& localToWorld);

    Edge(const Edge&) = default;
    Edge& operator=(const Edge&) = default;

    /// Topology与Geometry

    Topology_Edge topology() const;

    bool sharesGeometryWith(const Edge& other) const;

    const Geometry_Curve& geometry() const;
    const Geometry_Curve* geometryPointer() const;
    CurveKind kind() const;

    /// 有向端点

    MyMath::Vector3 localStartPoint() const;
    MyMath::Vector3 localEndPoint() const;
    MyMath::Vector3 worldStartPoint() const;
    MyMath::Vector3 worldEndPoint() const;

    /// 局部空间查询

    MyMath::Vector3 pointAt(double parameter) const;
    MyMath::Vector3 tangentAt(double parameter) const;

    /// 世界空间查询

    MyMath::Vector3 worldPointAt(double parameter) const;
    MyMath::Vector3 worldTangentAt(double parameter) const;

    /// 方向操作

    Edge reversed() const;
};

}

#endif // MYBREP_INSTANCE_EDGE_H
