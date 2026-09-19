#ifndef MYBREP_QUERY_WIREQUERY_H
#define MYBREP_QUERY_WIREQUERY_H

#include <cstddef>

#include "MyMath/Matrix4.h"
#include "MyBRep/Base/Bounds3.h"
#include "MyBRep/Instance/Wire.h"
#include "MyBRep/Tool/Query/EdgeQuery.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

namespace MyBRep
{

// 表示一个Topology_Wire在指定查询坐标系中的不可变线性几何查询快照。
//
// WireQuery不拥有也不复制Wire实例；构造时只读取Topology和空间放置。
// WireQuery按Edge-use顺序聚合范围和弧长；实例后续移动不会自动更新现有Query。
class WireQuery
{
public:
    // 使用Topology_Wire自身局部坐标系创建查询器。
    explicit WireQuery(const Topology_Wire& topology);
    // 使用指定空间放置和查询坐标系查询Topology_Wire，不创建Wire实例。
    WireQuery(const Topology_Wire& topology, const MyMath::Matrix4& localToWorld, const MyMath::Matrix4& queryToWorld);
    // 使用世界坐标系查询指定Wire实例的当前快照。
    explicit WireQuery(const Wire& wire);
    // 使用指定查询坐标系查询Wire实例的当前快照。
    WireQuery(const Wire& wire, const MyMath::Matrix4& queryToWorld);
    WireQuery(const WireQuery&) = default;
    WireQuery& operator=(const WireQuery&) = default;

    /// 查询对象与空间数据

    // 返回当前查询快照共享的局部Topology_Wire。
    const Topology_Wire& topology() const;
    // 返回Wire在查询坐标系中的保守轴对齐包围盒。
    const Bounds3& queryBounds() const;

    /// Edge查询

    // 返回当前Wire包含的Edge数量。
    std::size_t edgeCount() const;
    // 返回指定Edge-use在当前查询坐标系中的EdgeQuery。
    EdgeQuery edgeQuery(std::size_t index) const;

    /// 线性度量

    // 返回全部Edge-use在查询坐标系中的弧长总和。
    double length(double absoluteTolerance = 1.0e-10, unsigned int maxSubdivisionDepth = 20) const;

private:
    // 使用Topology、空间放置和查询坐标到世界坐标的变换建立查询快照。
    void initialize(const Topology_Wire& topology, const MyMath::Matrix4& localToWorld, const MyMath::Matrix4& queryToWorld);

private:
    Topology_Wire m_topology; // 当前查询快照共享的Topology_Wire。
    MyMath::Matrix4 m_localToWorld; // Wire局部坐标到世界坐标的空间放置快照。
    MyMath::Matrix4 m_queryToWorld; // 当前查询坐标到世界坐标的可逆仿射变换。
    Bounds3 m_queryBounds; // Wire全部有限Edge-use在查询坐标系中的保守轴对齐包围盒。
};

}

#endif // MYBREP_QUERY_WIREQUERY_H
