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

// 表示一个Topology_Wire在指定查询坐标系中的不可变线性几何查询器。
//
// WireQuery按Wire中的Edge-use顺序聚合有限Edge范围和弧长；同一共享Topology_TEdge重复出现时按实际使用次数计入。
class WireQuery
{
public:
    // 使用Topology_Wire自身局部坐标系创建查询器。
    explicit WireQuery(const Topology_Wire& topology);
    // 使用世界坐标系查询指定空间Wire实例。
    explicit WireQuery(const Wire& wire);
    // 使用指定查询坐标系查询空间Wire实例，queryToWorld必须为可逆仿射变换。
    WireQuery(const Wire& wire, const MyMath::Matrix4& queryToWorld);
    WireQuery(const WireQuery&) = default;
    WireQuery& operator=(const WireQuery&) = default;

    /// 查询对象与空间数据

    // 返回当前查询器引用的空间Wire实例。
    const Wire& wire() const;
    // 返回当前查询器引用的局部Topology_Wire。
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
    double length(double absoluteTolerance = 1.0e-10,
                  unsigned int maxSubdivisionDepth = 20) const;

private:
    // 使用空间Wire和查询坐标到世界坐标的变换建立查询缓存。
    void initialize(const Wire& wire, const MyMath::Matrix4& queryToWorld);

private:
    Wire m_wire; // 当前查询器引用的空间Wire实例。
    MyMath::Matrix4 m_queryToWorld; // 当前查询坐标到世界坐标的可逆仿射变换。
    Bounds3 m_queryBounds; // Wire全部有限Edge-use在查询坐标系中的保守轴对齐包围盒。
};

}

#endif // MYBREP_QUERY_WIREQUERY_H
