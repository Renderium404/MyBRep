#ifndef MYBREP_QUERY_SHAPEQUERY_H
#define MYBREP_QUERY_SHAPEQUERY_H

#include <array>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyBRep/Base/Bounds3.h"
#include "MyBRep/Geometry/Shape/Geometry_Shape.h"
#include "MyBRep/Geometry/Shape/ShapeRelation.h"
#include "MyBRep/Instance/Shape.h"
#include "MyBRep/Topology/Shape/Topology_Shape.h"

namespace MyBRep
{

// 表示一个Topology_Shape在指定查询坐标系中的不可变空间查询快照。
//
// ShapeQuery不拥有也不复制Shape实例；构造时只读取Topology和空间放置并建立查询缓存。
// ShapeQuery只查询Topology_Shape直接持有的Geometry_Shape连续体；实例后续移动不会自动更新现有Query。
class ShapeQuery
{
public:
    enum
    {
        OctantCount = 8
    };

public:
    // 使用Topology_Shape自身局部坐标系创建通用查询器。
    explicit ShapeQuery(const Topology_Shape& topology);
    // 使用指定空间放置和查询坐标系查询Topology_Shape，不创建Shape实例。
    ShapeQuery(const Topology_Shape& topology, const MyMath::Matrix4& localToWorld, const MyMath::Matrix4& queryToWorld);
    // 使用世界坐标系查询指定Shape实例的当前快照。
    explicit ShapeQuery(const Shape& shape);
    // 使用指定查询坐标系查询Shape实例的当前快照。
    ShapeQuery(const Shape& shape, const MyMath::Matrix4& queryToWorld);
    ShapeQuery(const ShapeQuery&) = delete;
    ShapeQuery& operator=(const ShapeQuery&) = delete;
    virtual ~ShapeQuery();

    /// 查询能力

    // 判断查询坐标系是否与Shape局部坐标系完全相同。
    bool isIdentityQuery() const;
    // 判断当前Geometry_Shape和查询坐标变换是否能够提供精确查询空间有符号距离。
    bool supportsSignedDistance() const;

    /// 查询对象与空间数据

    // 返回当前查询快照共享的局部Topology_Shape。
    const Topology_Shape& topology() const;
    // 返回当前Topology_Shape引用的局部连续实体几何内核。
    const Geometry_Shape& geometry() const;
    // 返回查询坐标到Shape局部坐标的可逆仿射变换。
    const MyMath::Matrix4& queryToLocal() const;
    // 返回Shape局部坐标到查询坐标的逆变换。
    const MyMath::Matrix4& localToQuery() const;
    // 返回Shape在查询坐标系中的保守轴对齐包围盒。
    const Bounds3& queryBounds() const;

    /// 标准空间查询

    // 判断查询坐标系中的指定点是否位于Shape内部或边界上。
    bool containsPoint(const MyMath::Vector3& point) const;
    // 返回查询坐标系中指定点到Shape边界的精确有符号距离；仅支持局部SDF且查询到局部为刚体或统一缩放变换。
    double signedDistanceToPoint(const MyMath::Vector3& point) const;
    // 使用完整Bounds3执行保守分类，非单位查询通过八角点变换得到局部保守AABB后调用Geometry_Shape标准分类。
    ShapeRelation classifyBounds(const Bounds3& bounds) const;

    /// 快速空间查询

    // 使用已经计算好的查询空间包围盒中心和半尺寸执行保守分类。
    ShapeRelation classifyBoundsFast(const MyMath::Vector3& center, const MyMath::Vector3& extent) const;
    // 批量分类由parentCenter和childExtent定义的八个等尺寸子包围盒。
    void classifyOctantBoundsFast(const MyMath::Vector3& parentCenter, const MyMath::Vector3& childExtent,
                                  std::array<ShapeRelation, OctantCount>& results) const;

protected:
    /// 特殊Shape快速查询扩展

    // 执行单个查询空间包围盒快速分类，派生查询器可针对特殊Shape覆写该热点实现。
    virtual ShapeRelation classifyBoundsFastImpl(const MyMath::Vector3& center, const MyMath::Vector3& extent) const;
    // 执行八个等尺寸子包围盒批量快速分类，派生查询器可共享中间量并覆写该热点实现。
    virtual void classifyOctantBoundsFastImpl(const MyMath::Vector3& parentCenter, const MyMath::Vector3& childExtent,
                                              std::array<ShapeRelation, OctantCount>& results) const;

    /// 通用快速查询辅助

    // 使用缓存的queryToLocal线性部分绝对值行，将查询空间AABB中心和半尺寸保守转换到Shape局部空间。
    void transformBoundsToLocalFast(const MyMath::Vector3& center, const MyMath::Vector3& extent,
                                    MyMath::Vector3& localCenter, MyMath::Vector3& localExtent) const;
    // 判断查询空间中心和半尺寸定义的AABB是否与Shape查询包围盒相交。
    bool intersectsQueryBounds(const MyMath::Vector3& center, const MyMath::Vector3& extent) const;

private:
    // 使用Topology、空间放置和查询坐标到世界坐标的变换建立全部查询缓存。
    void initialize(const Topology_Shape& topology, const MyMath::Matrix4& localToWorld, const MyMath::Matrix4& queryToWorld);
    // 更新queryToLocal线性部分三个矩阵行的分量绝对值缓存。
    void updateAbsoluteQueryToLocalRows();
    // 检查queryToLocal是否保持欧氏距离到统一比例，并缓存局部长度与查询长度的比例。
    void updateSignedDistanceMetric();

    // 判断半尺寸是否为有限非负数据。
    static bool isValidExtent(const MyMath::Vector3& extent);
    // 返回指定八分体的查询空间中心。
    static MyMath::Vector3 octantCenter(const MyMath::Vector3& parentCenter, const MyMath::Vector3& childExtent,
                                        unsigned int octantIndex);

private:
    static const unsigned int XOctantMask;
    static const unsigned int YOctantMask;
    static const unsigned int ZOctantMask;
    static const double SignedDistanceMetricToleranceScale;

    Topology_Shape m_topology; // 当前查询快照共享的Topology_Shape。
    MyMath::Matrix4 m_queryToLocal; // 查询坐标到Shape局部坐标的变换。
    MyMath::Matrix4 m_localToQuery; // Shape局部坐标到查询坐标的逆变换。
    MyMath::Vector3 m_absoluteQueryToLocalRowX; // queryToLocal线性部分第0行各分量绝对值。
    MyMath::Vector3 m_absoluteQueryToLocalRowY; // queryToLocal线性部分第1行各分量绝对值。
    MyMath::Vector3 m_absoluteQueryToLocalRowZ; // queryToLocal线性部分第2行各分量绝对值。
    double m_localDistancePerQueryUnit; // 查询空间单位长度经过queryToLocal后对应的统一局部长度。
    Bounds3 m_queryBounds; // Shape在查询坐标系中的保守轴对齐包围盒。
    bool m_signedDistanceMetricValid; // queryToLocal是否保持欧氏距离到统一比例。
    bool m_identityQuery; // 查询坐标系是否与Shape局部坐标系完全相同。
};

}

#endif // MYBREP_QUERY_SHAPEQUERY_H
