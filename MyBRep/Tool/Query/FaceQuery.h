#ifndef MYBREP_QUERY_FACEQUERY_H
#define MYBREP_QUERY_FACEQUERY_H

#include <cstddef>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector2.h"
#include "MyMath/Vector3.h"
#include "MyBRep/Base/Bounds3.h"
#include "MyBRep/Geometry/Surface/Geometry_Surface.h"
#include "MyBRep/Instance/Face.h"
#include "MyBRep/Tool/Query/EdgeQuery.h"
#include "MyBRep/Topology/Face/Topology_Face.h"

namespace MyBRep
{

// 表示一个Topology_Face在指定查询坐标系中的不可变参数曲面查询快照。
//
// FaceQuery不拥有也不复制Face实例，只在构造时读取Topology和空间放置。
// boundaryBounds()只描述全部裁剪Edge的三维范围；实例后续移动不会自动更新现有Query。
class FaceQuery
{
public:
    // 使用Topology_Face自身局部坐标系创建查询器。
    explicit FaceQuery(const Topology_Face& topology);
    // 使用指定空间放置和查询坐标系查询Topology_Face，不创建Face实例。
    FaceQuery(const Topology_Face& topology, const MyMath::Matrix4& localToWorld, const MyMath::Matrix4& queryToWorld);
    // 使用世界坐标系查询指定Face实例的当前快照。
    explicit FaceQuery(const Face& face);
    // 使用指定查询坐标系查询Face实例的当前快照。
    FaceQuery(const Face& face, const MyMath::Matrix4& queryToWorld);
    FaceQuery(const FaceQuery&) = default;
    FaceQuery& operator=(const FaceQuery&) = default;

    /// 查询对象与空间数据

    // 返回当前查询快照共享的局部Topology_Face。
    const Topology_Face& topology() const;
    // 返回当前Topology_Face引用的完整参数曲面。
    const Geometry_Surface& geometry() const;
    // 返回查询坐标到Face局部坐标的变换。
    const MyMath::Matrix4& queryToLocal() const;
    // 返回Face局部坐标到查询坐标的变换。
    const MyMath::Matrix4& localToQuery() const;

    /// 曲面参数查询

    // 返回指定U/V参数对应的查询坐标系曲面点。
    MyMath::Vector3 pointAt(double u, double v) const;
    // 返回指定正则U/V参数处符合Topology_Face使用方向的查询坐标系单位法向。
    MyMath::Vector3 normalAt(double u, double v) const;

    /// 裁剪边界查询

    // 返回当前Face使用方向下的裁剪Wire数量。
    std::size_t wireCount() const;
    // 返回指定Wire包含的Edge-use数量。
    std::size_t edgeCount(std::size_t wireIndex) const;
    // 返回指定Edge-use在当前查询坐标系中的EdgeQuery。
    EdgeQuery edgeQuery(std::size_t wireIndex, std::size_t edgeIndex) const;
    // 返回指定Edge-use规范化参数[0,1]对应的Surface二维参数。
    MyMath::Vector2 surfaceParameterAt(std::size_t wireIndex, std::size_t edgeIndex, double parameter) const;
    // 返回全部裁剪Edge在查询坐标系中的保守轴对齐包围盒；无裁剪Wire的Face返回无效Bounds3。
    Bounds3 boundaryBounds() const;

private:
    // 使用Topology、空间放置和查询坐标到世界坐标的变换建立查询快照。
    void initialize(const Topology_Face& topology, const MyMath::Matrix4& localToWorld, const MyMath::Matrix4& queryToWorld);

private:
    Topology_Face m_topology; // 当前查询快照共享的Topology_Face。
    MyMath::Matrix4 m_localToWorld; // Face局部坐标到世界坐标的空间放置快照。
    MyMath::Matrix4 m_queryToWorld; // 当前查询坐标到世界坐标的可逆仿射变换。
    MyMath::Matrix4 m_queryToLocal; // 查询坐标到Face局部坐标的变换。
    MyMath::Matrix4 m_localToQuery; // Face局部坐标到查询坐标的逆变换。
};

}

#endif // MYBREP_QUERY_FACEQUERY_H
