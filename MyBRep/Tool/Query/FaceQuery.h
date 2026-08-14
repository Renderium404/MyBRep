#ifndef MYBREP_QUERY_FACEQUERY_H
#define MYBREP_QUERY_FACEQUERY_H

#include <cstddef>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector2.h"
#include "MyMath/Vector3.h"
#include "MyBRep/Base/Bounds3.h"
#include "MyBRep/Geometry/Surface/Geometry_Surface.h"
#include "MyBRep/Instance/Face.h"
#include "MyBRep/Query/EdgeQuery.h"
#include "MyBRep/Topology/Face/Topology_Face.h"

namespace MyBRep
{

// 表示一个Topology_Face在指定查询坐标系中的不可变参数曲面查询器。
//
// FaceQuery直接复用Topology_Edge保存的Curve-on-Surface表示查询UV关系。
// boundaryBounds()只描述全部裁剪Edge的三维范围，不等价于裁剪Face完整空间包围盒。
class FaceQuery
{
public:
    // 使用Topology_Face自身局部坐标系创建查询器。
    explicit FaceQuery(const Topology_Face& topology);
    // 使用世界坐标系查询指定空间Face实例。
    explicit FaceQuery(const Face& face);
    // 使用指定查询坐标系查询空间Face实例，queryToWorld必须为可逆仿射变换。
    FaceQuery(const Face& face, const MyMath::Matrix4& queryToWorld);
    FaceQuery(const FaceQuery&) = default;
    FaceQuery& operator=(const FaceQuery&) = default;

    /// 查询对象与空间数据

    // 返回当前查询器引用的空间Face实例。
    const Face& face() const;
    // 返回当前查询器引用的局部Topology_Face。
    const Topology_Face& topology() const;
    // 返回当前Face引用的完整参数曲面。
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
    EdgeQuery edgeQuery(std::size_t wireIndex,
                        std::size_t edgeIndex) const;
    // 返回指定Edge-use规范化参数[0,1]对应的Surface二维参数。
    MyMath::Vector2 surfaceParameterAt(std::size_t wireIndex,
                                       std::size_t edgeIndex,
                                       double parameter) const;
    // 返回全部裁剪Edge在查询坐标系中的保守轴对齐包围盒；无裁剪Wire的Face返回无效Bounds3。
    Bounds3 boundaryBounds() const;

private:
    // 使用空间Face和查询坐标到世界坐标的变换建立查询缓存。
    void initialize(const Face& face, const MyMath::Matrix4& queryToWorld);

private:
    Face m_face; // 当前查询器引用的空间Face实例。
    MyMath::Matrix4 m_queryToWorld; // 当前查询坐标到世界坐标的可逆仿射变换。
    MyMath::Matrix4 m_queryToLocal; // 查询坐标到Face局部坐标的变换。
    MyMath::Matrix4 m_localToQuery; // Face局部坐标到查询坐标的逆变换。
};

}

#endif // MYBREP_QUERY_FACEQUERY_H
