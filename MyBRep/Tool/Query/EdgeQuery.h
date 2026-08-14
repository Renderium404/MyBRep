#ifndef MYBREP_QUERY_EDGEQUERY_H
#define MYBREP_QUERY_EDGEQUERY_H

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyBRep/Base/Bounds3.h"
#include "MyBRep/Geometry/Curve/Geometry_Curve.h"
#include "MyBRep/Instance/Edge.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"

namespace MyBRep
{

// 表示一个有限Topology_Edge在指定查询坐标系中的不可变几何查询器。
//
// EdgeQuery查询的是Topology_Edge裁剪后的有限曲线段，而不是完整Geometry_Curve。
// 查询坐标系通过queryToWorld定义；弧长在查询坐标系度量下计算，包围盒直接在查询坐标系中建立。
class EdgeQuery
{
public:
    // 使用Topology_Edge自身局部坐标系创建有限边查询器。
    explicit EdgeQuery(const Topology_Edge& topology);
    // 使用世界坐标系查询指定空间Edge实例。
    explicit EdgeQuery(const Edge& edge);
    // 使用指定查询坐标系查询空间Edge实例，queryToWorld必须为可逆仿射变换。
    EdgeQuery(const Edge& edge, const MyMath::Matrix4& queryToWorld);
    EdgeQuery(const EdgeQuery&) = default;
    EdgeQuery& operator=(const EdgeQuery&) = default;

    /// 查询对象与空间数据

    // 返回当前查询器引用的空间Edge实例。
    const Edge& edge() const;
    // 返回当前查询器引用的局部Topology_Edge。
    const Topology_Edge& topology() const;
    // 返回当前查询器引用的完整三维参数曲线。
    const Geometry_Curve& geometry() const;
    // 返回查询坐标到Edge局部坐标的变换。
    const MyMath::Matrix4& queryToLocal() const;
    // 返回Edge局部坐标到查询坐标的变换。
    const MyMath::Matrix4& localToQuery() const;
    // 返回有限Edge在查询坐标系中的保守轴对齐包围盒。
    const Bounds3& queryBounds() const;

    /// 参数查询

    // 返回当前Edge规范化参数[0,1]对应的底层三维曲线自然参数。
    double curveParameterAt(double parameter) const;
    // 返回当前Edge规范化参数[0,1]对应的查询坐标系曲线点。
    MyMath::Vector3 pointAt(double parameter) const;
    // 返回当前Edge规范化参数[0,1]对应的查询坐标系单位切向量。
    MyMath::Vector3 tangentAt(double parameter) const;

    /// 线性度量

    // 使用自适应Simpson积分返回查询坐标系中的Edge弧长。
    //
    // absoluteTolerance必须为有限正数；maxSubdivisionDepth必须大于零。
    double length(double absoluteTolerance = 1.0e-10,
                  unsigned int maxSubdivisionDepth = 20) const;

private:
    // 使用空间Edge和查询坐标到世界坐标的变换建立查询缓存。
    void initialize(const Edge& edge, const MyMath::Matrix4& queryToWorld);
    // 建立有限Edge在查询坐标系中的保守轴对齐包围盒。
    void rebuildBounds();

    // 返回指定三维曲线自然参数处在查询坐标系中的速度大小。
    double speedAtCurveParameter(double curveParameter) const;
    // 对具有连续一阶导数的指定自然参数区间执行自适应Simpson弧长积分。
    double integrateLength(double firstParameter,
                           double lastParameter,
                           double absoluteTolerance,
                           unsigned int maxSubdivisionDepth) const;
    // 对可能仅C0连续的曲线区间使用自适应折线细分计算弧长。
    double integratePolylineLength(double firstParameter,
                                   double lastParameter,
                                   double absoluteTolerance,
                                   unsigned int maxSubdivisionDepth) const;

private:
    Edge m_edge; // 当前查询器引用的空间Edge实例。
    MyMath::Matrix4 m_queryToLocal; // 查询坐标到Edge局部坐标的变换。
    MyMath::Matrix4 m_localToQuery; // Edge局部坐标到查询坐标的逆变换。
    Bounds3 m_queryBounds; // 有限Edge在查询坐标系中的保守轴对齐包围盒。
};

}

#endif // MYBREP_QUERY_EDGEQUERY_H
