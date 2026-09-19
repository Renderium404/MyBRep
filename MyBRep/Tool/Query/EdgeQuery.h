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

// 表示一个有限Topology_Edge在指定查询坐标系中的不可变几何查询快照。
//
// EdgeQuery不拥有也不复制Edge实例，只在构造时读取Topology和空间放置并建立查询缓存。
// EdgeQuery查询Topology_Edge裁剪后的有限曲线段；实例后续移动不会自动更新现有Query。
class EdgeQuery
{
public:
    // 使用Topology_Edge自身局部坐标系创建有限边查询器。
    explicit EdgeQuery(const Topology_Edge& topology);
    // 使用指定空间放置和查询坐标系查询Topology_Edge，不创建Edge实例。
    EdgeQuery(const Topology_Edge& topology, const MyMath::Matrix4& localToWorld, const MyMath::Matrix4& queryToWorld);
    // 使用世界坐标系查询指定Edge实例的当前快照。
    explicit EdgeQuery(const Edge& edge);
    // 使用指定查询坐标系查询Edge实例的当前快照。
    EdgeQuery(const Edge& edge, const MyMath::Matrix4& queryToWorld);
    EdgeQuery(const EdgeQuery&) = default;
    EdgeQuery& operator=(const EdgeQuery&) = default;

    /// 查询对象与空间数据

    // 返回当前查询快照引用的局部Topology_Edge。
    const Topology_Edge& topology() const;
    // 返回当前Topology_Edge引用的完整三维参数曲线。
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
    // absoluteTolerance必须为有限正数；maxSubdivisionDepth必须大于零。
    double length(double absoluteTolerance = 1.0e-10, unsigned int maxSubdivisionDepth = 20) const;

private:
    // 使用Topology、空间放置和查询坐标到世界坐标的变换建立查询快照。
    void initialize(const Topology_Edge& topology, const MyMath::Matrix4& localToWorld, const MyMath::Matrix4& queryToWorld);
    // 建立有限Edge在查询坐标系中的保守轴对齐包围盒。
    void rebuildBounds();

    // 判断标量是否为有限正数。
    static bool isFinitePositive(double value);
    // 返回Simpson三点公式在[first,last]上的积分值。
    static double simpsonValue(double first, double last, double firstValue, double middleValue, double lastValue);
    // 判断周期参数candidate经过整数个period平移后是否落入闭区间[first,last]。
    static bool periodicParameterInInterval(double candidate, double period, double first, double last);
    // 将指定有限点包含进Bounds3。
    static void includePoint(Bounds3& bounds, const MyMath::Vector3& point);
    // 返回Vector3指定轴分量。
    static double component(const MyMath::Vector3& value, int axis);

    // 返回指定三维曲线自然参数处在查询坐标系中的速度大小。
    double speedAtCurveParameter(double curveParameter) const;
    // 对具有连续一阶导数的指定自然参数区间执行自适应Simpson弧长积分。
    double integrateLength(double firstParameter, double lastParameter, double absoluteTolerance, unsigned int maxSubdivisionDepth) const;
    // 对可能仅C0连续的曲线区间使用自适应折线细分计算弧长。
    double integratePolylineLength(double firstParameter, double lastParameter, double absoluteTolerance, unsigned int maxSubdivisionDepth) const;

private:
    static const double SimpsonErrorScale;

    Topology_Edge m_topology; // 当前查询快照共享的Topology_Edge。
    MyMath::Matrix4 m_queryToLocal; // 查询坐标到Edge局部坐标的变换。
    MyMath::Matrix4 m_localToQuery; // Edge局部坐标到查询坐标的逆变换。
    Bounds3 m_queryBounds; // 有限Edge在查询坐标系中的保守轴对齐包围盒。
};

}

#endif // MYBREP_QUERY_EDGEQUERY_H
