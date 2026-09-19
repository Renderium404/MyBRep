#ifndef MYBREP_MODELING_EDGE_EDGEMODELING_H
#define MYBREP_MODELING_EDGE_EDGEMODELING_H

#include <cstddef>
#include <vector>

#include "MyMath/CoordinateSystem.h"
#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Curve/Geometry_Curve.h"
#include "MyBRep/Instance/Edge.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"

namespace MyBRep
{
namespace Modeling
{

namespace EdgeModelingDetail
{

// 判断指定有向参数区间是否覆盖周期曲线的一个完整周期。
bool isFullPeriodInterval(const Geometry_Curve& geometry,double firstParameter,double lastParameter);

// 返回自动闭合Edge端点重复求值所需的最小数值连接容差。
double automaticClosureTolerance(const MyMath::Vector3& firstPoint,const MyMath::Vector3& lastPoint);

// 校验Geometry_Curve有限Edge使用区间，允许参数方向正向或反向。
void validateCurveInterval(const Geometry_Curve& geometry,double firstParameter,double lastParameter);

// 校验圆弧建模公共输入。
void validateArcInput(double radius,double startAngle,double sweepAngle);

// 使用给定Geometry_Curve和参数区间自动创建端点Topology_Vertex并建立有向Topology_Edge。
Topology_Edge createAutomaticEdge(
    const Foundation::RefPtr<const Geometry_Curve>& geometry,
    double firstParameter,
    double lastParameter);

}

/// 局部Topology_Edge创建

/// 通用Geometry_Curve Edge

// 使用完整Geometry_Curve的指定有限有向参数区间自动创建端点Topology_Vertex和Topology_Edge。
Topology_Edge createEdge(
    const Foundation::RefPtr<const Geometry_Curve>& geometry,
    double firstParameter,
    double lastParameter);

// 使用两个已有Topology_Vertex、完整Geometry_Curve及有限有向参数区间创建Topology_Edge，并保持原有拓扑点身份。
Topology_Edge createEdge(
    const Topology_Vertex& startVertex,
    const Topology_Vertex& endVertex,
    const Foundation::RefPtr<const Geometry_Curve>& geometry,
    double firstParameter,
    double lastParameter,
    double connectionTolerance = MyMath::Vector3::DefaultEpsilon);

/// Line

// 使用两个不同有限三维点创建有限有向直线Topology_Edge。
Topology_Edge createLine(const MyMath::Vector3& startPoint,const MyMath::Vector3& endPoint);

// 使用两个已有且位置不同的Topology_Vertex创建有限有向直线Topology_Edge，并保持两个拓扑点身份。
Topology_Edge createLine(const Topology_Vertex& startVertex,const Topology_Vertex& endVertex);

/// Circle Arc

// 在世界XY平面中创建有限有向圆弧Topology_Edge，正扫掠为逆时针，负扫掠为顺时针。
Topology_Edge createArc(const MyMath::Vector3& center,double radius,double startAngle,double sweepAngle);

// 在指定坐标系XY平面中创建有限有向圆弧Topology_Edge，正扫掠沿坐标系正法向遵循右手规则。
Topology_Edge createArc(const MyMath::CoordinateSystem& coordinateSystem,double radius,double startAngle,double sweepAngle);

// 使用已有起终Topology_Vertex在世界XY平面中创建有限有向圆弧Topology_Edge，并校验顶点与圆弧端点一致。
Topology_Edge createArc(
    const Topology_Vertex& startVertex,
    const Topology_Vertex& endVertex,
    const MyMath::Vector3& center,
    double radius,
    double startAngle,
    double sweepAngle,
    double connectionTolerance = MyMath::Vector3::DefaultEpsilon);

// 使用已有起终Topology_Vertex在指定坐标系XY平面中创建有限有向圆弧Topology_Edge，并校验顶点与圆弧端点一致。
Topology_Edge createArc(
    const Topology_Vertex& startVertex,
    const Topology_Vertex& endVertex,
    const MyMath::CoordinateSystem& coordinateSystem,
    double radius,
    double startAngle,
    double sweepAngle,
    double connectionTolerance = MyMath::Vector3::DefaultEpsilon);

/// Bezier

// 使用完整Bezier控制点序列创建覆盖自然参数域[0,1]的Topology_Edge，并自动创建端点Topology_Vertex。
Topology_Edge createBezier(const std::vector<MyMath::Vector3>& controlPoints);

// 使用已有起终Topology_Vertex和Bezier控制点序列创建覆盖自然参数域[0,1]的Topology_Edge。
Topology_Edge createBezier(
    const Topology_Vertex& startVertex,
    const Topology_Vertex& endVertex,
    const std::vector<MyMath::Vector3>& controlPoints,
    double connectionTolerance = MyMath::Vector3::DefaultEpsilon);

/// B-Spline

// 使用次数、控制点和节点向量创建覆盖B-Spline完整自然参数域的Topology_Edge，并自动创建端点Topology_Vertex。
Topology_Edge createBSpline(
    std::size_t degree,
    const std::vector<MyMath::Vector3>& controlPoints,
    const std::vector<double>& knots);

// 使用已有起终Topology_Vertex、次数、控制点和节点向量创建覆盖B-Spline完整自然参数域的Topology_Edge。
Topology_Edge createBSpline(
    const Topology_Vertex& startVertex,
    const Topology_Vertex& endVertex,
    std::size_t degree,
    const std::vector<MyMath::Vector3>& controlPoints,
    const std::vector<double>& knots,
    double connectionTolerance = MyMath::Vector3::DefaultEpsilon);

/// 空间Edge实例创建

/// 通用Geometry_Curve Edge实例

// 使用单位变换创建通用Geometry_Curve Edge实例，并自动创建端点Topology_Vertex。
Edge makeEdge(
    const Foundation::RefPtr<const Geometry_Curve>& geometry,
    double firstParameter,
    double lastParameter);

// 使用指定可逆仿射变换创建通用Geometry_Curve Edge实例，并自动创建端点Topology_Vertex。
Edge makeEdge(
    const Foundation::RefPtr<const Geometry_Curve>& geometry,
    double firstParameter,
    double lastParameter,
    const MyMath::Matrix4& localToWorld);

// 使用已有起终Topology_Vertex和单位变换创建通用Geometry_Curve Edge实例。
Edge makeEdge(
    const Topology_Vertex& startVertex,
    const Topology_Vertex& endVertex,
    const Foundation::RefPtr<const Geometry_Curve>& geometry,
    double firstParameter,
    double lastParameter,
    double connectionTolerance = MyMath::Vector3::DefaultEpsilon);

// 使用已有起终Topology_Vertex和指定可逆仿射变换创建通用Geometry_Curve Edge实例。
Edge makeEdge(
    const Topology_Vertex& startVertex,
    const Topology_Vertex& endVertex,
    const Foundation::RefPtr<const Geometry_Curve>& geometry,
    double firstParameter,
    double lastParameter,
    const MyMath::Matrix4& localToWorld,
    double connectionTolerance = MyMath::Vector3::DefaultEpsilon);

/// Line实例

// 使用单位变换创建直线Edge实例。
Edge makeLine(const MyMath::Vector3& startPoint,const MyMath::Vector3& endPoint);

// 使用指定可逆仿射变换创建直线Edge实例。
Edge makeLine(const MyMath::Vector3& startPoint,const MyMath::Vector3& endPoint,const MyMath::Matrix4& localToWorld);

// 使用已有Topology_Vertex和单位变换创建直线Edge实例。
Edge makeLine(const Topology_Vertex& startVertex,const Topology_Vertex& endVertex);

// 使用已有Topology_Vertex和指定可逆仿射变换创建直线Edge实例。
Edge makeLine(const Topology_Vertex& startVertex,const Topology_Vertex& endVertex,const MyMath::Matrix4& localToWorld);

/// Circle Arc实例

// 使用单位变换创建世界XY平面圆弧Edge实例。
Edge makeArc(const MyMath::Vector3& center,double radius,double startAngle,double sweepAngle);

// 使用指定可逆仿射变换创建世界XY平面圆弧Edge实例。
Edge makeArc(const MyMath::Vector3& center,double radius,double startAngle,double sweepAngle,const MyMath::Matrix4& localToWorld);

// 使用单位变换创建指定坐标系平面圆弧Edge实例。
Edge makeArc(const MyMath::CoordinateSystem& coordinateSystem,double radius,double startAngle,double sweepAngle);

// 使用指定可逆仿射变换创建指定坐标系平面圆弧Edge实例。
Edge makeArc(
    const MyMath::CoordinateSystem& coordinateSystem,
    double radius,
    double startAngle,
    double sweepAngle,
    const MyMath::Matrix4& localToWorld);

// 使用已有Topology_Vertex和单位变换创建世界XY平面圆弧Edge实例。
Edge makeArc(
    const Topology_Vertex& startVertex,
    const Topology_Vertex& endVertex,
    const MyMath::Vector3& center,
    double radius,
    double startAngle,
    double sweepAngle,
    double connectionTolerance = MyMath::Vector3::DefaultEpsilon);

// 使用已有Topology_Vertex和指定可逆仿射变换创建世界XY平面圆弧Edge实例。
Edge makeArc(
    const Topology_Vertex& startVertex,
    const Topology_Vertex& endVertex,
    const MyMath::Vector3& center,
    double radius,
    double startAngle,
    double sweepAngle,
    const MyMath::Matrix4& localToWorld,
    double connectionTolerance = MyMath::Vector3::DefaultEpsilon);

// 使用已有Topology_Vertex和单位变换创建指定坐标系平面圆弧Edge实例。
Edge makeArc(
    const Topology_Vertex& startVertex,
    const Topology_Vertex& endVertex,
    const MyMath::CoordinateSystem& coordinateSystem,
    double radius,
    double startAngle,
    double sweepAngle,
    double connectionTolerance = MyMath::Vector3::DefaultEpsilon);

// 使用已有Topology_Vertex和指定可逆仿射变换创建指定坐标系平面圆弧Edge实例。
Edge makeArc(
    const Topology_Vertex& startVertex,
    const Topology_Vertex& endVertex,
    const MyMath::CoordinateSystem& coordinateSystem,
    double radius,
    double startAngle,
    double sweepAngle,
    const MyMath::Matrix4& localToWorld,
    double connectionTolerance = MyMath::Vector3::DefaultEpsilon);

/// Bezier实例

// 使用单位变换创建完整Bezier Edge实例。
Edge makeBezier(const std::vector<MyMath::Vector3>& controlPoints);

// 使用指定可逆仿射变换创建完整Bezier Edge实例。
Edge makeBezier(const std::vector<MyMath::Vector3>& controlPoints,const MyMath::Matrix4& localToWorld);

// 使用已有Topology_Vertex和单位变换创建完整Bezier Edge实例。
Edge makeBezier(
    const Topology_Vertex& startVertex,
    const Topology_Vertex& endVertex,
    const std::vector<MyMath::Vector3>& controlPoints,
    double connectionTolerance = MyMath::Vector3::DefaultEpsilon);

// 使用已有Topology_Vertex和指定可逆仿射变换创建完整Bezier Edge实例。
Edge makeBezier(
    const Topology_Vertex& startVertex,
    const Topology_Vertex& endVertex,
    const std::vector<MyMath::Vector3>& controlPoints,
    const MyMath::Matrix4& localToWorld,
    double connectionTolerance = MyMath::Vector3::DefaultEpsilon);

/// B-Spline实例

// 使用单位变换创建完整B-Spline Edge实例。
Edge makeBSpline(
    std::size_t degree,
    const std::vector<MyMath::Vector3>& controlPoints,
    const std::vector<double>& knots);

// 使用指定可逆仿射变换创建完整B-Spline Edge实例。
Edge makeBSpline(
    std::size_t degree,
    const std::vector<MyMath::Vector3>& controlPoints,
    const std::vector<double>& knots,
    const MyMath::Matrix4& localToWorld);

// 使用已有Topology_Vertex和单位变换创建完整B-Spline Edge实例。
Edge makeBSpline(
    const Topology_Vertex& startVertex,
    const Topology_Vertex& endVertex,
    std::size_t degree,
    const std::vector<MyMath::Vector3>& controlPoints,
    const std::vector<double>& knots,
    double connectionTolerance = MyMath::Vector3::DefaultEpsilon);

// 使用已有Topology_Vertex和指定可逆仿射变换创建完整B-Spline Edge实例。
Edge makeBSpline(
    const Topology_Vertex& startVertex,
    const Topology_Vertex& endVertex,
    std::size_t degree,
    const std::vector<MyMath::Vector3>& controlPoints,
    const std::vector<double>& knots,
    const MyMath::Matrix4& localToWorld,
    double connectionTolerance = MyMath::Vector3::DefaultEpsilon);

}
}

#endif // MYBREP_MODELING_EDGE_EDGEMODELING_H
