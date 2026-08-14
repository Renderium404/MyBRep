#ifndef MYBREP_MODELING_EDGE_EDGEMODELING_H
#define MYBREP_MODELING_EDGE_EDGEMODELING_H

#include "MyMath/CoordinateSystem.h"
#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"
#include "MyBRep/Instance/Edge.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"

namespace MyBRep
{
namespace Modeling
{

/// 局部Topology_Edge创建

// 使用两个不同有限三维点创建有限有向直线Topology_Edge。
Topology_Edge createLine(const MyMath::Vector3& startPoint,
                         const MyMath::Vector3& endPoint);

// 在世界XY平面中创建有限有向圆弧Topology_Edge，正扫掠为逆时针，负扫掠为顺时针。
Topology_Edge createArc(const MyMath::Vector3& center,
                        double radius,
                        double startAngle,
                        double sweepAngle);

// 在指定坐标系XY平面中创建有限有向圆弧Topology_Edge，正扫掠沿坐标系正法向遵循右手规则。
Topology_Edge createArc(const MyMath::CoordinateSystem& coordinateSystem,
                        double radius,
                        double startAngle,
                        double sweepAngle);

/// 空间Edge实例创建

// 使用单位变换创建直线Edge实例。
Edge makeLine(const MyMath::Vector3& startPoint,
              const MyMath::Vector3& endPoint);

// 使用指定可逆仿射变换创建直线Edge实例。
Edge makeLine(const MyMath::Vector3& startPoint,
              const MyMath::Vector3& endPoint,
              const MyMath::Matrix4& localToWorld);

// 使用单位变换创建世界XY平面圆弧Edge实例。
Edge makeArc(const MyMath::Vector3& center,
             double radius,
             double startAngle,
             double sweepAngle);

// 使用指定可逆仿射变换创建世界XY平面圆弧Edge实例。
Edge makeArc(const MyMath::Vector3& center,
             double radius,
             double startAngle,
             double sweepAngle,
             const MyMath::Matrix4& localToWorld);

// 使用单位变换创建指定坐标系平面圆弧Edge实例。
Edge makeArc(const MyMath::CoordinateSystem& coordinateSystem,
             double radius,
             double startAngle,
             double sweepAngle);

// 使用指定可逆仿射变换创建指定坐标系平面圆弧Edge实例。
Edge makeArc(const MyMath::CoordinateSystem& coordinateSystem,
             double radius,
             double startAngle,
             double sweepAngle,
             const MyMath::Matrix4& localToWorld);

}
}

#endif // MYBREP_MODELING_EDGE_EDGEMODELING_H
