#ifndef MYBREP_MODELING_SHAPE_SHAPEMODELING_H
#define MYBREP_MODELING_SHAPE_SHAPEMODELING_H

#include "MyMath/Matrix4.h"
#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Shape/Geometry_Shape.h"
#include "MyBRep/Instance/Shape.h"
#include "MyBRep/Instance/Wire.h"
#include "MyBRep/Topology/Shape/Topology_Shape.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

namespace MyBRep
{
namespace Modeling
{

/// 通用Topology_Shape创建

// 使用指定连续Geometry_Shape内核创建Topology_Shape。
Topology_Shape createShape(const Foundation::RefPtr<const Geometry_Shape>& geometry);

/// 标准解析Topology_Shape创建

// 创建以局部原点为中心并与局部坐标轴平行的标准长方体Topology_Shape。
Topology_Shape createBox(double sizeX,
                         double sizeY,
                         double sizeZ);

// 创建球心位于局部原点的标准球体Topology_Shape。
Topology_Shape createSphere(double radius);

// 创建轴线沿局部Z轴且中心位于局部原点的标准圆柱体Topology_Shape。
Topology_Shape createCylinder(double radius,double height);

// 创建底面位于局部Z负方向、顶点位于局部Z正方向的标准圆锥Topology_Shape。
Topology_Shape createCone(double bottomRadius,double height);

// 创建轴线沿局部Z轴且中心位于局部原点的标准圆锥台Topology_Shape。
Topology_Shape createConeFrustum(double bottomRadius, double topRadius, double height);
/// 构造连续体Topology_Shape创建
// 使用局部XY平面闭合Topology_Wire沿局部Z轴对称拉伸创建Geometry_Extruded连续体Topology_Shape。
Topology_Shape createExtruded( const Topology_Wire& profile,double height,double profileTolerance = MyMath::Vector3::DefaultEpsilon);

/// 空间Shape实例创建

// 使用指定连续Geometry_Shape和单位变换创建Shape实例。
Shape makeShape(const Foundation::RefPtr<const Geometry_Shape>& geometry);

// 使用指定连续Geometry_Shape和可逆仿射变换创建Shape实例。
Shape makeShape(const Foundation::RefPtr<const Geometry_Shape>& geometry,const MyMath::Matrix4& localToWorld);

// 使用单位变换创建标准长方体Shape实例。
Shape makeBox(double sizeX,
              double sizeY,
              double sizeZ);

// 使用指定可逆仿射变换创建标准长方体Shape实例。
Shape makeBox(double sizeX,
              double sizeY,
              double sizeZ,
              const MyMath::Matrix4& localToWorld);

// 使用单位变换创建标准球体Shape实例。
Shape makeSphere(double radius);

// 使用指定可逆仿射变换创建标准球体Shape实例。
Shape makeSphere(double radius, const MyMath::Matrix4& localToWorld);

// 使用单位变换创建标准圆柱体Shape实例。
Shape makeCylinder(double radius, double height);

// 使用指定可逆仿射变换创建标准圆柱体Shape实例。
Shape makeCylinder(double radius,double height,const MyMath::Matrix4& localToWorld);

// 使用单位变换创建标准圆锥Shape实例。
Shape makeCone(double bottomRadius,double height);

// 使用指定可逆仿射变换创建标准圆锥Shape实例。
Shape makeCone(double bottomRadius,double height,const MyMath::Matrix4& localToWorld);

// 使用单位变换创建标准圆锥台Shape实例。
Shape makeConeFrustum(double bottomRadius,double topRadius,double height);

// 使用指定可逆仿射变换创建标准圆锥台Shape实例。
Shape makeConeFrustum(double bottomRadius,double topRadius,double height,const MyMath::Matrix4& localToWorld);

// 使用单位变换创建闭合轮廓拉伸Shape实例。
Shape makeExtruded(const Topology_Wire& profile,double height,double profileTolerance = MyMath::Vector3::DefaultEpsilon);

// 使用指定可逆仿射变换创建闭合轮廓拉伸Shape实例。
Shape makeExtruded(const Topology_Wire& profile,double height,const MyMath::Matrix4& localToWorld,double profileTolerance = MyMath::Vector3::DefaultEpsilon);

// 使用Wire实例的同一空间放置创建闭合轮廓拉伸Shape实例。
Shape makeExtruded(const Wire& profile,double height,double profileTolerance = MyMath::Vector3::DefaultEpsilon);

}
}

#endif // MYBREP_MODELING_SHAPE_SHAPEMODELING_H
