#ifndef MYBREP_MODELING_FACE_FACEMODELING_H
#define MYBREP_MODELING_FACE_FACEMODELING_H

#include <vector>

#include "MyMath/CoordinateSystem.h"
#include "MyMath/Matrix4.h"
#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Surface/Geometry_Surface.h"
#include "MyBRep/Instance/Face.h"
#include "MyBRep/Topology/Face/Topology_Face.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

namespace MyBRep
{
namespace Modeling
{

/// 通用Topology_Face创建

// 使用完整参数曲面和已经具有对应Curve-on-Surface表示的闭合Wire创建Topology_Face。
Topology_Face createFace(const Foundation::RefPtr<const Geometry_Surface>& surface,const std::vector<Topology_Wire>& wires);

// 使用完整参数曲面创建不含显式裁剪Wire的Topology_Face。
Topology_Face createFace(const Foundation::RefPtr<const Geometry_Surface>& surface);

/// 平面Topology_Face创建

// 使用世界XY平面和指定闭合Wire序列创建Planar Face，并自动建立全部Edge的二维参数曲线表示。
Topology_Face createPlanarFace(const std::vector<Topology_Wire>& wires,double tolerance = MyMath::Vector3::DefaultEpsilon);

// 使用指定正交坐标系XY平面和闭合Wire序列创建Planar Face，并自动建立全部Edge的二维参数曲线表示。
Topology_Face createPlanarFace(
    const MyMath::CoordinateSystem& coordinateSystem,
    const std::vector<Topology_Wire>& wires,
    double tolerance = MyMath::Vector3::DefaultEpsilon);

// 使用世界XY平面和单个闭合Wire创建Planar Face。
Topology_Face createPlanarFace(
    const Topology_Wire& wire,
    double tolerance = MyMath::Vector3::DefaultEpsilon);

// 使用指定正交坐标系XY平面和单个闭合Wire创建Planar Face。
Topology_Face createPlanarFace(
    const MyMath::CoordinateSystem& coordinateSystem,
    const Topology_Wire& wire,
    double tolerance = MyMath::Vector3::DefaultEpsilon);

/// 空间Face实例创建

// 使用单位变换创建通用Face实例。
Face makeFace(
    const Foundation::RefPtr<const Geometry_Surface>& surface,
    const std::vector<Topology_Wire>& wires);

// 使用指定可逆仿射变换创建通用Face实例。
Face makeFace(
    const Foundation::RefPtr<const Geometry_Surface>& surface,
    const std::vector<Topology_Wire>& wires,
    const MyMath::Matrix4& localToWorld);

// 使用单位变换创建世界XY平面Planar Face实例。
Face makePlanarFace(
    const std::vector<Topology_Wire>& wires,
    double tolerance = MyMath::Vector3::DefaultEpsilon);

// 使用指定可逆仿射变换创建世界XY平面Planar Face实例。
Face makePlanarFace(
    const std::vector<Topology_Wire>& wires,
    const MyMath::Matrix4& localToWorld,
    double tolerance = MyMath::Vector3::DefaultEpsilon);

// 使用单位变换创建指定坐标系平面Planar Face实例。
Face makePlanarFace(
    const MyMath::CoordinateSystem& coordinateSystem,
    const std::vector<Topology_Wire>& wires,
    double tolerance = MyMath::Vector3::DefaultEpsilon);

// 使用指定可逆仿射变换创建指定坐标系平面Planar Face实例。
Face makePlanarFace(
    const MyMath::CoordinateSystem& coordinateSystem,
    const std::vector<Topology_Wire>& wires,
    const MyMath::Matrix4& localToWorld,
    double tolerance = MyMath::Vector3::DefaultEpsilon);

}
}

#endif // MYBREP_MODELING_FACE_FACEMODELING_H
