#ifndef MYBREP_MODELING_FACE_FACEMODELING_H
#define MYBREP_MODELING_FACE_FACEMODELING_H

#include <vector>

#include "MyMath/CoordinateSystem.h"
#include "MyMath/Matrix4.h"
#include "MyMath/Vector2.h"
#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Surface/Geometry_Surface.h"
#include "MyBRep/Instance/Face.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Face/Topology_Face.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

namespace MyBRep
{

class Geometry_Curve2D;
class Geometry_PlaneSurface;

namespace Modeling
{

namespace FaceModelingDetail
{

// 返回三维点在指定Plane参数空间中的(U,V)坐标，并校验点到Plane的距离不超过tolerance。
MyMath::Vector2 pointToUV(const Geometry_PlaneSurface& plane,const MyMath::Vector3& point,double tolerance);

// 返回三维向量在指定Plane U/V正交基中的二维分量。
MyMath::Vector2 vectorToUV(const Geometry_PlaneSurface& plane,const MyMath::Vector3& vector);

// 返回用于向Topology_Builder写入标准TEdge几何表示的Forward Edge句柄。
Topology_Edge forwardEdge(const Topology_Edge& edge);

// 从平面内三维Edge创建对应完整二维参数曲线，并返回二维自然参数区间。
Foundation::RefPtr<const Geometry_Curve2D> createPlanarCurve2D(
    const Topology_Edge& edge,
    const Geometry_PlaneSurface& plane,
    double tolerance,
    double& firstParameter,
    double& lastParameter);

// 为全部Wire Edge建立当前Plane上的Curve-on-Surface表示。
void attachPlanarCurveRepresentations(
    const Foundation::RefPtr<const Geometry_Surface>& surface,
    const Geometry_PlaneSurface& plane,
    const std::vector<Topology_Wire>& wires,
    double tolerance);

}

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
Topology_Face createPlanarFace(const Topology_Wire& wire,double tolerance = MyMath::Vector3::DefaultEpsilon);

// 使用指定正交坐标系XY平面和单个闭合Wire创建Planar Face。
Topology_Face createPlanarFace(
    const MyMath::CoordinateSystem& coordinateSystem,
    const Topology_Wire& wire,
    double tolerance = MyMath::Vector3::DefaultEpsilon);

/// 空间Face实例创建

// 使用完整参数曲面和单位变换创建不含显式裁剪Wire的Face实例。
Face makeFace(const Foundation::RefPtr<const Geometry_Surface>& surface);

// 使用完整参数曲面和指定可逆仿射变换创建不含显式裁剪Wire的Face实例。
Face makeFace(const Foundation::RefPtr<const Geometry_Surface>& surface,const MyMath::Matrix4& localToWorld);

// 使用单位变换创建通用Face实例。
Face makeFace(const Foundation::RefPtr<const Geometry_Surface>& surface,const std::vector<Topology_Wire>& wires);

// 使用指定可逆仿射变换创建通用Face实例。
Face makeFace(
    const Foundation::RefPtr<const Geometry_Surface>& surface,
    const std::vector<Topology_Wire>& wires,
    const MyMath::Matrix4& localToWorld);

// 使用单位变换创建世界XY平面Planar Face实例。
Face makePlanarFace(const std::vector<Topology_Wire>& wires,double tolerance = MyMath::Vector3::DefaultEpsilon);

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

// 使用单位变换创建世界XY平面单Wire Planar Face实例。
Face makePlanarFace(const Topology_Wire& wire,double tolerance = MyMath::Vector3::DefaultEpsilon);

// 使用指定可逆仿射变换创建世界XY平面单Wire Planar Face实例。
Face makePlanarFace(
    const Topology_Wire& wire,
    const MyMath::Matrix4& localToWorld,
    double tolerance = MyMath::Vector3::DefaultEpsilon);

// 使用单位变换创建指定坐标系平面的单Wire Planar Face实例。
Face makePlanarFace(
    const MyMath::CoordinateSystem& coordinateSystem,
    const Topology_Wire& wire,
    double tolerance = MyMath::Vector3::DefaultEpsilon);

// 使用指定可逆仿射变换创建指定坐标系平面的单Wire Planar Face实例。
Face makePlanarFace(
    const MyMath::CoordinateSystem& coordinateSystem,
    const Topology_Wire& wire,
    const MyMath::Matrix4& localToWorld,
    double tolerance = MyMath::Vector3::DefaultEpsilon);

}
}

#endif // MYBREP_MODELING_FACE_FACEMODELING_H
