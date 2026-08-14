#ifndef MYBREP_TOPOLOGY_EDGE_TOPOLOGY_EDGE_H
#define MYBREP_TOPOLOGY_EDGE_TOPOLOGY_EDGE_H

#include <cstddef>

#include "MyMath/Vector2.h"
#include "MyMath/Vector3.h"
#include "MyBRep/Geometry/Curve/Geometry_Curve.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Curve2D.h"
#include "MyBRep/Geometry/Surface/Geometry_Surface.h"
#include "MyBRep/Topology/Topology_Object.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"
#include "Topology_TEdge.h"

namespace MyBRep
{

class Topology_Builder;

// 作为共享拓扑边实体的轻量公开句柄，有限边由完整三维曲线和参数区间共同确定。
//
// Edge公开参数始终规范化为[0,1]；当前使用方向决定起终点、三维曲线参数映射以及Curve-on-Surface二维参数方向。
// 同一Edge可以在多个Surface上保存二维参数曲线表示，闭合Surface seam允许同一Surface保存两条P-Curve。
class Topology_Edge : public Topology_Object
{
    friend class Topology_Builder;

public:
    // 构造空拓扑边句柄。
    Topology_Edge();

    // 使用起终拓扑点、完整三维曲线及严格递增有限参数区间创建新的Forward拓扑边身份。
    Topology_Edge(const Topology_Vertex& startVertex,
                  const Topology_Vertex& endVertex,
                  const Foundation::RefPtr<const Geometry_Curve>& geometry,
                  double firstParameter,
                  double lastParameter,
                  double connectionTolerance = MyMath::Vector3::DefaultEpsilon);

    /// 拓扑数据

    // 返回当前使用方向的起始拓扑点。
    const Topology_Vertex& startVertex() const;
    // 返回当前使用方向的终止拓扑点。
    const Topology_Vertex& endVertex() const;

    /// 三维主曲线表示

    // 返回底层拓扑边引用的完整不可变三维参数曲线，句柄反向不会复制或反转该几何资源。
    const Geometry_Curve& geometry() const;
    // 返回底层拓扑边引用的完整不可变三维参数曲线资源。
    const Foundation::RefPtr<const Geometry_Curve>& geometryResource() const;
    // 返回当前使用方向起点对应的三维曲线自然参数。
    double firstParameter() const;
    // 返回当前使用方向终点对应的三维曲线自然参数。
    double lastParameter() const;
    // 将当前使用方向规范化Edge参数[0,1]映射为底层三维曲线自然参数。
    double curveParameterAt(double parameter) const;
    // 返回当前使用方向规范化参数对应的三维曲线点。
    MyMath::Vector3 pointAt(double parameter) const;
    // 返回当前使用方向规范化参数对应的三维单位切向量。
    MyMath::Vector3 tangentAt(double parameter) const;

    /// Curve-on-Surface表示

    // 返回当前Edge已经保存的不同Surface表示数量。
    std::size_t curveOnSurfaceCount() const;
    // 判断当前Edge是否具有指定完整Surface上的二维参数曲线表示。
    bool hasCurveOnSurface(const Geometry_Surface& surface) const;
    // 判断当前Edge在指定Surface上是否具有两条P-Curve，即是否作为该闭合Surface的seam使用。
    bool isSeamOnSurface(const Geometry_Surface& surface) const;
    // 返回当前使用方向在指定Surface上应采用的完整二维参数曲线。
    const Geometry_Curve2D& curveOnSurface(const Geometry_Surface& surface) const;
    // 返回当前使用方向在指定Surface上应采用的完整二维参数曲线资源。
    const Foundation::RefPtr<const Geometry_Curve2D>& curveOnSurfaceResource(const Geometry_Surface& surface) const;
    // 返回当前使用方向起点对应的二维曲线自然参数。
    double curveOnSurfaceFirstParameter(const Geometry_Surface& surface) const;
    // 返回当前使用方向终点对应的二维曲线自然参数。
    double curveOnSurfaceLastParameter(const Geometry_Surface& surface) const;
    // 将当前使用方向规范化Edge参数[0,1]映射为指定Surface上P-Curve的自然参数。
    double curveOnSurfaceParameterAt(const Geometry_Surface& surface, double parameter) const;
    // 返回当前Edge在指定Surface参数空间中的二维UV点。
    MyMath::Vector2 surfaceParameterAt(const Geometry_Surface& surface, double parameter) const;

    /// 拓扑创建

    // 返回共享同一Topology_TEdge身份但使用方向相反的拓扑边句柄。
    Topology_Edge reversed() const;

private:
    // 使用已有共享拓扑边身份和指定方向创建句柄。
    Topology_Edge(const Foundation::RefPtr<Topology_TObject>& object,
                  Topology_Orientation orientation);

    // 返回当前句柄引用的强类型共享拓扑边实体。
    const Topology_TEdge& tEdge() const;
    // 返回当前句柄引用的可修改强类型共享拓扑边实体，仅供Topology_Builder构造阶段使用。
    Topology_TEdge& mutableTEdge() const;
};

}

#endif // MYBREP_TOPOLOGY_EDGE_TOPOLOGY_EDGE_H
