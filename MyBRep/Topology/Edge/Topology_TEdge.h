#ifndef MYBREP_TOPOLOGY_EDGE_TOPOLOGY_TEDGE_H
#define MYBREP_TOPOLOGY_EDGE_TOPOLOGY_TEDGE_H

#include <cstddef>
#include <vector>

#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Curve/Geometry_Curve.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Curve2D.h"
#include "MyBRep/Geometry/Surface/Geometry_Surface.h"
#include "MyBRep/Topology/Topology_TObject.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"

namespace MyBRep
{

class Topology_Builder;
class Topology_Edge;

// 保存一个共享拓扑边身份及其标准方向顶点，并统一管理该边的三维曲线和Curve-on-Surface几何表示。
//
// 三维主曲线定义Edge的标准Forward方向；每个曲面表示保存一条P-Curve，闭合曲面seam允许在同一Surface上保存第二条P-Curve。
// P-Curve拥有独立自然参数区间，不要求与三维主曲线使用相同参数化。
class Topology_TEdge : public Topology_TObject
{
    friend class Topology_Builder;
    friend class Topology_Edge;

private:
    // 保存一条Edge在某个完整参数曲面上的二维曲线表示。
    struct CurveOnSurfaceRepresentation
    {
        Foundation::RefPtr<const Geometry_Surface> surface;     // 当前二维表示所属的完整参数曲面。
        Foundation::RefPtr<const Geometry_Curve2D> firstCurve;  // 普通表示或seam第一侧使用的二维参数曲线。
        double firstCurveFirstParameter;                        // firstCurve对应TEdge Forward起点的二维曲线参数。
        double firstCurveLastParameter;                         // firstCurve对应TEdge Forward终点的二维曲线参数。
        Foundation::RefPtr<const Geometry_Curve2D> secondCurve; // seam第二侧二维参数曲线，普通表示为空。
        double secondCurveFirstParameter;                       // secondCurve对应TEdge Forward起点的二维曲线参数。
        double secondCurveLastParameter;                        // secondCurve对应TEdge Forward终点的二维曲线参数。
    };

public:
    /// 拓扑数据

    // 返回标准Forward方向起始拓扑点。
    const Topology_Vertex& startVertex() const;
    // 返回标准Forward方向终止拓扑点。
    const Topology_Vertex& endVertex() const;

    /// 三维主曲线表示

    // 返回当前拓扑边引用的完整不可变三维参数曲线。
    const Geometry_Curve& geometry() const;
    // 返回当前拓扑边引用的完整不可变三维参数曲线资源。
    const Foundation::RefPtr<const Geometry_Curve>& geometryResource() const;
    // 返回标准Forward方向对应的三维曲线起始参数。
    double firstParameter() const;
    // 返回标准Forward方向对应的三维曲线终止参数。
    double lastParameter() const;

    /// Curve-on-Surface表示

    // 返回当前Edge已经保存的不同Surface表示数量。
    std::size_t curveOnSurfaceCount() const;
    // 返回指定Surface是否已经具有Curve-on-Surface表示，Surface按几何资源身份匹配。
    bool hasCurveOnSurface(const Geometry_Surface& surface) const;
    // 判断指定Surface上的表示是否包含两条P-Curve，即当前Edge是否在该Surface上作为seam使用。
    bool isSeamOnSurface(const Geometry_Surface& surface) const;

protected:
    // 使用两个拓扑点、完整三维曲线及严格递增有限参数区间创建共享拓扑边实体。
    Topology_TEdge(const Topology_Vertex& startVertex,
                   const Topology_Vertex& endVertex,
                   const Foundation::RefPtr<const Geometry_Curve>& geometry,
                   double firstParameter,
                   double lastParameter,
                   double connectionTolerance);

    // 通过Topology_Edge持有的最终共享引用释放拓扑边实体。
    ~Topology_TEdge() override = default;

private:
    // 返回指定Surface对应的Curve-on-Surface表示编号，不存在时返回curveOnSurfaceCount()。
    std::size_t findCurveOnSurface(const Geometry_Surface& surface) const;
    // 返回指定Surface对应的Curve-on-Surface表示。
    const CurveOnSurfaceRepresentation& curveOnSurface(const Geometry_Surface& surface) const;
    // 返回指定Surface对应的可修改Curve-on-Surface表示。
    CurveOnSurfaceRepresentation& curveOnSurface(const Geometry_Surface& surface);

    // 增加普通Curve-on-Surface表示，调用前必须保证该Surface尚不存在表示。
    void addCurveOnSurface(const Foundation::RefPtr<const Geometry_Surface>& surface,
                           const Foundation::RefPtr<const Geometry_Curve2D>& curve,
                           double curveFirstParameter,
                           double curveLastParameter);
    // 增加闭合Surface seam表示，调用前必须保证该Surface尚不存在表示。
    void addCurveOnClosedSurface(const Foundation::RefPtr<const Geometry_Surface>& surface,
                                 const Foundation::RefPtr<const Geometry_Curve2D>& firstCurve,
                                 double firstCurveFirstParameter,
                                 double firstCurveLastParameter,
                                 const Foundation::RefPtr<const Geometry_Curve2D>& secondCurve,
                                 double secondCurveFirstParameter,
                                 double secondCurveLastParameter);

private:
    Topology_Vertex m_startVertex; // 标准Forward方向起始拓扑点。
    Topology_Vertex m_endVertex; // 标准Forward方向终止拓扑点。
    Foundation::RefPtr<const Geometry_Curve> m_geometry; // 当前边的完整不可变三维主曲线。
    double m_firstParameter; // 标准Forward方向对应的三维曲线起始参数。
    double m_lastParameter; // 标准Forward方向对应的三维曲线终止参数。
    std::vector<CurveOnSurfaceRepresentation> m_curveOnSurfaces; // 当前Edge在各完整Surface上的二维参数曲线表示。
};

}

#endif // MYBREP_TOPOLOGY_EDGE_TOPOLOGY_TEDGE_H
