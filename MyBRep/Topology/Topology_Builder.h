#ifndef MYBREP_TOPOLOGY_TOPOLOGY_BUILDER_H
#define MYBREP_TOPOLOGY_TOPOLOGY_BUILDER_H

#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Curve2D.h"
#include "MyBRep/Geometry/Surface/Geometry_Surface.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"

namespace MyBRep
{

// 提供共享拓扑实体构造和重建阶段的受控写入口，公开Topology_Object句柄本身保持只读语义。
class Topology_Builder
{
public:
    /// Edge曲面表示

    // 为共享Forward Edge增加指定Surface上的单条二维参数曲线表示。
    static void addCurveOnSurface(Topology_Edge& edge, const Foundation::RefPtr<const Geometry_Surface>& surface,
                                  const Foundation::RefPtr<const Geometry_Curve2D>& curve, double curveFirstParameter,
                                  double curveLastParameter, double tolerance = MyMath::Vector3::DefaultEpsilon);

    // 为共享Forward Edge增加闭合Surface seam上的两条二维参数曲线表示。
    static void addCurveOnClosedSurface(Topology_Edge& edge, const Foundation::RefPtr<const Geometry_Surface>& surface,
                                        const Foundation::RefPtr<const Geometry_Curve2D>& firstCurve,
                                        double firstCurveFirstParameter, double firstCurveLastParameter,
                                        const Foundation::RefPtr<const Geometry_Curve2D>& secondCurve,
                                        double secondCurveFirstParameter, double secondCurveLastParameter,
                                        double tolerance = MyMath::Vector3::DefaultEpsilon);

    // 将源Forward Edge在规范化区间[firstParameter,lastParameter]上的全部Curve-on-Surface表示复制到目标Forward Edge。
    //
    // 源和目标必须共享同一个Geometry_Curve资源；该接口只裁剪表示参数范围，不复制Geometry_Surface或Geometry_Curve2D资源。
    static void copyCurveOnSurfaceRange(const Topology_Edge& sourceEdge, double firstParameter, double lastParameter,
                                        Topology_Edge& targetEdge);

private:
    // 校验一条二维参数曲线区间能够把TEdge标准Forward起终点映射到指定Surface。
    static void validateCurveOnSurface(const Topology_TEdge& edge, const Geometry_Surface& surface,
                                       const Geometry_Curve2D& curve, double curveFirstParameter,
                                       double curveLastParameter, double tolerance);
};

}

#endif // MYBREP_TOPOLOGY_TOPOLOGY_BUILDER_H
