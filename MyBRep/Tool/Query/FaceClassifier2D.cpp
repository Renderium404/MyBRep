#include "FaceClassifier2D.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "MyBRep/Foundation/Diagnostic.h"
#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Curve2D/Geometry_BSpline2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Bezier2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Circle2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Line2D.h"
#include "MyBRep/Operation/Intersection/CurveCurve2DIntersection.h"
#include "MyBRep/Operation/Intersection/WireCurve2DIntersection.h"

namespace
{

const double RayPerturbationScale = 128.0; // 水平射线纵坐标避开顶点和水平边界退化时覆盖浮点舍入误差使用的固定倍数。
const double CrossingSampleScale = 1.0e-6; // 判断交点两侧是否真正跨越水平射线时使用的Edge规范化初始采样距离。
const double MaximumCrossingSample = 1.0e-2; // 局部跨越判断最大只扩展到Edge规范化长度的1%，避免跨过其他局部特征。
const int MaximumRayAttempts = 6; // 极少数射线与水平边界重合时最多更换6个微小纵向偏移。

struct BoundaryXRange
{
    BoundaryXRange()
        : minimum((std::numeric_limits<double>::infinity)())
        , maximum(-(std::numeric_limits<double>::infinity)())
    {
    }

    double minimum;
    double maximum;
};

// 判断标量是否为有限值。
bool isFiniteValue(double value)
{
    const double infinity = (std::numeric_limits<double>::infinity)();
    return value == value && value != infinity && value != -infinity;
}

// 判断标量是否为有限非负值。
bool isFiniteNonNegative(double value)
{
    return isFiniteValue(value) && value >= 0.0;
}

// 将二维点X坐标包含进边界范围。
void includeX(BoundaryXRange& range, const MyMath::Vector2& point)
{
    range.minimum = (std::min)(range.minimum, point.x());
    range.maximum = (std::max)(range.maximum, point.x());
}

// 将指定有限P-Curve使用的保守X范围包含进边界范围。
void includeCurveXRange(BoundaryXRange& range, const MyBRep::Geometry_Curve2D& curve, double firstParameter, double lastParameter)
{
    includeX(range, curve.pointAt(firstParameter));
    includeX(range, curve.pointAt(lastParameter));

    if (curve.kind() == MyBRep::CurveKind::Circle)
    {
        const MyBRep::Geometry_Circle2D& circle = static_cast<const MyBRep::Geometry_Circle2D&>(curve);
        range.minimum = (std::min)(range.minimum, circle.center().x() - circle.radius());
        range.maximum = (std::max)(range.maximum, circle.center().x() + circle.radius());
        return;
    }

    if (curve.kind() == MyBRep::CurveKind::Bezier)
    {
        const MyBRep::Geometry_Bezier2D& bezier = static_cast<const MyBRep::Geometry_Bezier2D&>(curve);

        for (std::size_t index = 0; index < bezier.controlPointCount(); ++index)
        {
            includeX(range, bezier.controlPoint(index));
        }

        return;
    }

    if (curve.kind() == MyBRep::CurveKind::BSpline)
    {
        const MyBRep::Geometry_BSpline2D& spline = static_cast<const MyBRep::Geometry_BSpline2D&>(curve);

        for (std::size_t index = 0; index < spline.controlPointCount(); ++index)
        {
            includeX(range, spline.controlPoint(index));
        }
    }
}

// 返回Face全部trimming P-Curve的保守X范围。
BoundaryXRange boundaryXRange(const MyBRep::Topology_Face& face)
{
    BoundaryXRange range;

    for (std::size_t wireIndex = 0; wireIndex < face.wireCount(); ++wireIndex)
    {
        const MyBRep::Topology_Wire wire = face.wire(wireIndex);

        for (std::size_t edgeIndex = 0; edgeIndex < wire.edgeCount(); ++edgeIndex)
        {
            const MyBRep::Topology_Edge edge = wire.edge(edgeIndex);
            const MyBRep::Geometry_Curve2D& curve = edge.curveOnSurface(face.geometry());
            includeCurveXRange(range, curve, edge.curveOnSurfaceFirstParameter(face.geometry()), edge.curveOnSurfaceLastParameter(face.geometry()));
        }
    }

    MYBREP_ASSERT_MESSAGE(isFiniteValue(range.minimum) && isFiniteValue(range.maximum) && range.minimum <= range.maximum,
                          "FaceClassifier2D failed to build a finite trimming boundary X range.");
    return range;
}

// 返回点到有限二维线段的最短距离。
double pointSegmentDistance(const MyMath::Vector2& point, const MyMath::Vector2& start, const MyMath::Vector2& end)
{
    const MyMath::Vector2 segment = end - start;
    const double lengthSquared = MyMath::Vector2::dot(segment, segment);

    if (lengthSquared == 0.0)
    {
        return (point - start).length();
    }

    double parameter = MyMath::Vector2::dot(point - start, segment) / lengthSquared;
    parameter = (std::max)(0.0, (std::min)(1.0, parameter));
    return (point - (start + segment * parameter)).length();
}

// 判断UV点是否落在指定有限P-Curve使用的容差边界带内。
bool pointOnCurveUse(const MyBRep::Geometry_Curve2D& curve, double firstParameter, double lastParameter,
                     const MyMath::Vector2& point, const BoundaryXRange& range, double tolerance)
{
    if (curve.kind() == MyBRep::CurveKind::Line)
    {
        return pointSegmentDistance(point, curve.pointAt(firstParameter), curve.pointAt(lastParameter)) <= tolerance;
    }

    double scale = 1.0;
    scale = (std::max)(scale, std::fabs(range.minimum));
    scale = (std::max)(scale, std::fabs(range.maximum));
    scale = (std::max)(scale, std::fabs(point.x()));
    scale = (std::max)(scale, std::fabs(point.y()));

    const double span = (std::max)(1.0, (std::max)(std::fabs(point.x() - range.minimum), std::fabs(range.maximum - point.x())) + scale);
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> horizontal(
        new MyBRep::Geometry_Line2D(MyMath::Vector2(point.x() - span, point.y()), MyMath::Vector2::unitX()));

    const MyBRep::Operation::Intersection::CurveCurve2DIntersectionResult result =
        MyBRep::Operation::Intersection::intersectCurveCurve2D(curve, firstParameter, lastParameter, *horizontal, 0.0, span * 2.0, tolerance);

    if (result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Overlap)
    {
        return true;
    }

    for (std::size_t index = 0; index < result.points.size(); ++index)
    {
        if ((result.points[index].point - point).length() <= tolerance)
        {
            return true;
        }
    }

    return false;
}

// 判断UV点是否落在Face任意trimming边界的容差带内。
bool pointOnBoundary(const MyBRep::Topology_Face& face, const MyMath::Vector2& point, const BoundaryXRange& range, double tolerance)
{
    for (std::size_t wireIndex = 0; wireIndex < face.wireCount(); ++wireIndex)
    {
        const MyBRep::Topology_Wire wire = face.wire(wireIndex);

        for (std::size_t edgeIndex = 0; edgeIndex < wire.edgeCount(); ++edgeIndex)
        {
            const MyBRep::Topology_Edge edge = wire.edge(edgeIndex);
            const MyBRep::Geometry_Curve2D& curve = edge.curveOnSurface(face.geometry());

            if (pointOnCurveUse(curve, edge.curveOnSurfaceFirstParameter(face.geometry()), edge.curveOnSurfaceLastParameter(face.geometry()),
                                point, range, tolerance))
            {
                return true;
            }
        }
    }

    return false;
}

// 返回指定Edge规范化参数附近一点相对于水平射线的V方向符号。
double edgeSide(const MyBRep::Topology_Edge& edge, const MyBRep::Geometry_Surface& surface, double parameter, double rayV)
{
    return edge.surfaceParameterAt(surface, parameter).y() - rayV;
}

// 判断Wire离散命中是否真正从水平射线一侧跨到另一侧；切触只接触不计奇偶翻转。
bool crossesHorizontalRay(const MyBRep::Topology_Wire& wire, const MyBRep::Geometry_Surface& surface,
                          const MyBRep::Operation::Intersection::WireCurve2DIntersectionPoint& hit, double rayV, double scale)
{
    double sample = CrossingSampleScale;
    const double sideTolerance = (std::max)(1.0, scale) * (std::numeric_limits<double>::epsilon)() * RayPerturbationScale;

    for (; sample <= MaximumCrossingSample; sample *= 2.0)
    {
        double beforeSide = 0.0;
        double afterSide = 0.0;

        if (hit.edgeParameter <= sample)
        {
            const std::size_t previousIndex = (hit.edgeIndex + wire.edgeCount() - 1) % wire.edgeCount();
            beforeSide = edgeSide(wire.edge(previousIndex), surface, 1.0 - sample, rayV);
            afterSide = edgeSide(wire.edge(hit.edgeIndex), surface, sample, rayV);
        }
        else if (hit.edgeParameter >= 1.0 - sample)
        {
            const std::size_t nextIndex = (hit.edgeIndex + 1) % wire.edgeCount();
            beforeSide = edgeSide(wire.edge(hit.edgeIndex), surface, 1.0 - sample, rayV);
            afterSide = edgeSide(wire.edge(nextIndex), surface, sample, rayV);
        }
        else
        {
            beforeSide = edgeSide(wire.edge(hit.edgeIndex), surface, hit.edgeParameter - sample, rayV);
            afterSide = edgeSide(wire.edge(hit.edgeIndex), surface, hit.edgeParameter + sample, rayV);
        }

        if (std::fabs(beforeSide) <= sideTolerance || std::fabs(afterSide) <= sideTolerance)
        {
            continue;
        }

        return (beforeSide < 0.0) != (afterSide < 0.0);
    }

    return false;
}

// 使用一个经过轻微V扰动的向右水平射线计算全部trimming Wire的even-odd奇偶值；与边界连续重合时返回false要求更换射线。
bool rayParity(const MyBRep::Topology_Face& face, const MyMath::Vector2& point, const BoundaryXRange& range,
               double rayV, double rayIntersectionTolerance, bool& inside)
{
    double scale = 1.0;
    scale = (std::max)(scale, std::fabs(range.minimum));
    scale = (std::max)(scale, std::fabs(range.maximum));
    scale = (std::max)(scale, std::fabs(point.x()));
    scale = (std::max)(scale, std::fabs(point.y()));

    if (point.x() > range.maximum + rayIntersectionTolerance)
    {
        inside = false;
        return true;
    }

    const double rayLength = range.maximum - point.x() + scale + 1.0;
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> ray(
        new MyBRep::Geometry_Line2D(MyMath::Vector2(point.x(), rayV), MyMath::Vector2::unitX()));

    bool parity = false;

    for (std::size_t wireIndex = 0; wireIndex < face.wireCount(); ++wireIndex)
    {
        const MyBRep::Topology_Wire wire = face.wire(wireIndex);
        const MyBRep::Operation::Intersection::WireCurve2DIntersectionResult result =
            MyBRep::Operation::Intersection::intersectWireCurve2D(wire, face.geometry(), *ray, 0.0, rayLength, rayIntersectionTolerance);

        if (result.kind == MyBRep::Operation::Intersection::CurveCurve2DIntersectionKind::Overlap)
        {
            return false;
        }

        for (std::size_t index = 0; index < result.points.size(); ++index)
        {
            if (crossesHorizontalRay(wire, face.geometry(), result.points[index], rayV, scale))
            {
                parity = !parity;
            }
        }
    }

    inside = parity;
    return true;
}

}

namespace MyBRep
{

FaceUVClassification classifyFaceUV(const Topology_Face& face, const MyMath::Vector2& parameter, double tolerance)
{
    MYBREP_ASSERT_MESSAGE(face.isValid(), "FaceClassifier2D requires a valid Topology_Face.");
    MYBREP_ASSERT_MESSAGE(parameter.isFinite(), "FaceClassifier2D requires a finite UV parameter.");
    MYBREP_ASSERT_MESSAGE(isFiniteNonNegative(tolerance), "FaceClassifier2D tolerance must be finite and non-negative.");

    const Geometry_Surface& surface = face.geometry();

    if (!surface.isParameterInDomain(parameter.x(), parameter.y()))
    {
        return FaceUVClassification::Outside;
    }

    if (face.wireCount() == 0)
    {
        return FaceUVClassification::Inside;
    }

    MYBREP_ASSERT_MESSAGE(!surface.isUPeriodic() && !surface.isVPeriodic(),
                          "FaceClassifier2D v1 requires a single non-periodic UV parameter chart for trimmed Faces.");

    const BoundaryXRange range = boundaryXRange(face);

    if (pointOnBoundary(face, parameter, range, tolerance))
    {
        return FaceUVClassification::Boundary;
    }

    double scale = 1.0;
    scale = (std::max)(scale, std::fabs(range.minimum));
    scale = (std::max)(scale, std::fabs(range.maximum));
    scale = (std::max)(scale, std::fabs(parameter.x()));
    scale = (std::max)(scale, std::fabs(parameter.y()));

    const double numericalOffset = scale * (std::numeric_limits<double>::epsilon)() * RayPerturbationScale;
    const double rayIntersectionTolerance = (std::max)(numericalOffset * 8.0, tolerance * 1.0e-4); // 奇偶射线使用远小于边界分类容差的内部求交容差，避免近水平边被误判为连续重合。
    const double rayOffset = (std::max)(numericalOffset, rayIntersectionTolerance * 4.0);
    const double offsets[MaximumRayAttempts] =
    {
        rayOffset,
        -rayOffset,
        rayOffset * 2.0,
        -rayOffset * 2.0,
        rayOffset * 4.0,
        -rayOffset * 4.0
    };

    bool inside = false;

    for (int attempt = 0; attempt < MaximumRayAttempts; ++attempt)
    {
        if (rayParity(face, parameter, range, parameter.y() + offsets[attempt], rayIntersectionTolerance, inside))
        {
            return inside ? FaceUVClassification::Inside : FaceUVClassification::Outside;
        }
    }

    MYBREP_ASSERT_MESSAGE(false, "FaceClassifier2D failed to find a non-degenerate horizontal classification ray.");
    return FaceUVClassification::Outside;
}

}