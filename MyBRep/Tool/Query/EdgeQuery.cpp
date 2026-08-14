#include "EdgeQuery.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "MyBRep/Foundation/Diagnostic.h"
#include "MyBRep/Geometry/Curve/Geometry_BSpline.h"
#include "MyBRep/Geometry/Curve/Geometry_Bezier.h"
#include "MyBRep/Geometry/Curve/Geometry_Circle.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795; // 圆参数极值计算使用的圆周率。
const double TwoPi = Pi * 2.0; // 完整圆参数周期。
const double SimpsonErrorScale = 15.0; // 自适应Simpson误差修正分母来自经典五阶误差估计。

// 判断标量是否为有限正数。
bool isFinitePositive(double value)
{
    const double infinity = (std::numeric_limits<double>::infinity)();
    return value == value &&
           value != infinity &&
           value != -infinity &&
           value > 0.0;
}

// 返回Simpson三点公式在[first,last]上的积分值。
double simpsonValue(double first,
                    double last,
                    double firstValue,
                    double middleValue,
                    double lastValue)
{
    return (last - first) *
           (firstValue + 4.0 * middleValue + lastValue) /
           6.0;
}

// 判断周期参数candidate经过整数个period平移后是否落入闭区间[first,last]。
bool periodicParameterInInterval(double candidate,
                                 double period,
                                 double first,
                                 double last)
{
    const double minimumParameter = (std::min)(first, last);
    const double maximumParameter = (std::max)(first, last);
    const double shiftCount =
        std::ceil((minimumParameter - candidate) / period);
    const double shiftedCandidate =
        candidate + shiftCount * period;

    return shiftedCandidate >= minimumParameter &&
           shiftedCandidate <= maximumParameter;
}

// 将指定点包含进Bounds3。
void includePoint(MyBRep::Bounds3& bounds,
                  const MyMath::Vector3& point)
{
    MYBREP_ASSERT_MESSAGE(point.isFinite(),
                          "EdgeQuery bounds point must be finite.");

    bounds.include(point);
}

// 返回Vector3指定轴分量。
double component(const MyMath::Vector3& value, int axis)
{
    if (axis == 0)
    {
        return value.x();
    }

    if (axis == 1)
    {
        return value.y();
    }

    return value.z();
}

}

namespace MyBRep
{

EdgeQuery::EdgeQuery(const Topology_Edge& topology)
    : m_queryToLocal(MyMath::Matrix4::identity())
    , m_localToQuery(MyMath::Matrix4::identity())
{
    MYBREP_ASSERT_MESSAGE(topology.isValid(),
                          "EdgeQuery requires a valid Topology_Edge.");

    initialize(Edge(topology), MyMath::Matrix4::identity());
}

EdgeQuery::EdgeQuery(const Edge& edge)
    : m_queryToLocal(MyMath::Matrix4::identity())
    , m_localToQuery(MyMath::Matrix4::identity())
{
    initialize(edge, MyMath::Matrix4::identity());
}

EdgeQuery::EdgeQuery(const Edge& edge,
                     const MyMath::Matrix4& queryToWorld)
    : m_queryToLocal(MyMath::Matrix4::identity())
    , m_localToQuery(MyMath::Matrix4::identity())
{
    initialize(edge, queryToWorld);
}

/// 查询对象与空间数据

const Edge& EdgeQuery::edge() const
{
    return m_edge;
}

const Topology_Edge& EdgeQuery::topology() const
{
    return m_edge.topology();
}

const Geometry_Curve& EdgeQuery::geometry() const
{
    return m_edge.geometry();
}

const MyMath::Matrix4& EdgeQuery::queryToLocal() const
{
    return m_queryToLocal;
}

const MyMath::Matrix4& EdgeQuery::localToQuery() const
{
    return m_localToQuery;
}

const Bounds3& EdgeQuery::queryBounds() const
{
    return m_queryBounds;
}

/// 参数查询

double EdgeQuery::curveParameterAt(double parameter) const
{
    return topology().curveParameterAt(parameter);
}

MyMath::Vector3 EdgeQuery::pointAt(double parameter) const
{
    return m_localToQuery.transformPoint(
        topology().pointAt(parameter));
}

MyMath::Vector3 EdgeQuery::tangentAt(double parameter) const
{
    const MyMath::Vector3 queryTangent =
        m_localToQuery.transformVector(
            topology().tangentAt(parameter));

    MYBREP_ASSERT_MESSAGE(queryTangent.isVector(0.0),
                          "EdgeQuery tangent must remain non-zero under an invertible affine transform.");

    return queryTangent.normalized(0.0);
}

/// 线性度量

double EdgeQuery::length(double absoluteTolerance,
                         unsigned int maxSubdivisionDepth) const
{
    MYBREP_ASSERT_MESSAGE(isFinitePositive(absoluteTolerance),
                          "EdgeQuery length tolerance must be finite and positive.");
    MYBREP_ASSERT_MESSAGE(maxSubdivisionDepth > 0,
                          "EdgeQuery maximum subdivision depth must be positive.");

    const double firstParameter = topology().firstParameter();
    const double lastParameter = topology().lastParameter();
    const double minimumParameter = (std::min)(firstParameter, lastParameter);
    const double maximumParameter = (std::max)(firstParameter, lastParameter);

    if (geometry().kind() == CurveKind::BSpline)
    {
        return integratePolylineLength(minimumParameter,
                                       maximumParameter,
                                       absoluteTolerance,
                                       maxSubdivisionDepth);
    }

    return integrateLength(minimumParameter,
                           maximumParameter,
                           absoluteTolerance,
                           maxSubdivisionDepth);
}

/// 初始化

void EdgeQuery::initialize(const Edge& edge,
                           const MyMath::Matrix4& queryToWorld)
{
    MYBREP_ASSERT_MESSAGE(edge.isValid(),
                          "EdgeQuery requires a valid Edge.");
    MYBREP_ASSERT_MESSAGE(queryToWorld.isAffine(),
                          "EdgeQuery query-to-world transform must be affine.");

    MyMath::Matrix4 worldToQuery;
    const bool invertible =
        queryToWorld.inverted(worldToQuery);

    MYBREP_ASSERT_MESSAGE(invertible,
                          "EdgeQuery query-to-world transform must be invertible.");

    m_edge = edge;
    m_queryToLocal =
        edge.worldToLocal() * queryToWorld;
    m_localToQuery =
        worldToQuery * edge.localToWorld();

    rebuildBounds();

    MYBREP_ASSERT_MESSAGE(m_queryBounds.isValid(),
                          "EdgeQuery finite Edge bounds must be valid.");
}

void EdgeQuery::rebuildBounds()
{
    m_queryBounds.clear();

    const double firstParameter = topology().firstParameter();
    const double lastParameter = topology().lastParameter();

    includePoint(m_queryBounds,
                 m_localToQuery.transformPoint(
                     geometry().pointAt(firstParameter)));
    includePoint(m_queryBounds,
                 m_localToQuery.transformPoint(
                     geometry().pointAt(lastParameter)));

    if (geometry().kind() == CurveKind::Line)
    {
        return;
    }

    if (geometry().kind() == CurveKind::Circle)
    {
        const Geometry_Circle& circle =
            static_cast<const Geometry_Circle&>(geometry());

        const MyMath::Vector3 localCenter = circle.center();
        const double radius = circle.radius();
        const MyMath::Vector3 localXDirection =
            (circle.pointAt(0.0) - localCenter).normalized(0.0);
        const MyMath::Vector3 localYDirection =
            circle.firstDerivativeAt(0.0).normalized(0.0);

        const MyMath::Vector3 queryCenter =
            m_localToQuery.transformPoint(localCenter);
        const MyMath::Vector3 queryXDirection =
            m_localToQuery.transformVector(localXDirection);
        const MyMath::Vector3 queryYDirection =
            m_localToQuery.transformVector(localYDirection);

        for (int axis = 0; axis < 3; ++axis)
        {
            const double cosineCoefficient =
                component(queryXDirection, axis);
            const double sineCoefficient =
                component(queryYDirection, axis);

            if (cosineCoefficient == 0.0 &&
                sineCoefficient == 0.0)
            {
                continue;
            }

            const double extremumParameter =
                std::atan2(sineCoefficient,
                           cosineCoefficient);

            const double candidates[2] =
            {
                extremumParameter,
                extremumParameter + Pi
            };

            for (int candidateIndex = 0;
                 candidateIndex < 2;
                 ++candidateIndex)
            {
                if (!periodicParameterInInterval(candidates[candidateIndex],
                                                 TwoPi,
                                                 firstParameter,
                                                 lastParameter))
                {
                    continue;
                }

                includePoint(
                    m_queryBounds,
                    m_localToQuery.transformPoint(
                        circle.pointAt(candidates[candidateIndex])));
            }
        }

        return;
    }

    if (geometry().kind() == CurveKind::Bezier)
    {
        const Geometry_Bezier& bezier =
            static_cast<const Geometry_Bezier&>(geometry());

        // Bezier曲线完全位于完整控制多边形凸包内；对裁剪Edge使用完整控制点得到保守但可能不紧的有限范围。
        for (std::size_t index = 0;
             index < bezier.controlPointCount();
             ++index)
        {
            includePoint(
                m_queryBounds,
                m_localToQuery.transformPoint(
                    bezier.controlPoint(index)));
        }

        return;
    }

    if (geometry().kind() == CurveKind::BSpline)
    {
        const Geometry_BSpline& spline =
            static_cast<const Geometry_BSpline&>(geometry());

        // 非有理B样条曲线完全位于完整控制点凸包内；对裁剪Edge使用完整控制网得到保守但可能不紧的有限范围。
        for (std::size_t index = 0;
             index < spline.controlPointCount();
             ++index)
        {
            includePoint(
                m_queryBounds,
                m_localToQuery.transformPoint(
                    spline.controlPoint(index)));
        }

        return;
    }

    MYBREP_ASSERT_MESSAGE(false,
                          "EdgeQuery bounds do not support the current Geometry_Curve kind.");
}

/// 数值积分

double EdgeQuery::speedAtCurveParameter(double curveParameter) const
{
    const MyMath::Vector3 queryDerivative =
        m_localToQuery.transformVector(
            geometry().firstDerivativeAt(curveParameter));

    MYBREP_ASSERT_MESSAGE(queryDerivative.isFinite(),
                          "EdgeQuery transformed curve derivative must be finite.");

    return queryDerivative.length();
}

double EdgeQuery::integrateLength(double firstParameter,
                                  double lastParameter,
                                  double absoluteTolerance,
                                  unsigned int maxSubdivisionDepth) const
{
    struct Integrator
    {
        const EdgeQuery* query;

        double integrate(double first,
                         double last,
                         double firstValue,
                         double middleValue,
                         double lastValue,
                         double whole,
                         double tolerance,
                         unsigned int remainingDepth) const
        {
            const double middle =
                (first + last) * 0.5;
            const double leftMiddle =
                (first + middle) * 0.5;
            const double rightMiddle =
                (middle + last) * 0.5;

            const double leftMiddleValue =
                query->speedAtCurveParameter(leftMiddle);
            const double rightMiddleValue =
                query->speedAtCurveParameter(rightMiddle);

            const double left =
                simpsonValue(first,
                             middle,
                             firstValue,
                             leftMiddleValue,
                             middleValue);
            const double right =
                simpsonValue(middle,
                             last,
                             middleValue,
                             rightMiddleValue,
                             lastValue);
            const double refined =
                left + right;
            const double error =
                std::fabs(refined - whole);

            if (error <= SimpsonErrorScale * tolerance)
            {
                return refined +
                       (refined - whole) / SimpsonErrorScale;
            }

            MYBREP_ASSERT_MESSAGE(remainingDepth > 0,
                                  "EdgeQuery length integration exceeded the requested subdivision depth.");

            if (remainingDepth == 0)
            {
                return refined;
            }

            return integrate(first,
                             middle,
                             firstValue,
                             leftMiddleValue,
                             middleValue,
                             left,
                             tolerance * 0.5,
                             remainingDepth - 1) +
                   integrate(middle,
                             last,
                             middleValue,
                             rightMiddleValue,
                             lastValue,
                             right,
                             tolerance * 0.5,
                             remainingDepth - 1);
        }
    };

    const double middleParameter =
        (firstParameter + lastParameter) * 0.5;
    const double firstValue =
        speedAtCurveParameter(firstParameter);
    const double middleValue =
        speedAtCurveParameter(middleParameter);
    const double lastValue =
        speedAtCurveParameter(lastParameter);
    const double whole =
        simpsonValue(firstParameter,
                     lastParameter,
                     firstValue,
                     middleValue,
                     lastValue);

    Integrator integrator;
    integrator.query = this;

    return integrator.integrate(firstParameter,
                                lastParameter,
                                firstValue,
                                middleValue,
                                lastValue,
                                whole,
                                absoluteTolerance,
                                maxSubdivisionDepth);
}


double EdgeQuery::integratePolylineLength(double firstParameter,
                                          double lastParameter,
                                          double absoluteTolerance,
                                          unsigned int maxSubdivisionDepth) const
{
    struct Integrator
    {
        const EdgeQuery* query;

        MyMath::Vector3 point(double parameter) const
        {
            return query->m_localToQuery.transformPoint(
                query->geometry().pointAt(parameter));
        }

        double integrate(double first,
                         double last,
                         const MyMath::Vector3& firstPoint,
                         const MyMath::Vector3& lastPoint,
                         double tolerance,
                         unsigned int remainingDepth) const
        {
            const double quarter =
                first + (last - first) * 0.25;
            const double middle =
                (first + last) * 0.5;
            const double threeQuarter =
                first + (last - first) * 0.75;

            const MyMath::Vector3 quarterPoint = point(quarter);
            const MyMath::Vector3 middlePoint = point(middle);
            const MyMath::Vector3 threeQuarterPoint = point(threeQuarter);

            const double coarseLength =
                (middlePoint - firstPoint).length() +
                (lastPoint - middlePoint).length();

            const double refinedLength =
                (quarterPoint - firstPoint).length() +
                (middlePoint - quarterPoint).length() +
                (threeQuarterPoint - middlePoint).length() +
                (lastPoint - threeQuarterPoint).length();

            if (std::fabs(refinedLength - coarseLength) <= tolerance)
            {
                return refinedLength;
            }

            MYBREP_ASSERT_MESSAGE(remainingDepth > 0,
                                  "EdgeQuery B-Spline length integration exceeded the requested subdivision depth.");

            if (remainingDepth == 0)
            {
                return refinedLength;
            }

            return integrate(first,
                             middle,
                             firstPoint,
                             middlePoint,
                             tolerance * 0.5,
                             remainingDepth - 1) +
                   integrate(middle,
                             last,
                             middlePoint,
                             lastPoint,
                             tolerance * 0.5,
                             remainingDepth - 1);
        }
    };

    Integrator integrator;
    integrator.query = this;

    const MyMath::Vector3 firstPoint =
        m_localToQuery.transformPoint(
            geometry().pointAt(firstParameter));
    const MyMath::Vector3 lastPoint =
        m_localToQuery.transformPoint(
            geometry().pointAt(lastParameter));

    return integrator.integrate(firstParameter,
                                lastParameter,
                                firstPoint,
                                lastPoint,
                                absoluteTolerance,
                                maxSubdivisionDepth);
}

}
