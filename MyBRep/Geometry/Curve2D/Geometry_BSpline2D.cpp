#include "Geometry_BSpline2D.h"

#include <limits>

#include "MyBRep/Foundation/Diagnostic.h"

namespace
{

// 判断标量是否为有限值。
bool isFiniteValue(double value)
{
    const double infinity = (std::numeric_limits<double>::infinity)();
    return value == value && value != infinity && value != -infinity;
}

}

namespace MyBRep
{

Geometry_BSpline2D::Geometry_BSpline2D(
    std::size_t degree,
    const std::vector<MyMath::Vector2>& controlPoints,
    const std::vector<double>& knots)
    : m_degree(degree)
    , m_controlPoints(controlPoints)
    , m_knots(knots)
{
    MYBREP_ASSERT_MESSAGE(m_degree >= 1,
                          "Geometry_BSpline2D degree must be at least 1.");
    MYBREP_ASSERT_MESSAGE(m_controlPoints.size() >= m_degree + 1,
                          "Geometry_BSpline2D control point count must be at least degree + 1.");
    MYBREP_ASSERT_MESSAGE(m_knots.size() == m_controlPoints.size() + m_degree + 1,
                          "Geometry_BSpline2D knot count must equal controlPointCount + degree + 1.");

    for (std::size_t index = 0; index < m_controlPoints.size(); ++index)
    {
        MYBREP_ASSERT_MESSAGE(m_controlPoints[index].isFinite(),
                              "Geometry_BSpline2D control points must be finite.");
    }

    bool hasDifferentPoint = false;

    for (std::size_t index = 1; index < m_controlPoints.size(); ++index)
    {
        if (!m_controlPoints[index].isEqualTo(m_controlPoints[0], 0.0))
        {
            hasDifferentPoint = true;
            break;
        }
    }

    MYBREP_ASSERT_MESSAGE(hasDifferentPoint,
                          "Geometry_BSpline2D control points must not all coincide.");

    for (std::size_t index = 0; index < m_knots.size(); ++index)
    {
        MYBREP_ASSERT_MESSAGE(isFiniteValue(m_knots[index]),
                              "Geometry_BSpline2D knots must be finite.");

        if (index > 0)
        {
            MYBREP_ASSERT_MESSAGE(m_knots[index] >= m_knots[index - 1],
                                  "Geometry_BSpline2D knots must be non-decreasing.");
        }
    }

    MYBREP_ASSERT_MESSAGE(domainStart() < domainEnd(),
                          "Geometry_BSpline2D natural parameter domain must be non-degenerate.");

    std::size_t index = m_degree + 1;

    while (index < m_controlPoints.size())
    {
        const double currentKnot = m_knots[index];

        if (currentKnot <= domainStart() || currentKnot >= domainEnd())
        {
            ++index;
            continue;
        }

        std::size_t multiplicity = 1;
        ++index;

        while (index < m_controlPoints.size() && m_knots[index] == currentKnot)
        {
            ++multiplicity;
            ++index;
        }

        MYBREP_ASSERT_MESSAGE(multiplicity <= m_degree,
                              "Geometry_BSpline2D internal knot multiplicity must not exceed degree.");
    }
}

/// 样条数据

std::size_t Geometry_BSpline2D::degree() const
{
    return m_degree;
}

std::size_t Geometry_BSpline2D::controlPointCount() const
{
    return m_controlPoints.size();
}

const MyMath::Vector2& Geometry_BSpline2D::controlPoint(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(index < m_controlPoints.size(),
                          "Geometry_BSpline2D control point index is out of range.");

    return m_controlPoints[index];
}

const std::vector<MyMath::Vector2>& Geometry_BSpline2D::controlPoints() const
{
    return m_controlPoints;
}

std::size_t Geometry_BSpline2D::knotCount() const
{
    return m_knots.size();
}

double Geometry_BSpline2D::knot(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(index < m_knots.size(),
                          "Geometry_BSpline2D knot index is out of range.");

    return m_knots[index];
}

const std::vector<double>& Geometry_BSpline2D::knots() const
{
    return m_knots;
}

/// 曲线类型

CurveKind Geometry_BSpline2D::kind() const
{
    return CurveKind::BSpline;
}

/// 定义域

bool Geometry_BSpline2D::isDomainBounded() const
{
    return true;
}

double Geometry_BSpline2D::domainStart() const
{
    return m_knots[m_degree];
}

double Geometry_BSpline2D::domainEnd() const
{
    return m_knots[m_controlPoints.size()];
}

/// 周期性

double Geometry_BSpline2D::period() const
{
    return 0.0;
}

/// 参数查询

MyMath::Vector2 Geometry_BSpline2D::pointAt(double parameter) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(parameter),
                          "Geometry_BSpline2D parameter is outside the natural parameter domain.");

    return evaluate(m_controlPoints, m_knots, m_degree, parameter);
}

MyMath::Vector2 Geometry_BSpline2D::firstDerivativeAt(double parameter) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(parameter),
                          "Geometry_BSpline2D derivative parameter is outside the natural parameter domain.");
    MYBREP_ASSERT_MESSAGE(isDerivativeDefined(parameter, 1),
                          "Geometry_BSpline2D first derivative is not uniquely defined at this internal knot.");

    std::vector<MyMath::Vector2> derivativeControlPoints;
    std::vector<double> derivativeKnots;

    buildDerivativeCurve(m_controlPoints,
                         m_knots,
                         m_degree,
                         derivativeControlPoints,
                         derivativeKnots);

    return evaluate(derivativeControlPoints,
                    derivativeKnots,
                    m_degree - 1,
                    parameter);
}

MyMath::Vector2 Geometry_BSpline2D::secondDerivativeAt(double parameter) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(parameter),
                          "Geometry_BSpline2D derivative parameter is outside the natural parameter domain.");
    MYBREP_ASSERT_MESSAGE(isDerivativeDefined(parameter, 2),
                          "Geometry_BSpline2D second derivative is not uniquely defined at this internal knot.");

    if (m_degree == 1)
    {
        return MyMath::Vector2::zero();
    }

    std::vector<MyMath::Vector2> firstDerivativeControlPoints;
    std::vector<double> firstDerivativeKnots;

    buildDerivativeCurve(m_controlPoints,
                         m_knots,
                         m_degree,
                         firstDerivativeControlPoints,
                         firstDerivativeKnots);

    std::vector<MyMath::Vector2> secondDerivativeControlPoints;
    std::vector<double> secondDerivativeKnots;

    buildDerivativeCurve(firstDerivativeControlPoints,
                         firstDerivativeKnots,
                         m_degree - 1,
                         secondDerivativeControlPoints,
                         secondDerivativeKnots);

    return evaluate(secondDerivativeControlPoints,
                    secondDerivativeKnots,
                    m_degree - 2,
                    parameter);
}

/// 内部辅助

std::size_t Geometry_BSpline2D::findSpan(const std::vector<double>& knots,
                                         std::size_t degree,
                                         std::size_t controlPointCount,
                                         double parameter)
{
    MYBREP_ASSERT_MESSAGE(controlPointCount >= degree + 1,
                          "Geometry_BSpline2D span query requires a valid control point count.");

    const std::size_t lastControlPoint = controlPointCount - 1;
    const double domainRight = knots[controlPointCount];

    if (parameter == domainRight)
    {
        return lastControlPoint;
    }

    std::size_t low = degree;
    std::size_t high = controlPointCount;
    std::size_t middle = (low + high) / 2;

    while (parameter < knots[middle] || parameter >= knots[middle + 1])
    {
        if (parameter < knots[middle])
        {
            high = middle;
        }
        else
        {
            low = middle;
        }

        middle = (low + high) / 2;
    }

    return middle;
}

MyMath::Vector2 Geometry_BSpline2D::evaluate(
    const std::vector<MyMath::Vector2>& controlPoints,
    const std::vector<double>& knots,
    std::size_t degree,
    double parameter)
{
    MYBREP_ASSERT_MESSAGE(!controlPoints.empty(),
                          "Geometry_BSpline2D evaluation requires control points.");
    MYBREP_ASSERT_MESSAGE(knots.size() == controlPoints.size() + degree + 1,
                          "Geometry_BSpline2D evaluation data is inconsistent.");

    const std::size_t span =
        findSpan(knots, degree, controlPoints.size(), parameter);

    std::vector<MyMath::Vector2> workPoints(degree + 1);

    for (std::size_t index = 0; index <= degree; ++index)
    {
        workPoints[index] = controlPoints[span - degree + index];
    }

    for (std::size_t level = 1; level <= degree; ++level)
    {
        for (std::size_t index = degree; index >= level; --index)
        {
            const std::size_t knotIndex = span - degree + index;
            const double denominator =
                knots[knotIndex + degree - level + 1] -
                knots[knotIndex];

            MYBREP_ASSERT_MESSAGE(denominator > 0.0,
                                  "Geometry_BSpline2D de Boor evaluation encountered a zero knot interval.");

            const double alpha =
                (parameter - knots[knotIndex]) / denominator;

            workPoints[index] =
                workPoints[index - 1] * (1.0 - alpha) +
                workPoints[index] * alpha;

            if (index == level)
            {
                break;
            }
        }
    }

    return workPoints[degree];
}

void Geometry_BSpline2D::buildDerivativeCurve(
    const std::vector<MyMath::Vector2>& controlPoints,
    const std::vector<double>& knots,
    std::size_t degree,
    std::vector<MyMath::Vector2>& derivativeControlPoints,
    std::vector<double>& derivativeKnots)
{
    MYBREP_ASSERT_MESSAGE(degree >= 1,
                          "Geometry_BSpline2D derivative construction requires degree >= 1.");
    MYBREP_ASSERT_MESSAGE(knots.size() == controlPoints.size() + degree + 1,
                          "Geometry_BSpline2D derivative construction data is inconsistent.");

    derivativeControlPoints.clear();
    derivativeControlPoints.reserve(controlPoints.size() - 1);

    const double degreeScale = static_cast<double>(degree);

    for (std::size_t index = 0; index + 1 < controlPoints.size(); ++index)
    {
        const double denominator =
            knots[index + degree + 1] - knots[index + 1];

        if (denominator == 0.0)
        {
            // 重复节点形成零长度节点区间时，该导数控制项按B样条导数定义取零。
            derivativeControlPoints.push_back(MyMath::Vector2::zero());
            continue;
        }

        derivativeControlPoints.push_back(
            (controlPoints[index + 1] - controlPoints[index]) *
            (degreeScale / denominator));
    }

    derivativeKnots.assign(knots.begin() + 1, knots.end() - 1);
}

std::size_t Geometry_BSpline2D::internalKnotMultiplicity(double parameter) const
{
    if (parameter <= domainStart() || parameter >= domainEnd())
    {
        return 0;
    }

    std::size_t multiplicity = 0;

    for (std::size_t index = m_degree + 1; index < m_controlPoints.size(); ++index)
    {
        if (m_knots[index] == parameter)
        {
            ++multiplicity;
        }
    }

    return multiplicity;
}

bool Geometry_BSpline2D::isDerivativeDefined(double parameter,
                                             std::size_t derivativeOrder) const
{
    MYBREP_ASSERT_MESSAGE(derivativeOrder >= 1 && derivativeOrder <= 2,
                          "Geometry_BSpline2D derivative order must be 1 or 2.");

    const std::size_t multiplicity = internalKnotMultiplicity(parameter);

    if (multiplicity == 0)
    {
        return true;
    }

    return m_degree >= multiplicity + derivativeOrder;
}

}
