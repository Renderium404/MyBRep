#include "Geometry_BSplineSurface.h"

#include <limits>

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

Geometry_BSplineSurface::Geometry_BSplineSurface(
    std::size_t uDegree,
    std::size_t vDegree,
    std::size_t uControlPointCount,
    std::size_t vControlPointCount,
    const std::vector<MyMath::Vector3>& controlPoints,
    const std::vector<double>& uKnots,
    const std::vector<double>& vKnots)
    : m_uDegree(uDegree)
    , m_vDegree(vDegree)
    , m_uControlPointCount(uControlPointCount)
    , m_vControlPointCount(vControlPointCount)
    , m_controlPoints(controlPoints)
    , m_uKnots(uKnots)
    , m_vKnots(vKnots)
{
    MYBREP_ASSERT_MESSAGE(m_uDegree >= 1,
                          "Geometry_BSplineSurface U degree must be at least 1.");
    MYBREP_ASSERT_MESSAGE(m_vDegree >= 1,
                          "Geometry_BSplineSurface V degree must be at least 1.");
    MYBREP_ASSERT_MESSAGE(m_uControlPointCount >= m_uDegree + 1,
                          "Geometry_BSplineSurface U control point count must be at least U degree + 1.");
    MYBREP_ASSERT_MESSAGE(m_vControlPointCount >= m_vDegree + 1,
                          "Geometry_BSplineSurface V control point count must be at least V degree + 1.");
    MYBREP_ASSERT_MESSAGE(m_controlPoints.size() == m_uControlPointCount * m_vControlPointCount,
                          "Geometry_BSplineSurface control point count does not match the control net dimensions.");
    MYBREP_ASSERT_MESSAGE(m_uKnots.size() == m_uControlPointCount + m_uDegree + 1,
                          "Geometry_BSplineSurface U knot count is inconsistent.");
    MYBREP_ASSERT_MESSAGE(m_vKnots.size() == m_vControlPointCount + m_vDegree + 1,
                          "Geometry_BSplineSurface V knot count is inconsistent.");

    for (std::size_t index = 0; index < m_controlPoints.size(); ++index)
    {
        MYBREP_ASSERT_MESSAGE(m_controlPoints[index].isFinite(),
                              "Geometry_BSplineSurface control points must be finite.");
    }

    for (std::size_t index = 0; index < m_uKnots.size(); ++index)
    {
        MYBREP_ASSERT_MESSAGE(isFiniteValue(m_uKnots[index]),
                              "Geometry_BSplineSurface U knots must be finite.");

        if (index > 0)
        {
            MYBREP_ASSERT_MESSAGE(m_uKnots[index] >= m_uKnots[index - 1],
                                  "Geometry_BSplineSurface U knots must be non-decreasing.");
        }
    }

    for (std::size_t index = 0; index < m_vKnots.size(); ++index)
    {
        MYBREP_ASSERT_MESSAGE(isFiniteValue(m_vKnots[index]),
                              "Geometry_BSplineSurface V knots must be finite.");

        if (index > 0)
        {
            MYBREP_ASSERT_MESSAGE(m_vKnots[index] >= m_vKnots[index - 1],
                                  "Geometry_BSplineSurface V knots must be non-decreasing.");
        }
    }

    MYBREP_ASSERT_MESSAGE(uDomainStart() < uDomainEnd(),
                          "Geometry_BSplineSurface U natural parameter domain must be non-degenerate.");
    MYBREP_ASSERT_MESSAGE(vDomainStart() < vDomainEnd(),
                          "Geometry_BSplineSurface V natural parameter domain must be non-degenerate.");

    std::size_t uIndex = m_uDegree + 1;

    while (uIndex < m_uControlPointCount)
    {
        const double currentKnot = m_uKnots[uIndex];

        if (currentKnot <= uDomainStart() || currentKnot >= uDomainEnd())
        {
            ++uIndex;
            continue;
        }

        std::size_t multiplicity = 1;
        ++uIndex;

        while (uIndex < m_uControlPointCount && m_uKnots[uIndex] == currentKnot)
        {
            ++multiplicity;
            ++uIndex;
        }

        MYBREP_ASSERT_MESSAGE(multiplicity <= m_uDegree,
                              "Geometry_BSplineSurface internal U knot multiplicity must not exceed U degree.");
    }

    std::size_t vIndex = m_vDegree + 1;

    while (vIndex < m_vControlPointCount)
    {
        const double currentKnot = m_vKnots[vIndex];

        if (currentKnot <= vDomainStart() || currentKnot >= vDomainEnd())
        {
            ++vIndex;
            continue;
        }

        std::size_t multiplicity = 1;
        ++vIndex;

        while (vIndex < m_vControlPointCount && m_vKnots[vIndex] == currentKnot)
        {
            ++multiplicity;
            ++vIndex;
        }

        MYBREP_ASSERT_MESSAGE(multiplicity <= m_vDegree,
                              "Geometry_BSplineSurface internal V knot multiplicity must not exceed V degree.");
    }
}

/// 样条数据

std::size_t Geometry_BSplineSurface::uDegree() const
{
    return m_uDegree;
}

std::size_t Geometry_BSplineSurface::vDegree() const
{
    return m_vDegree;
}

std::size_t Geometry_BSplineSurface::uControlPointCount() const
{
    return m_uControlPointCount;
}

std::size_t Geometry_BSplineSurface::vControlPointCount() const
{
    return m_vControlPointCount;
}

const MyMath::Vector3& Geometry_BSplineSurface::controlPoint(std::size_t uIndex, std::size_t vIndex) const
{
    MYBREP_ASSERT_MESSAGE(uIndex < m_uControlPointCount,
                          "Geometry_BSplineSurface U control point index is out of range.");
    MYBREP_ASSERT_MESSAGE(vIndex < m_vControlPointCount,
                          "Geometry_BSplineSurface V control point index is out of range.");

    return m_controlPoints[controlPointIndex(uIndex, vIndex, m_uControlPointCount)];
}

const std::vector<MyMath::Vector3>& Geometry_BSplineSurface::controlPoints() const
{
    return m_controlPoints;
}

std::size_t Geometry_BSplineSurface::uKnotCount() const
{
    return m_uKnots.size();
}

std::size_t Geometry_BSplineSurface::vKnotCount() const
{
    return m_vKnots.size();
}

double Geometry_BSplineSurface::uKnot(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(index < m_uKnots.size(),
                          "Geometry_BSplineSurface U knot index is out of range.");

    return m_uKnots[index];
}

double Geometry_BSplineSurface::vKnot(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(index < m_vKnots.size(),
                          "Geometry_BSplineSurface V knot index is out of range.");

    return m_vKnots[index];
}

const std::vector<double>& Geometry_BSplineSurface::uKnots() const
{
    return m_uKnots;
}

const std::vector<double>& Geometry_BSplineSurface::vKnots() const
{
    return m_vKnots;
}

/// 曲面类型

SurfaceKind Geometry_BSplineSurface::kind() const
{
    return SurfaceKind::BSpline;
}

/// U定义域

bool Geometry_BSplineSurface::isUDomainBounded() const
{
    return true;
}

double Geometry_BSplineSurface::uDomainStart() const
{
    return m_uKnots[m_uDegree];
}

double Geometry_BSplineSurface::uDomainEnd() const
{
    return m_uKnots[m_uControlPointCount];
}

/// V定义域

bool Geometry_BSplineSurface::isVDomainBounded() const
{
    return true;
}

double Geometry_BSplineSurface::vDomainStart() const
{
    return m_vKnots[m_vDegree];
}

double Geometry_BSplineSurface::vDomainEnd() const
{
    return m_vKnots[m_vControlPointCount];
}

/// 周期性

double Geometry_BSplineSurface::uPeriod() const
{
    return 0.0;
}

double Geometry_BSplineSurface::vPeriod() const
{
    return 0.0;
}

/// 参数查询

MyMath::Vector3 Geometry_BSplineSurface::pointAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                          "Geometry_BSplineSurface parameters are outside the natural parameter domain.");

    return evaluateSurface(m_controlPoints,
                           m_uControlPointCount,
                           m_vControlPointCount,
                           m_uKnots,
                           m_vKnots,
                           m_uDegree,
                           m_vDegree,
                           u,
                           v);
}

/// 一阶偏导

MyMath::Vector3 Geometry_BSplineSurface::firstDerivativeUAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                          "Geometry_BSplineSurface derivative parameters are outside the natural parameter domain.");
    MYBREP_ASSERT_MESSAGE(isUDerivativeDefined(u, 1),
                          "Geometry_BSplineSurface U first derivative is not uniquely defined at this internal knot.");

    std::vector<MyMath::Vector3> derivativeControlPoints;
    std::vector<double> derivativeUKnots;

    buildUDerivativeSurface(m_controlPoints,
                            m_uControlPointCount,
                            m_vControlPointCount,
                            m_uKnots,
                            m_uDegree,
                            derivativeControlPoints,
                            derivativeUKnots);

    return evaluateSurface(derivativeControlPoints,
                           m_uControlPointCount - 1,
                           m_vControlPointCount,
                           derivativeUKnots,
                           m_vKnots,
                           m_uDegree - 1,
                           m_vDegree,
                           u,
                           v);
}

MyMath::Vector3 Geometry_BSplineSurface::firstDerivativeVAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                          "Geometry_BSplineSurface derivative parameters are outside the natural parameter domain.");
    MYBREP_ASSERT_MESSAGE(isVDerivativeDefined(v, 1),
                          "Geometry_BSplineSurface V first derivative is not uniquely defined at this internal knot.");

    std::vector<MyMath::Vector3> derivativeControlPoints;
    std::vector<double> derivativeVKnots;

    buildVDerivativeSurface(m_controlPoints,
                            m_uControlPointCount,
                            m_vControlPointCount,
                            m_vKnots,
                            m_vDegree,
                            derivativeControlPoints,
                            derivativeVKnots);

    return evaluateSurface(derivativeControlPoints,
                           m_uControlPointCount,
                           m_vControlPointCount - 1,
                           m_uKnots,
                           derivativeVKnots,
                           m_uDegree,
                           m_vDegree - 1,
                           u,
                           v);
}

/// 二阶偏导

MyMath::Vector3 Geometry_BSplineSurface::secondDerivativeUUAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                          "Geometry_BSplineSurface derivative parameters are outside the natural parameter domain.");
    MYBREP_ASSERT_MESSAGE(isUDerivativeDefined(u, 2),
                          "Geometry_BSplineSurface U second derivative is not uniquely defined at this internal knot.");

    if (m_uDegree == 1)
    {
        return MyMath::Vector3::zero();
    }

    std::vector<MyMath::Vector3> firstDerivativeControlPoints;
    std::vector<double> firstDerivativeUKnots;

    buildUDerivativeSurface(m_controlPoints,
                            m_uControlPointCount,
                            m_vControlPointCount,
                            m_uKnots,
                            m_uDegree,
                            firstDerivativeControlPoints,
                            firstDerivativeUKnots);

    std::vector<MyMath::Vector3> secondDerivativeControlPoints;
    std::vector<double> secondDerivativeUKnots;

    buildUDerivativeSurface(firstDerivativeControlPoints,
                            m_uControlPointCount - 1,
                            m_vControlPointCount,
                            firstDerivativeUKnots,
                            m_uDegree - 1,
                            secondDerivativeControlPoints,
                            secondDerivativeUKnots);

    return evaluateSurface(secondDerivativeControlPoints,
                           m_uControlPointCount - 2,
                           m_vControlPointCount,
                           secondDerivativeUKnots,
                           m_vKnots,
                           m_uDegree - 2,
                           m_vDegree,
                           u,
                           v);
}

MyMath::Vector3 Geometry_BSplineSurface::secondDerivativeUVAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                          "Geometry_BSplineSurface derivative parameters are outside the natural parameter domain.");
    MYBREP_ASSERT_MESSAGE(isUDerivativeDefined(u, 1),
                          "Geometry_BSplineSurface UV derivative requires a unique U first derivative.");
    MYBREP_ASSERT_MESSAGE(isVDerivativeDefined(v, 1),
                          "Geometry_BSplineSurface UV derivative requires a unique V first derivative.");

    std::vector<MyMath::Vector3> uDerivativeControlPoints;
    std::vector<double> uDerivativeKnots;

    buildUDerivativeSurface(m_controlPoints,
                            m_uControlPointCount,
                            m_vControlPointCount,
                            m_uKnots,
                            m_uDegree,
                            uDerivativeControlPoints,
                            uDerivativeKnots);

    std::vector<MyMath::Vector3> uvDerivativeControlPoints;
    std::vector<double> uvDerivativeVKnots;

    buildVDerivativeSurface(uDerivativeControlPoints,
                            m_uControlPointCount - 1,
                            m_vControlPointCount,
                            m_vKnots,
                            m_vDegree,
                            uvDerivativeControlPoints,
                            uvDerivativeVKnots);

    return evaluateSurface(uvDerivativeControlPoints,
                           m_uControlPointCount - 1,
                           m_vControlPointCount - 1,
                           uDerivativeKnots,
                           uvDerivativeVKnots,
                           m_uDegree - 1,
                           m_vDegree - 1,
                           u,
                           v);
}

MyMath::Vector3 Geometry_BSplineSurface::secondDerivativeVVAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                          "Geometry_BSplineSurface derivative parameters are outside the natural parameter domain.");
    MYBREP_ASSERT_MESSAGE(isVDerivativeDefined(v, 2),
                          "Geometry_BSplineSurface V second derivative is not uniquely defined at this internal knot.");

    if (m_vDegree == 1)
    {
        return MyMath::Vector3::zero();
    }

    std::vector<MyMath::Vector3> firstDerivativeControlPoints;
    std::vector<double> firstDerivativeVKnots;

    buildVDerivativeSurface(m_controlPoints,
                            m_uControlPointCount,
                            m_vControlPointCount,
                            m_vKnots,
                            m_vDegree,
                            firstDerivativeControlPoints,
                            firstDerivativeVKnots);

    std::vector<MyMath::Vector3> secondDerivativeControlPoints;
    std::vector<double> secondDerivativeVKnots;

    buildVDerivativeSurface(firstDerivativeControlPoints,
                            m_uControlPointCount,
                            m_vControlPointCount - 1,
                            firstDerivativeVKnots,
                            m_vDegree - 1,
                            secondDerivativeControlPoints,
                            secondDerivativeVKnots);

    return evaluateSurface(secondDerivativeControlPoints,
                           m_uControlPointCount,
                           m_vControlPointCount - 2,
                           m_uKnots,
                           secondDerivativeVKnots,
                           m_uDegree,
                           m_vDegree - 2,
                           u,
                           v);
}

/// 内部辅助

std::size_t Geometry_BSplineSurface::controlPointIndex(std::size_t uIndex,
                                                       std::size_t vIndex,
                                                       std::size_t uControlPointCount)
{
    return vIndex * uControlPointCount + uIndex;
}

std::size_t Geometry_BSplineSurface::findSpan(const std::vector<double>& knots,
                                              std::size_t degree,
                                              std::size_t controlPointCount,
                                              double parameter)
{
    MYBREP_ASSERT_MESSAGE(controlPointCount >= degree + 1,
                          "Geometry_BSplineSurface span query requires a valid control point count.");

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

MyMath::Vector3 Geometry_BSplineSurface::evaluateCurve(
    const std::vector<MyMath::Vector3>& controlPoints,
    const std::vector<double>& knots,
    std::size_t degree,
    double parameter)
{
    MYBREP_ASSERT_MESSAGE(!controlPoints.empty(),
                          "Geometry_BSplineSurface curve evaluation requires control points.");
    MYBREP_ASSERT_MESSAGE(knots.size() == controlPoints.size() + degree + 1,
                          "Geometry_BSplineSurface curve evaluation data is inconsistent.");

    const std::size_t span = findSpan(knots, degree, controlPoints.size(), parameter);

    std::vector<MyMath::Vector3> workPoints(degree + 1);

    for (std::size_t index = 0; index <= degree; ++index)
    {
        workPoints[index] = controlPoints[span - degree + index];
    }

    for (std::size_t level = 1; level <= degree; ++level)
    {
        for (std::size_t index = degree; index >= level; --index)
        {
            const std::size_t knotIndex = span - degree + index;
            const double denominator = knots[knotIndex + degree - level + 1] - knots[knotIndex];

            MYBREP_ASSERT_MESSAGE(denominator > 0.0,
                                  "Geometry_BSplineSurface de Boor evaluation encountered a zero knot interval.");

            const double alpha = (parameter - knots[knotIndex]) / denominator;

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

MyMath::Vector3 Geometry_BSplineSurface::evaluateSurface(
    const std::vector<MyMath::Vector3>& controlPoints,
    std::size_t uControlPointCount,
    std::size_t vControlPointCount,
    const std::vector<double>& uKnots,
    const std::vector<double>& vKnots,
    std::size_t uDegree,
    std::size_t vDegree,
    double u,
    double v)
{
    MYBREP_ASSERT_MESSAGE(controlPoints.size() == uControlPointCount * vControlPointCount,
                          "Geometry_BSplineSurface evaluation control net dimensions are inconsistent.");

    std::vector<MyMath::Vector3> vDirectionPoints;
    vDirectionPoints.reserve(vControlPointCount);

    for (std::size_t vIndex = 0; vIndex < vControlPointCount; ++vIndex)
    {
        std::vector<MyMath::Vector3> uDirectionPoints;
        uDirectionPoints.reserve(uControlPointCount);

        for (std::size_t uIndex = 0; uIndex < uControlPointCount; ++uIndex)
        {
            uDirectionPoints.push_back(
                controlPoints[controlPointIndex(uIndex, vIndex, uControlPointCount)]);
        }

        vDirectionPoints.push_back(
            evaluateCurve(uDirectionPoints, uKnots, uDegree, u));
    }

    return evaluateCurve(vDirectionPoints, vKnots, vDegree, v);
}

void Geometry_BSplineSurface::buildUDerivativeSurface(
    const std::vector<MyMath::Vector3>& controlPoints,
    std::size_t uControlPointCount,
    std::size_t vControlPointCount,
    const std::vector<double>& uKnots,
    std::size_t uDegree,
    std::vector<MyMath::Vector3>& derivativeControlPoints,
    std::vector<double>& derivativeUKnots)
{
    MYBREP_ASSERT_MESSAGE(uDegree >= 1,
                          "Geometry_BSplineSurface U derivative construction requires U degree >= 1.");
    MYBREP_ASSERT_MESSAGE(controlPoints.size() == uControlPointCount * vControlPointCount,
                          "Geometry_BSplineSurface U derivative control net dimensions are inconsistent.");
    MYBREP_ASSERT_MESSAGE(uKnots.size() == uControlPointCount + uDegree + 1,
                          "Geometry_BSplineSurface U derivative knot vector is inconsistent.");

    const double degreeScale = static_cast<double>(uDegree);

    derivativeControlPoints.clear();
    derivativeControlPoints.reserve((uControlPointCount - 1) * vControlPointCount);

    for (std::size_t vIndex = 0; vIndex < vControlPointCount; ++vIndex)
    {
        for (std::size_t uIndex = 0; uIndex + 1 < uControlPointCount; ++uIndex)
        {
            const double denominator = uKnots[uIndex + uDegree + 1] - uKnots[uIndex + 1];

            if (denominator == 0.0)
            {
                // 重复节点形成零长度节点区间时，该导数控制项按B样条导数定义取零。
                derivativeControlPoints.push_back(MyMath::Vector3::zero());
                continue;
            }

            const MyMath::Vector3& current =
                controlPoints[controlPointIndex(uIndex, vIndex, uControlPointCount)];
            const MyMath::Vector3& next =
                controlPoints[controlPointIndex(uIndex + 1, vIndex, uControlPointCount)];

            derivativeControlPoints.push_back(
                (next - current) * (degreeScale / denominator));
        }
    }

    derivativeUKnots.assign(uKnots.begin() + 1, uKnots.end() - 1);
}

void Geometry_BSplineSurface::buildVDerivativeSurface(
    const std::vector<MyMath::Vector3>& controlPoints,
    std::size_t uControlPointCount,
    std::size_t vControlPointCount,
    const std::vector<double>& vKnots,
    std::size_t vDegree,
    std::vector<MyMath::Vector3>& derivativeControlPoints,
    std::vector<double>& derivativeVKnots)
{
    MYBREP_ASSERT_MESSAGE(vDegree >= 1,
                          "Geometry_BSplineSurface V derivative construction requires V degree >= 1.");
    MYBREP_ASSERT_MESSAGE(controlPoints.size() == uControlPointCount * vControlPointCount,
                          "Geometry_BSplineSurface V derivative control net dimensions are inconsistent.");
    MYBREP_ASSERT_MESSAGE(vKnots.size() == vControlPointCount + vDegree + 1,
                          "Geometry_BSplineSurface V derivative knot vector is inconsistent.");

    const double degreeScale = static_cast<double>(vDegree);

    derivativeControlPoints.clear();
    derivativeControlPoints.reserve(uControlPointCount * (vControlPointCount - 1));

    for (std::size_t vIndex = 0; vIndex + 1 < vControlPointCount; ++vIndex)
    {
        const double denominator = vKnots[vIndex + vDegree + 1] - vKnots[vIndex + 1];

        for (std::size_t uIndex = 0; uIndex < uControlPointCount; ++uIndex)
        {
            if (denominator == 0.0)
            {
                // 重复节点形成零长度节点区间时，该导数控制项按B样条导数定义取零。
                derivativeControlPoints.push_back(MyMath::Vector3::zero());
                continue;
            }

            const MyMath::Vector3& current =
                controlPoints[controlPointIndex(uIndex, vIndex, uControlPointCount)];
            const MyMath::Vector3& next =
                controlPoints[controlPointIndex(uIndex, vIndex + 1, uControlPointCount)];

            derivativeControlPoints.push_back(
                (next - current) * (degreeScale / denominator));
        }
    }

    derivativeVKnots.assign(vKnots.begin() + 1, vKnots.end() - 1);
}

std::size_t Geometry_BSplineSurface::internalUKnotMultiplicity(double parameter) const
{
    if (parameter <= uDomainStart() || parameter >= uDomainEnd())
    {
        return 0;
    }

    std::size_t multiplicity = 0;

    for (std::size_t index = m_uDegree + 1; index < m_uControlPointCount; ++index)
    {
        if (m_uKnots[index] == parameter)
        {
            ++multiplicity;
        }
    }

    return multiplicity;
}

std::size_t Geometry_BSplineSurface::internalVKnotMultiplicity(double parameter) const
{
    if (parameter <= vDomainStart() || parameter >= vDomainEnd())
    {
        return 0;
    }

    std::size_t multiplicity = 0;

    for (std::size_t index = m_vDegree + 1; index < m_vControlPointCount; ++index)
    {
        if (m_vKnots[index] == parameter)
        {
            ++multiplicity;
        }
    }

    return multiplicity;
}

bool Geometry_BSplineSurface::isUDerivativeDefined(double parameter, std::size_t derivativeOrder) const
{
    MYBREP_ASSERT_MESSAGE(derivativeOrder >= 1 && derivativeOrder <= 2,
                          "Geometry_BSplineSurface U derivative order must be 1 or 2.");

    const std::size_t multiplicity = internalUKnotMultiplicity(parameter);

    if (multiplicity == 0)
    {
        return true;
    }

    return m_uDegree >= multiplicity + derivativeOrder;
}

bool Geometry_BSplineSurface::isVDerivativeDefined(double parameter, std::size_t derivativeOrder) const
{
    MYBREP_ASSERT_MESSAGE(derivativeOrder >= 1 && derivativeOrder <= 2,
                          "Geometry_BSplineSurface V derivative order must be 1 or 2.");

    const std::size_t multiplicity = internalVKnotMultiplicity(parameter);

    if (multiplicity == 0)
    {
        return true;
    }

    return m_vDegree >= multiplicity + derivativeOrder;
}

}