#include "Geometry_BezierSurface.h"

namespace MyBRep
{

Geometry_BezierSurface::Geometry_BezierSurface(
    std::size_t uControlPointCount,
    std::size_t vControlPointCount,
    const std::vector<MyMath::Vector3>& controlPoints)
    : m_uControlPointCount(uControlPointCount)
    , m_vControlPointCount(vControlPointCount)
    , m_controlPoints(controlPoints)
{
    MYBREP_ASSERT_MESSAGE(m_uControlPointCount >= 2,
                          "Geometry_BezierSurface requires at least two U control points.");
    MYBREP_ASSERT_MESSAGE(m_vControlPointCount >= 2,
                          "Geometry_BezierSurface requires at least two V control points.");
    MYBREP_ASSERT_MESSAGE(m_controlPoints.size() == m_uControlPointCount * m_vControlPointCount,
                          "Geometry_BezierSurface control point count does not match the control net dimensions.");

    for (std::size_t index = 0; index < m_controlPoints.size(); ++index)
    {
        MYBREP_ASSERT_MESSAGE(m_controlPoints[index].isFinite(),
                              "Geometry_BezierSurface control points must be finite.");
    }
}

/// 控制数据

std::size_t Geometry_BezierSurface::uControlPointCount() const
{
    return m_uControlPointCount;
}

std::size_t Geometry_BezierSurface::vControlPointCount() const
{
    return m_vControlPointCount;
}

std::size_t Geometry_BezierSurface::uDegree() const
{
    return m_uControlPointCount - 1;
}

std::size_t Geometry_BezierSurface::vDegree() const
{
    return m_vControlPointCount - 1;
}

const MyMath::Vector3& Geometry_BezierSurface::controlPoint(std::size_t uIndex, std::size_t vIndex) const
{
    MYBREP_ASSERT_MESSAGE(uIndex < m_uControlPointCount,
                          "Geometry_BezierSurface U control point index is out of range.");
    MYBREP_ASSERT_MESSAGE(vIndex < m_vControlPointCount,
                          "Geometry_BezierSurface V control point index is out of range.");

    return m_controlPoints[controlPointIndex(uIndex, vIndex, m_uControlPointCount)];
}

const std::vector<MyMath::Vector3>& Geometry_BezierSurface::controlPoints() const
{
    return m_controlPoints;
}

/// 曲面类型

SurfaceKind Geometry_BezierSurface::kind() const
{
    return SurfaceKind::Bezier;
}

/// U定义域

bool Geometry_BezierSurface::isUDomainBounded() const
{
    return true;
}

double Geometry_BezierSurface::uDomainStart() const
{
    return 0.0;
}

double Geometry_BezierSurface::uDomainEnd() const
{
    return 1.0;
}

/// V定义域

bool Geometry_BezierSurface::isVDomainBounded() const
{
    return true;
}

double Geometry_BezierSurface::vDomainStart() const
{
    return 0.0;
}

double Geometry_BezierSurface::vDomainEnd() const
{
    return 1.0;
}

/// 周期性

double Geometry_BezierSurface::uPeriod() const
{
    return 0.0;
}

double Geometry_BezierSurface::vPeriod() const
{
    return 0.0;
}

/// 参数查询

MyMath::Vector3 Geometry_BezierSurface::pointAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                          "Geometry_BezierSurface parameters are outside the natural parameter domain.");

    return evaluateSurface(m_controlPoints,
                           m_uControlPointCount,
                           m_vControlPointCount,
                           u,
                           v);
}

/// 一阶偏导

MyMath::Vector3 Geometry_BezierSurface::firstDerivativeUAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                          "Geometry_BezierSurface derivative parameters are outside the natural parameter domain.");

    std::vector<MyMath::Vector3> derivativeControlPoints;

    buildUDerivativeControlNet(m_controlPoints,
                               m_uControlPointCount,
                               m_vControlPointCount,
                               derivativeControlPoints);

    return evaluateSurface(derivativeControlPoints,
                           m_uControlPointCount - 1,
                           m_vControlPointCount,
                           u,
                           v);
}

MyMath::Vector3 Geometry_BezierSurface::firstDerivativeVAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                          "Geometry_BezierSurface derivative parameters are outside the natural parameter domain.");

    std::vector<MyMath::Vector3> derivativeControlPoints;

    buildVDerivativeControlNet(m_controlPoints,
                               m_uControlPointCount,
                               m_vControlPointCount,
                               derivativeControlPoints);

    return evaluateSurface(derivativeControlPoints,
                           m_uControlPointCount,
                           m_vControlPointCount - 1,
                           u,
                           v);
}

/// 二阶偏导

MyMath::Vector3 Geometry_BezierSurface::secondDerivativeUUAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                          "Geometry_BezierSurface derivative parameters are outside the natural parameter domain.");

    if (uDegree() == 1)
    {
        return MyMath::Vector3::zero();
    }

    std::vector<MyMath::Vector3> firstDerivativeControlPoints;
    buildUDerivativeControlNet(m_controlPoints,
                               m_uControlPointCount,
                               m_vControlPointCount,
                               firstDerivativeControlPoints);

    std::vector<MyMath::Vector3> secondDerivativeControlPoints;
    buildUDerivativeControlNet(firstDerivativeControlPoints,
                               m_uControlPointCount - 1,
                               m_vControlPointCount,
                               secondDerivativeControlPoints);

    return evaluateSurface(secondDerivativeControlPoints,
                           m_uControlPointCount - 2,
                           m_vControlPointCount,
                           u,
                           v);
}

MyMath::Vector3 Geometry_BezierSurface::secondDerivativeUVAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                          "Geometry_BezierSurface derivative parameters are outside the natural parameter domain.");

    std::vector<MyMath::Vector3> uDerivativeControlPoints;
    buildUDerivativeControlNet(m_controlPoints,
                               m_uControlPointCount,
                               m_vControlPointCount,
                               uDerivativeControlPoints);

    std::vector<MyMath::Vector3> uvDerivativeControlPoints;
    buildVDerivativeControlNet(uDerivativeControlPoints,
                               m_uControlPointCount - 1,
                               m_vControlPointCount,
                               uvDerivativeControlPoints);

    return evaluateSurface(uvDerivativeControlPoints,
                           m_uControlPointCount - 1,
                           m_vControlPointCount - 1,
                           u,
                           v);
}

MyMath::Vector3 Geometry_BezierSurface::secondDerivativeVVAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                          "Geometry_BezierSurface derivative parameters are outside the natural parameter domain.");

    if (vDegree() == 1)
    {
        return MyMath::Vector3::zero();
    }

    std::vector<MyMath::Vector3> firstDerivativeControlPoints;
    buildVDerivativeControlNet(m_controlPoints,
                               m_uControlPointCount,
                               m_vControlPointCount,
                               firstDerivativeControlPoints);

    std::vector<MyMath::Vector3> secondDerivativeControlPoints;
    buildVDerivativeControlNet(firstDerivativeControlPoints,
                               m_uControlPointCount,
                               m_vControlPointCount - 1,
                               secondDerivativeControlPoints);

    return evaluateSurface(secondDerivativeControlPoints,
                           m_uControlPointCount,
                           m_vControlPointCount - 2,
                           u,
                           v);
}

/// 内部辅助

std::size_t Geometry_BezierSurface::controlPointIndex(std::size_t uIndex,
                                                      std::size_t vIndex,
                                                      std::size_t uControlPointCount)
{
    return vIndex * uControlPointCount + uIndex;
}

MyMath::Vector3 Geometry_BezierSurface::evaluateCurve(
    const std::vector<MyMath::Vector3>& points,
    double parameter)
{
    MYBREP_ASSERT_MESSAGE(!points.empty(),
                          "Geometry_BezierSurface curve evaluation requires at least one control point.");
    MYBREP_ASSERT_MESSAGE(parameter >= 0.0 && parameter <= 1.0,
                          "Geometry_BezierSurface curve evaluation parameter must be in [0,1].");

    std::vector<MyMath::Vector3> workPoints(points);

    for (std::size_t level = workPoints.size() - 1; level > 0; --level)
    {
        for (std::size_t index = 0; index < level; ++index)
        {
            workPoints[index] =
                workPoints[index] * (1.0 - parameter) +
                workPoints[index + 1] * parameter;
        }
    }

    return workPoints[0];
}

MyMath::Vector3 Geometry_BezierSurface::evaluateSurface(
    const std::vector<MyMath::Vector3>& controlPoints,
    std::size_t uControlPointCount,
    std::size_t vControlPointCount,
    double u,
    double v)
{
    MYBREP_ASSERT_MESSAGE(uControlPointCount >= 1,
                          "Geometry_BezierSurface evaluation requires U control points.");
    MYBREP_ASSERT_MESSAGE(vControlPointCount >= 1,
                          "Geometry_BezierSurface evaluation requires V control points.");
    MYBREP_ASSERT_MESSAGE(controlPoints.size() == uControlPointCount * vControlPointCount,
                          "Geometry_BezierSurface evaluation control net dimensions are inconsistent.");

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

        vDirectionPoints.push_back(evaluateCurve(uDirectionPoints, u));
    }

    return evaluateCurve(vDirectionPoints, v);
}

void Geometry_BezierSurface::buildUDerivativeControlNet(
    const std::vector<MyMath::Vector3>& controlPoints,
    std::size_t uControlPointCount,
    std::size_t vControlPointCount,
    std::vector<MyMath::Vector3>& derivativeControlPoints)
{
    MYBREP_ASSERT_MESSAGE(uControlPointCount >= 2,
                          "Geometry_BezierSurface U derivative requires at least two U control points.");
    MYBREP_ASSERT_MESSAGE(controlPoints.size() == uControlPointCount * vControlPointCount,
                          "Geometry_BezierSurface U derivative control net dimensions are inconsistent.");

    const double degreeScale = static_cast<double>(uControlPointCount - 1);

    derivativeControlPoints.clear();
    derivativeControlPoints.reserve((uControlPointCount - 1) * vControlPointCount);

    for (std::size_t vIndex = 0; vIndex < vControlPointCount; ++vIndex)
    {
        for (std::size_t uIndex = 0; uIndex + 1 < uControlPointCount; ++uIndex)
        {
            const MyMath::Vector3& current =
                controlPoints[controlPointIndex(uIndex, vIndex, uControlPointCount)];
            const MyMath::Vector3& next =
                controlPoints[controlPointIndex(uIndex + 1, vIndex, uControlPointCount)];

            derivativeControlPoints.push_back((next - current) * degreeScale);
        }
    }
}

void Geometry_BezierSurface::buildVDerivativeControlNet(
    const std::vector<MyMath::Vector3>& controlPoints,
    std::size_t uControlPointCount,
    std::size_t vControlPointCount,
    std::vector<MyMath::Vector3>& derivativeControlPoints)
{
    MYBREP_ASSERT_MESSAGE(vControlPointCount >= 2,
                          "Geometry_BezierSurface V derivative requires at least two V control points.");
    MYBREP_ASSERT_MESSAGE(controlPoints.size() == uControlPointCount * vControlPointCount,
                          "Geometry_BezierSurface V derivative control net dimensions are inconsistent.");

    const double degreeScale = static_cast<double>(vControlPointCount - 1);

    derivativeControlPoints.clear();
    derivativeControlPoints.reserve(uControlPointCount * (vControlPointCount - 1));

    for (std::size_t vIndex = 0; vIndex + 1 < vControlPointCount; ++vIndex)
    {
        for (std::size_t uIndex = 0; uIndex < uControlPointCount; ++uIndex)
        {
            const MyMath::Vector3& current =
                controlPoints[controlPointIndex(uIndex, vIndex, uControlPointCount)];
            const MyMath::Vector3& next =
                controlPoints[controlPointIndex(uIndex, vIndex + 1, uControlPointCount)];

            derivativeControlPoints.push_back((next - current) * degreeScale);
        }
    }
}

}