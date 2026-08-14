#include "Geometry_Bezier2D.h"

#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

Geometry_Bezier2D::Geometry_Bezier2D(const std::vector<MyMath::Vector2>& controlPoints)
    : m_controlPoints(controlPoints)
{
    validateControlPoints(m_controlPoints);
}

/// 控制数据

std::size_t Geometry_Bezier2D::controlPointCount() const
{
    return m_controlPoints.size();
}

std::size_t Geometry_Bezier2D::degree() const
{
    MYBREP_ASSERT_MESSAGE(m_controlPoints.size() >= 2,
                          "Geometry_Bezier2D requires at least two control points.");

    return m_controlPoints.size() - 1;
}

const MyMath::Vector2& Geometry_Bezier2D::controlPoint(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(index < m_controlPoints.size(),
                          "Geometry_Bezier2D control point index is out of range.");

    return m_controlPoints[index];
}

const std::vector<MyMath::Vector2>& Geometry_Bezier2D::controlPoints() const
{
    return m_controlPoints;
}

/// 曲线类型

CurveKind Geometry_Bezier2D::kind() const
{
    return CurveKind::Bezier;
}

/// 定义域

bool Geometry_Bezier2D::isDomainBounded() const
{
    return true;
}

double Geometry_Bezier2D::domainStart() const
{
    return 0.0;
}

double Geometry_Bezier2D::domainEnd() const
{
    return 1.0;
}

/// 周期性

double Geometry_Bezier2D::period() const
{
    return 0.0;
}

/// 参数查询

MyMath::Vector2 Geometry_Bezier2D::pointAt(double parameter) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(parameter),
                          "Geometry_Bezier2D parameter is outside the natural parameter domain.");

    return evaluate(m_controlPoints, parameter);
}

MyMath::Vector2 Geometry_Bezier2D::firstDerivativeAt(double parameter) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(parameter),
                          "Geometry_Bezier2D derivative parameter is outside the natural parameter domain.");

    const std::size_t curveDegree = degree();
    const double degreeScale = static_cast<double>(curveDegree);
    std::vector<MyMath::Vector2> derivativeControlPoints;
    derivativeControlPoints.reserve(curveDegree);

    for (std::size_t index = 0; index < curveDegree; ++index)
    {
        derivativeControlPoints.push_back(
            (m_controlPoints[index + 1] - m_controlPoints[index]) * degreeScale);
    }

    return evaluate(derivativeControlPoints, parameter);
}

MyMath::Vector2 Geometry_Bezier2D::secondDerivativeAt(double parameter) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(parameter),
                          "Geometry_Bezier2D derivative parameter is outside the natural parameter domain.");

    const std::size_t curveDegree = degree();

    if (curveDegree == 1)
    {
        return MyMath::Vector2::zero();
    }

    const double degreeScale =
        static_cast<double>(curveDegree * (curveDegree - 1));
    std::vector<MyMath::Vector2> derivativeControlPoints;
    derivativeControlPoints.reserve(curveDegree - 1);

    for (std::size_t index = 0; index + 1 < curveDegree; ++index)
    {
        const MyMath::Vector2 secondDifference =
            m_controlPoints[index + 2] -
            m_controlPoints[index + 1] * 2.0 +
            m_controlPoints[index];

        derivativeControlPoints.push_back(secondDifference * degreeScale);
    }

    return evaluate(derivativeControlPoints, parameter);
}

/// 内部辅助

MyMath::Vector2 Geometry_Bezier2D::evaluate(const std::vector<MyMath::Vector2>& points,
                                             double parameter)
{
    MYBREP_ASSERT_MESSAGE(!points.empty(),
                          "Geometry_Bezier2D de Casteljau evaluation requires at least one control point.");
    MYBREP_ASSERT_MESSAGE(parameter >= 0.0 && parameter <= 1.0,
                          "Geometry_Bezier2D de Casteljau parameter must be in [0,1].");

    std::vector<MyMath::Vector2> workPoints(points);
    const double oneMinusParameter = 1.0 - parameter;

    for (std::size_t level = workPoints.size() - 1; level > 0; --level)
    {
        for (std::size_t index = 0; index < level; ++index)
        {
            workPoints[index] =
                workPoints[index] * oneMinusParameter +
                workPoints[index + 1] * parameter;
        }
    }

    MYBREP_ASSERT_MESSAGE(workPoints.front().isFinite(),
                          "Geometry_Bezier2D evaluation produced a non-finite result.");

    return workPoints.front();
}

void Geometry_Bezier2D::validateControlPoints(
    const std::vector<MyMath::Vector2>& controlPoints)
{
    MYBREP_ASSERT_MESSAGE(controlPoints.size() >= 2,
                          "Geometry_Bezier2D requires at least two control points.");

    for (std::size_t index = 0; index < controlPoints.size(); ++index)
    {
        MYBREP_ASSERT_MESSAGE(controlPoints[index].isFinite(),
                              "Geometry_Bezier2D control points must be finite.");
    }

    bool hasDifferentPoint = false;

    for (std::size_t index = 1; index < controlPoints.size(); ++index)
    {
        if (!controlPoints[index].isEqualTo(controlPoints[0], 0.0))
        {
            hasDifferentPoint = true;
            break;
        }
    }

    MYBREP_ASSERT_MESSAGE(hasDifferentPoint,
                          "Geometry_Bezier2D control points must not define a constant degenerate curve.");
}

}
