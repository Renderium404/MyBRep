#include "Geometry_Bezier.h"

namespace MyBRep
{

Geometry_Bezier::Geometry_Bezier(const std::vector<MyMath::Vector3>& controlPoints)
    : m_controlPoints(controlPoints)
{
    MYBREP_ASSERT_MESSAGE(m_controlPoints.size() >= 2,
                          "Geometry_Bezier requires at least two control points.");

    for (std::size_t index = 0; index < m_controlPoints.size(); ++index)
    {
        MYBREP_ASSERT_MESSAGE(m_controlPoints[index].isFinite(),
                              "Geometry_Bezier control points must be finite.");
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
                          "Geometry_Bezier control points must not all coincide.");
}

/// 控制数据

std::size_t Geometry_Bezier::controlPointCount() const
{
    return m_controlPoints.size();
}

std::size_t Geometry_Bezier::degree() const
{
    return m_controlPoints.size() - 1;
}

const MyMath::Vector3& Geometry_Bezier::controlPoint(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(index < m_controlPoints.size(),
                          "Geometry_Bezier control point index is out of range.");

    return m_controlPoints[index];
}

const std::vector<MyMath::Vector3>& Geometry_Bezier::controlPoints() const
{
    return m_controlPoints;
}

/// 曲线类型

CurveKind Geometry_Bezier::kind() const
{
    return CurveKind::Bezier;
}

/// 定义域

bool Geometry_Bezier::isDomainBounded() const
{
    return true;
}

double Geometry_Bezier::domainStart() const
{
    return 0.0;
}

double Geometry_Bezier::domainEnd() const
{
    return 1.0;
}

/// 周期性

double Geometry_Bezier::period() const
{
    return 0.0;
}

/// 参数查询

MyMath::Vector3 Geometry_Bezier::pointAt(double parameter) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(parameter),"Geometry_Bezier parameter is outside the natural parameter domain.");

    return evaluate(m_controlPoints, parameter);
}

MyMath::Vector3 Geometry_Bezier::firstDerivativeAt(double parameter) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(parameter),
                          "Geometry_Bezier derivative parameter is outside the natural parameter domain.");

    const std::size_t curveDegree = degree();
    const double degreeScale = static_cast<double>(curveDegree);

    std::vector<MyMath::Vector3> derivativeControlPoints;
    derivativeControlPoints.reserve(curveDegree);

    for (std::size_t index = 0; index < curveDegree; ++index)
    {
        derivativeControlPoints.push_back((m_controlPoints[index + 1] - m_controlPoints[index]) * degreeScale);
    }

    return evaluate(derivativeControlPoints, parameter);
}

MyMath::Vector3 Geometry_Bezier::secondDerivativeAt(double parameter) const
{
    MYBREP_ASSERT_MESSAGE(isParameterInDomain(parameter),
                          "Geometry_Bezier derivative parameter is outside the natural parameter domain.");

    const std::size_t curveDegree = degree();

    if (curveDegree == 1)
    {
        return MyMath::Vector3(0.0, 0.0, 0.0);
    }

    const double degreeScale = static_cast<double>(curveDegree * (curveDegree - 1));

    std::vector<MyMath::Vector3> derivativeControlPoints;
    derivativeControlPoints.reserve(curveDegree - 1);

    for (std::size_t index = 0; index + 1 < curveDegree; ++index)
    {
        const MyMath::Vector3 secondDifference =
            m_controlPoints[index + 2] -
            m_controlPoints[index + 1] * 2.0 +
            m_controlPoints[index];

        derivativeControlPoints.push_back(secondDifference * degreeScale);
    }

    return evaluate(derivativeControlPoints, parameter);
}

/// 内部辅助

MyMath::Vector3 Geometry_Bezier::evaluate(const std::vector<MyMath::Vector3>& points, double parameter)
{
    MYBREP_ASSERT_MESSAGE(!points.empty(),
                          "Geometry_Bezier evaluation requires at least one control point.");
    MYBREP_ASSERT_MESSAGE(parameter >= 0.0 && parameter <= 1.0,
                          "Geometry_Bezier evaluation parameter must be in [0,1].");

    std::vector<MyMath::Vector3> workPoints(points);

    for (std::size_t level = workPoints.size() - 1; level > 0; --level)
    {
        for (std::size_t index = 0; index < level; ++index)
        {
            workPoints[index] =workPoints[index] * (1.0 - parameter) +workPoints[index + 1] * parameter;
        }
    }

    return workPoints[0];
}

}