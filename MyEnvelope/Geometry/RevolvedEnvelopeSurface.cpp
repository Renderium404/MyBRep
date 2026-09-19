#include "MyEnvelope/Geometry/RevolvedEnvelopeSurface.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

namespace
{

const double Pi = 3.1415926535897932384626433832795;
const double TwoPi = Pi * 2.0;

bool isFiniteValue(double value)
{
    const double infinity = (std::numeric_limits<double>::infinity)();
    return value == value && value != infinity && value != -infinity;
}

double clampValue(double value, double minimum, double maximum)
{
    return (std::max)(minimum, (std::min)(maximum, value));
}

}

namespace MyEnvelope
{

RevolvedEnvelopeSurface::RevolvedEnvelopeSurface(const MyBRep::Geometry_Revolved::ProfileSegment& profileSegment,
                                                 double radialSign,
                                                 const MotionSegment& motion,
                                                 RevolvedEnvelopeBranch branch,
                                                 double epsilon)
    : m_profileSegment(profileSegment), m_radialSign(radialSign), m_motion(motion), m_branch(branch), m_epsilon(epsilon)
{
    assert(m_profileSegment.curve);
    assert(isFiniteValue(m_profileSegment.firstParameter));
    assert(isFiniteValue(m_profileSegment.lastParameter));
    assert(m_profileSegment.firstParameter != m_profileSegment.lastParameter);
    assert(m_radialSign == 1.0 || m_radialSign == -1.0);
    assert(isFiniteValue(m_epsilon) && m_epsilon > 0.0);
}

const MyBRep::Geometry_Revolved::ProfileSegment& RevolvedEnvelopeSurface::profileSegment() const
{
    return m_profileSegment;
}

double RevolvedEnvelopeSurface::radialSign() const
{
    return m_radialSign;
}

const MotionSegment& RevolvedEnvelopeSurface::motion() const
{
    return m_motion;
}

RevolvedEnvelopeBranch RevolvedEnvelopeSurface::branch() const
{
    return m_branch;
}

double RevolvedEnvelopeSurface::epsilon() const
{
    return m_epsilon;
}

bool RevolvedEnvelopeSurface::isUParameterInDomain(double u) const
{
    return u >= 0.0 && u <= 1.0;
}

bool RevolvedEnvelopeSurface::isVParameterInDomain(double v) const
{
    return v >= 0.0 && v <= 1.0;
}

bool RevolvedEnvelopeSurface::isParameterInDomain(double u, double v) const
{
    return isUParameterInDomain(u) && isVParameterInDomain(v);
}

bool RevolvedEnvelopeSurface::isDefinedAt(double u, double v) const
{
    if (!isParameterInDomain(u, v)) return false;

    ProfileData profile;
    MotionData motionData;
    EquationData equation;

    if (!buildProfileData(u, profile) || !buildMotionData(v, motionData) || !buildEquationData(profile, motionData, equation)) return false;

    double angle = 0.0;
    bool regular = false;
    return solveAngle(equation, angle, regular);
}

bool RevolvedEnvelopeSurface::isRegularAt(double u, double v) const
{
    if (!isParameterInDomain(u, v)) return false;

    ProfileData profile;
    MotionData motionData;
    EquationData equation;

    if (!buildProfileData(u, profile) || !buildMotionData(v, motionData) || !buildEquationData(profile, motionData, equation)) return false;

    double angle = 0.0;
    bool regular = false;
    return solveAngle(equation, angle, regular) && regular;
}

double RevolvedEnvelopeSurface::sourceAngleAt(double u, double v) const
{
    assert(isParameterInDomain(u, v));

    ProfileData profile;
    MotionData motionData;
    EquationData equation;

    const bool profileBuilt = buildProfileData(u, profile);
    const bool motionBuilt = buildMotionData(v, motionData);
    const bool equationBuilt = profileBuilt && motionBuilt && buildEquationData(profile, motionData, equation);

    assert(profileBuilt);
    assert(motionBuilt);
    assert(equationBuilt);

    double angle = 0.0;
    bool regular = false;
    const bool angleSolved = equationBuilt && solveAngle(equation, angle, regular);

    assert(angleSolved);

    (void)profileBuilt;
    (void)motionBuilt;
    (void)equationBuilt;
    (void)angleSolved;

    return angle;
}

bool RevolvedEnvelopeSurface::isUDomainBounded() const
{
    return true;
}

double RevolvedEnvelopeSurface::uDomainStart() const
{
    return 0.0;
}

double RevolvedEnvelopeSurface::uDomainEnd() const
{
    return 1.0;
}

bool RevolvedEnvelopeSurface::isVDomainBounded() const
{
    return true;
}

double RevolvedEnvelopeSurface::vDomainStart() const
{
    return 0.0;
}

double RevolvedEnvelopeSurface::vDomainEnd() const
{
    return 1.0;
}

double RevolvedEnvelopeSurface::uPeriod() const
{
    return 0.0;
}

double RevolvedEnvelopeSurface::vPeriod() const
{
    return 0.0;
}

MyMath::Vector3 RevolvedEnvelopeSurface::pointAt(double u, double v) const
{
    assert(isDefinedAt(u, v));

    ProfileData profile;
    const bool profileBuilt = buildProfileData(u, profile);
    assert(profileBuilt);
    (void)profileBuilt;

    const double angle = sourceAngleAt(u, v);
    const MyMath::Vector3 localPoint(profile.radius * std::cos(angle), profile.radius * std::sin(angle), profile.axial);

    return m_motion.coordinateSystemAt(v).toGlobal(localPoint);
}

MyMath::Vector3 RevolvedEnvelopeSurface::firstDerivativeUAt(double u, double v) const
{
    assert(isRegularAt(u, v));

    ProfileData profile;
    MotionData motionData;
    EquationData equation;

    const bool profileBuilt = buildProfileData(u, profile);
    const bool motionBuilt = buildMotionData(v, motionData);
    const bool equationBuilt = profileBuilt && motionBuilt && buildEquationData(profile, motionData, equation);

    assert(profileBuilt);
    assert(motionBuilt);
    assert(equationBuilt);

    double angle = 0.0;
    bool regular = false;
    const bool angleSolved = equationBuilt && solveAngle(equation, angle, regular);

    assert(angleSolved && regular);

    (void)profileBuilt;
    (void)motionBuilt;
    (void)equationBuilt;
    (void)angleSolved;

    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    const double k = profile.normalRadial * profile.axial - profile.normalAxial * profile.radius;

    const double kDerivative =
        profile.normalRadialDerivative * profile.axial +
        profile.normalRadial * profile.axialDerivative -
        profile.normalAxialDerivative * profile.radius -
        profile.normalAxial * profile.radiusDerivative;

    const double aDerivative =
        profile.normalRadialDerivative * motionData.translationVelocity.x() +
        kDerivative * motionData.angularVelocity.y();

    const double bDerivative =
        profile.normalRadialDerivative * motionData.translationVelocity.y() -
        kDerivative * motionData.angularVelocity.x();

    const double cDerivative =
        profile.normalAxialDerivative * motionData.translationVelocity.z();

    const double equationAngleDerivative = -equation.a * sine + equation.b * cosine;
    const double equationUDerivative = aDerivative * cosine + bDerivative * sine + cDerivative;
    const double angleDerivative = -equationUDerivative / equationAngleDerivative;

    const MyMath::Vector3 localDerivative(
        profile.radiusDerivative * cosine - profile.radius * sine * angleDerivative,
        profile.radiusDerivative * sine + profile.radius * cosine * angleDerivative,
        profile.axialDerivative);

    return motionData.coordinateSystem.mapVector(localDerivative);
}

MyMath::Vector3 RevolvedEnvelopeSurface::firstDerivativeVAt(double u, double v) const
{
    assert(isRegularAt(u, v));

    ProfileData profile;
    MotionData motionData;
    EquationData equation;

    const bool profileBuilt = buildProfileData(u, profile);
    const bool motionBuilt = buildMotionData(v, motionData);
    const bool equationBuilt = profileBuilt && motionBuilt && buildEquationData(profile, motionData, equation);

    assert(profileBuilt);
    assert(motionBuilt);
    assert(equationBuilt);

    double angle = 0.0;
    bool regular = false;
    const bool angleSolved = equationBuilt && solveAngle(equation, angle, regular);

    assert(angleSolved && regular);

    (void)profileBuilt;
    (void)motionBuilt;
    (void)equationBuilt;
    (void)angleSolved;

    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);

    const double aDerivative = profile.normalRadial * motionData.translationVelocityDerivative.x();
    const double bDerivative = profile.normalRadial * motionData.translationVelocityDerivative.y();
    const double cDerivative = profile.normalAxial * motionData.translationVelocityDerivative.z();

    const double equationAngleDerivative = -equation.a * sine + equation.b * cosine;
    const double equationVDerivative = aDerivative * cosine + bDerivative * sine + cDerivative;
    const double angleDerivative = -equationVDerivative / equationAngleDerivative;

    const MyMath::Vector3 localPoint(profile.radius * cosine, profile.radius * sine, profile.axial);
    const MyMath::Vector3 localAngleDerivative(-profile.radius * sine, profile.radius * cosine, 0.0);

    return m_motion.pointVelocityAt(localPoint, v) +
           motionData.coordinateSystem.mapVector(localAngleDerivative * angleDerivative);
}

double RevolvedEnvelopeSurface::curveParameterAt(double u) const
{
    assert(isUParameterInDomain(u));
    return m_profileSegment.firstParameter +
           (m_profileSegment.lastParameter - m_profileSegment.firstParameter) * u;
}

bool RevolvedEnvelopeSurface::buildProfileData(double u, ProfileData& result) const
{
    if (!isUParameterInDomain(u) || !m_profileSegment.curve) return false;

    const double parameterSpan = m_profileSegment.lastParameter - m_profileSegment.firstParameter;
    const double parameter = curveParameterAt(u);
    const MyBRep::Geometry_Curve& curve = *m_profileSegment.curve;

    if (!curve.isParameterInDomain(parameter)) return false;

    const MyMath::Vector3 point = curve.pointAt(parameter);
    const MyMath::Vector3 sourceFirstDerivative = curve.firstDerivativeAt(parameter);
    const MyMath::Vector3 sourceSecondDerivative = curve.secondDerivativeAt(parameter);

    if (!point.isFinite() || !sourceFirstDerivative.isFinite() || !sourceSecondDerivative.isFinite()) return false;

    const MyMath::Vector3 firstDerivative = sourceFirstDerivative * parameterSpan;
    const MyMath::Vector3 secondDerivative = sourceSecondDerivative * (parameterSpan * parameterSpan);

    const double radius = m_radialSign * point.x();
    const double radiusDerivative = m_radialSign * firstDerivative.x();
    const double axialDerivative = firstDerivative.y();
    const double tangentLengthSquared = radiusDerivative * radiusDerivative + axialDerivative * axialDerivative;

    if (radius <= m_epsilon) return false;
    if (tangentLengthSquared <= m_epsilon * m_epsilon) return false;

    const double tangentLength = std::sqrt(tangentLengthSquared);
    const double tangentDotSecond =
        radiusDerivative * (m_radialSign * secondDerivative.x()) +
        axialDerivative * secondDerivative.y();

    const double tangentLengthCubed = tangentLengthSquared * tangentLength;

    result.radius = radius;
    result.axial = point.y();
    result.radiusDerivative = radiusDerivative;
    result.axialDerivative = axialDerivative;
    result.radiusSecondDerivative = m_radialSign * secondDerivative.x();
    result.axialSecondDerivative = secondDerivative.y();

    result.normalRadial = axialDerivative / tangentLength;
    result.normalAxial = -radiusDerivative / tangentLength;

    result.normalRadialDerivative =
        (result.axialSecondDerivative * tangentLengthSquared - axialDerivative * tangentDotSecond) /
        tangentLengthCubed;

    result.normalAxialDerivative =
        (-result.radiusSecondDerivative * tangentLengthSquared + radiusDerivative * tangentDotSecond) /
        tangentLengthCubed;

    return true;
}

bool RevolvedEnvelopeSurface::buildMotionData(double v, MotionData& result) const
{
    if (!isVParameterInDomain(v)) return false;

    result.coordinateSystem = m_motion.coordinateSystemAt(v);
    result.translationVelocity = m_motion.localTranslationVelocityAt(v);
    result.angularVelocity = m_motion.localAngularVelocity();

    result.translationVelocityDerivative =
        MyMath::Vector3::cross(result.angularVelocity, result.translationVelocity) * -1.0;

    return result.translationVelocity.isFinite() &&
           result.translationVelocityDerivative.isFinite() &&
           result.angularVelocity.isFinite();
}

bool RevolvedEnvelopeSurface::buildEquationData(const ProfileData& profile,
                                                const MotionData& motionData,
                                                EquationData& result) const
{
    const double k = profile.normalRadial * profile.axial - profile.normalAxial * profile.radius;

    result.a =
        profile.normalRadial * motionData.translationVelocity.x() +
        k * motionData.angularVelocity.y();

    result.b =
        profile.normalRadial * motionData.translationVelocity.y() -
        k * motionData.angularVelocity.x();

    result.c =
        profile.normalAxial * motionData.translationVelocity.z();

    return isFiniteValue(result.a) && isFiniteValue(result.b) && isFiniteValue(result.c);
}

bool RevolvedEnvelopeSurface::solveAngle(const EquationData& equation, double& angle, bool& regular) const
{
    const double d = std::sqrt(equation.a * equation.a + equation.b * equation.b);
    const double maximumCoefficient =
        (std::max)(std::fabs(equation.a), (std::max)(std::fabs(equation.b), std::fabs(equation.c)));
    const double scale = (std::max)(1.0, maximumCoefficient);
    const double tolerance = m_epsilon * scale;

    if (d <= tolerance) return false;
    if (std::fabs(equation.c) > d + tolerance) return false;

    const double ratio = clampValue(-equation.c / d, -1.0, 1.0);
    const double phi = std::atan2(equation.b, equation.a);
    const double delta = std::acos(ratio);

    angle = normalizeAngle(m_branch == RevolvedEnvelopeBranch::Plus ? phi + delta : phi - delta);

    const double equationAngleDerivative =
        -equation.a * std::sin(angle) + equation.b * std::cos(angle);

    regular = std::fabs(equationAngleDerivative) > tolerance;
    return true;
}

double RevolvedEnvelopeSurface::normalizeAngle(double angle)
{
    double result = std::fmod(angle, TwoPi);
    if (result < 0.0) result += TwoPi;
    return result;
}

}
