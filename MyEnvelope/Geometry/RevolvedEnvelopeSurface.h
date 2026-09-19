#ifndef MYENVELOPE_GEOMETRY_REVOLVEDENVELOPESURFACE_H
#define MYENVELOPE_GEOMETRY_REVOLVEDENVELOPESURFACE_H

#include "MyBRep/Geometry/Construction/Geometry_Revolved.h"
#include "MyEnvelope/Envelope/RevolvedEnvelopeBranch.h"
#include "MyEnvelope/Motion/MotionSegment.h"

namespace MyEnvelope
{

// 表示Geometry_Revolved一个有限母线段在一段刚体运动下产生的单条规则包络几何分支。
//
// U为当前有限母线段的规范化参数[0,1]，V为运动参数[0,1]。
// MyBRep::Geometry_Revolved母线使用局部XY平面，其中X为带符号旋转半径、Y为实体局部Z坐标。
// radialSign用于将带符号母线X映射为实际非负旋转半径。
//
// 当前类型只表示包络专属连续几何，不继承MyBRep::Geometry_Surface；
// 后续进入Topology_Face阶段时再统一补齐MyBRep曲面接口要求的二阶偏导。
class RevolvedEnvelopeSurface
{
public:
    RevolvedEnvelopeSurface(const MyBRep::Geometry_Revolved::ProfileSegment& profileSegment,
                            double radialSign,
                            const MotionSegment& motion,
                            RevolvedEnvelopeBranch branch,
                            double epsilon = MyMath::Vector3::DefaultEpsilon);

    const MyBRep::Geometry_Revolved::ProfileSegment& profileSegment() const;
    double radialSign() const;
    const MotionSegment& motion() const;
    RevolvedEnvelopeBranch branch() const;
    double epsilon() const;

    bool isUParameterInDomain(double u) const;
    bool isVParameterInDomain(double v) const;
    bool isParameterInDomain(double u, double v) const;

    bool isDefinedAt(double u, double v) const;
    bool isRegularAt(double u, double v) const;

    // 返回当前包络分支在源回转体上的旋转角，结果规范化到[0,2π)。
    double sourceAngleAt(double u, double v) const;

    bool isUDomainBounded() const;
    double uDomainStart() const;
    double uDomainEnd() const;

    bool isVDomainBounded() const;
    double vDomainStart() const;
    double vDomainEnd() const;

    double uPeriod() const;
    double vPeriod() const;

    MyMath::Vector3 pointAt(double u, double v) const;
    MyMath::Vector3 firstDerivativeUAt(double u, double v) const;
    MyMath::Vector3 firstDerivativeVAt(double u, double v) const;

private:
    struct ProfileData
    {
        double radius;
        double axial;
        double radiusDerivative;
        double axialDerivative;
        double radiusSecondDerivative;
        double axialSecondDerivative;
        double normalRadial;
        double normalAxial;
        double normalRadialDerivative;
        double normalAxialDerivative;
    };

    struct MotionData
    {
        MyMath::CoordinateSystem coordinateSystem;
        MyMath::Vector3 translationVelocity;
        MyMath::Vector3 translationVelocityDerivative;
        MyMath::Vector3 angularVelocity;
    };

    struct EquationData
    {
        double a;
        double b;
        double c;
    };

private:
    double curveParameterAt(double u) const;
    bool buildProfileData(double u, ProfileData& result) const;
    bool buildMotionData(double v, MotionData& result) const;
    bool buildEquationData(const ProfileData& profile, const MotionData& motion, EquationData& result) const;
    bool solveAngle(const EquationData& equation, double& angle, bool& regular) const;

    static double normalizeAngle(double angle);

private:
    MyBRep::Geometry_Revolved::ProfileSegment m_profileSegment;
    double m_radialSign;
    MotionSegment m_motion;
    RevolvedEnvelopeBranch m_branch;
    double m_epsilon;
};

}

#endif // MYENVELOPE_GEOMETRY_REVOLVEDENVELOPESURFACE_H
