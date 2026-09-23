#ifndef MYMATH_ARC_H
#define MYMATH_ARC_H

#include "Profile.h"

namespace MyMath
{

// (r, h) 平面中的圆弧母线。
// 角度单位为弧度，逆时针为正方向。
// 当 endAngle > startAngle 时为逆时针弧，反之为顺时针弧。
class Arc : public Profile
{
public:
    // 使用圆心、半径、起始角和终止角构造圆弧。
    // 调用者需保证半径为正且角度有限。
    Arc(const Vector2& center, double radius,
        double startAngle, double endAngle);

    Vector2 normalAtL(double l, double epsilon = DefaultEpsilon) const override;
    double length() const override { return m_length; }
    double r(double l) const override;
    double h(double l) const override;

    Vector2 center() const { return m_center; }
    double radius() const { return m_radius; }
    double startAngle() const { return m_startAngle; }
    double endAngle() const { return m_endAngle; }

private:
    // 返回弧长 l 对应的极角。
    double angleAtL(double l) const;

private:
    Vector2 m_center;
    double m_radius;
    double m_startAngle;
    double m_endAngle;
    double m_length;
    double m_sweepSign; // +1 逆时针，-1 顺时针
};

}

#endif // MYMATH_ARC_H