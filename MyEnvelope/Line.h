#ifndef MYMATH_LINE_H
#define MYMATH_LINE_H

#include "Profile.h"

namespace MyMath
{

// (r, h) 平面中的直线段母线。
// 起点对应 l = 0，终点对应 l = length()。
class Line : public Profile
{
public:
    // 使用起点和终点构造直线段，调用者需保证两点不重合。
    Line(const Vector2& start, const Vector2& end);

    Vector2 normalAtL(double l, double epsilon = DefaultEpsilon) const override;
    double length() const override { return m_length; }
    double r(double l) const override;
    double h(double l) const override;

    Vector2 start() const { return m_start; }
    Vector2 end() const { return m_end; }
    Vector2 direction() const { return m_direction; }
    Vector2 normal() const { return m_normal; }

private:
    Vector2 m_start;
    Vector2 m_end;
    Vector2 m_direction;   // 从 start 指向 end 的单位切向量
    Vector2 m_normal;      // 单位法向量
    double m_length;
};

}

#endif // MYMATH_LINE_H