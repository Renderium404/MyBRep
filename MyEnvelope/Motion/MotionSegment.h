#ifndef MYENVELOPE_MOTION_MOTIONSEGMENT_H
#define MYENVELOPE_MOTION_MOTIONSEGMENT_H

#include "MyMath/CoordinateSystem.h"
#include "MyMath/Quaternion.h"
#include "MyMath/Vector3.h"

namespace MyEnvelope
{

// 表示两个右手正交坐标系之间的一段刚体运动。
// 位置按规范化参数t线性插值，方向按最短四元数弧执行SLERP，t的定义域固定为[0,1]。
class MotionSegment
{
public:
    MotionSegment(const MyMath::CoordinateSystem& start, const MyMath::CoordinateSystem& end);

    const MyMath::CoordinateSystem& start() const;
    const MyMath::CoordinateSystem& end() const;

    // 返回规范化运动参数t对应的坐标系。
    MyMath::CoordinateSystem coordinateSystemAt(double t) const;

    // 返回原点相对于规范化运动参数t的世界坐标导数。
    const MyMath::Vector3& translationVelocity() const;

    // 返回刚体相对于规范化运动参数t的恒定世界角速度向量。
    const MyMath::Vector3& angularVelocity() const;

    // 返回刚体相对于规范化运动参数t的恒定局部角速度向量。
    const MyMath::Vector3& localAngularVelocity() const;

    // 返回世界平移速度在当前运动坐标系中的局部分量。
    MyMath::Vector3 localTranslationVelocityAt(double t) const;

    // 返回指定局部点在当前运动参数处相对于t的世界速度。
    MyMath::Vector3 pointVelocityAt(const MyMath::Vector3& localPoint, double t) const;

private:
    MyMath::CoordinateSystem m_start;
    MyMath::CoordinateSystem m_end;
    MyMath::Quaternion m_startOrientation;
    MyMath::Quaternion m_endOrientation;
    MyMath::Vector3 m_translationVelocity;
    MyMath::Vector3 m_angularVelocity;
    MyMath::Vector3 m_localAngularVelocity;
};

}

#endif // MYENVELOPE_MOTION_MOTIONSEGMENT_H
