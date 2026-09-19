#include "MyEnvelope/Motion/MotionSegment.h"

#include <cassert>

namespace MyEnvelope
{

MotionSegment::MotionSegment(const MyMath::CoordinateSystem& start, const MyMath::CoordinateSystem& end)
    : m_start(start), m_end(end), m_translationVelocity(end.origin() - start.origin())
{
    assert(m_start.isValid());
    assert(m_end.isValid());
    assert(!m_start.isLeftHanded());
    assert(!m_end.isLeftHanded());

    const bool startOrientationValid = m_start.orientation(m_startOrientation);
    const bool endOrientationValid = m_end.orientation(m_endOrientation);

    assert(startOrientationValid);
    assert(endOrientationValid);

    if (MyMath::Quaternion::dot(m_startOrientation, m_endOrientation) < 0.0) m_endOrientation = -m_endOrientation;

    const MyMath::Quaternion relativeOrientation = m_startOrientation.conjugated() * m_endOrientation;
    MyMath::Vector3 localAxis;
    double angle = 0.0;
    relativeOrientation.toAxisAngle(localAxis, angle);

    m_localAngularVelocity = localAxis * angle;
    m_angularVelocity = m_start.mapVector(m_localAngularVelocity);
}

const MyMath::CoordinateSystem& MotionSegment::start() const
{
    return m_start;
}

const MyMath::CoordinateSystem& MotionSegment::end() const
{
    return m_end;
}

MyMath::CoordinateSystem MotionSegment::coordinateSystemAt(double t) const
{
    assert(t >= 0.0 && t <= 1.0);

    const MyMath::Vector3 origin = m_start.origin() + m_translationVelocity * t;
    const MyMath::Quaternion orientation = MyMath::Quaternion::slerp(m_startOrientation, m_endOrientation, t);

    assert(orientation.isUnit());
    return MyMath::CoordinateSystem::fromQuaternion(origin, orientation);
}

const MyMath::Vector3& MotionSegment::translationVelocity() const
{
    return m_translationVelocity;
}

const MyMath::Vector3& MotionSegment::angularVelocity() const
{
    return m_angularVelocity;
}

const MyMath::Vector3& MotionSegment::localAngularVelocity() const
{
    return m_localAngularVelocity;
}

MyMath::Vector3 MotionSegment::localTranslationVelocityAt(double t) const
{
    assert(t >= 0.0 && t <= 1.0);
    return coordinateSystemAt(t).unmapVector(m_translationVelocity);
}

MyMath::Vector3 MotionSegment::pointVelocityAt(const MyMath::Vector3& localPoint, double t) const
{
    assert(localPoint.isFinite());
    assert(t >= 0.0 && t <= 1.0);

    const MyMath::CoordinateSystem coordinateSystem = coordinateSystemAt(t);
    const MyMath::Vector3 rotatedPoint = coordinateSystem.mapVector(localPoint);

    return m_translationVelocity + MyMath::Vector3::cross(m_angularVelocity, rotatedPoint);
}

}
