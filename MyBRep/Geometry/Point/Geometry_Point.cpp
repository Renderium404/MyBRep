#include "Geometry_Point.h"

#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

Geometry_Point::Geometry_Point(const MyMath::Vector3& position)
    : m_position(position)
{
    MYBREP_ASSERT_MESSAGE(position.isFinite(), "Geometry_Point position must be finite.");
}

/// 几何数据

const MyMath::Vector3& Geometry_Point::position() const
{
    return m_position;
}

}