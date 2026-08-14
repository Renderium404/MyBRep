#include "Topology_TVertex.h"

#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

Topology_TVertex::Topology_TVertex(const MyMath::Vector3& point)
    : m_point(point)
{
    MYBREP_ASSERT_MESSAGE(point.isFinite(),
                          "Topology_TVertex point must be finite.");
}

/// 几何数据

const MyMath::Vector3& Topology_TVertex::point() const
{
    return m_point;
}

}
