#include "Topology_TShape.h"

#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

Topology_TShape::Topology_TShape(const Foundation::RefPtr<const Geometry_Shape>& geometry)
    : m_geometry(geometry)
{
    MYBREP_ASSERT_MESSAGE(geometry,
                          "Topology_TShape geometry must be non-null.");
}

/// 几何内核

const Geometry_Shape& Topology_TShape::geometry() const
{
    return *m_geometry;
}

const Foundation::RefPtr<const Geometry_Shape>& Topology_TShape::geometryResource() const
{
    return m_geometry;
}

}