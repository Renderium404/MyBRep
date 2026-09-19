#include "BRepSolidGeometryResource.h"

namespace MyBRep
{
namespace Display
{

BRepSolidGeometryResource::BRepSolidGeometryResource()
    : m_id(InvalidBRepSolidGeometryResourceId)
    , m_surfaceGeometryId(InvalidResourceId)
    , m_wireframeGeometryId(InvalidResourceId)
{
}

BRepSolidGeometryResource::BRepSolidGeometryResource(
    BRepSolidGeometryResourceId id,
    const Topology_Solid& topology,
    ResourceId surfaceGeometryId,
    ResourceId wireframeGeometryId)
    : m_id(id)
    , m_topology(topology)
    , m_surfaceGeometryId(surfaceGeometryId)
    , m_wireframeGeometryId(wireframeGeometryId)
{
}

bool BRepSolidGeometryResource::isValid() const
{
    return m_id != InvalidBRepSolidGeometryResourceId &&
           m_topology.isValid() &&
           m_surfaceGeometryId != InvalidResourceId &&
           m_wireframeGeometryId != InvalidResourceId;
}

bool BRepSolidGeometryResource::matches(const Topology_Solid& topology) const
{
    return isValid() &&
           topology.isValid() &&
           m_topology.isSame(topology) &&
           m_topology.orientation() == topology.orientation();
}

BRepSolidGeometryResourceId BRepSolidGeometryResource::id() const
{
    return m_id;
}

const Topology_Solid& BRepSolidGeometryResource::topology() const
{
    return m_topology;
}

ResourceId BRepSolidGeometryResource::surfaceGeometryId() const
{
    return m_surfaceGeometryId;
}

ResourceId BRepSolidGeometryResource::wireframeGeometryId() const
{
    return m_wireframeGeometryId;
}

}
}