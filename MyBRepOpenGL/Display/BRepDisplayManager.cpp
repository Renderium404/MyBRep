#include "BRepDisplayManager.h"

namespace MyBRep
{
namespace Display
{

BRepTopologyBinding::BRepTopologyBinding()
    : topologyId(InvalidTopologyId)
    , resourceId(InvalidResourceId)
    , resourceType(BRepTopologyResourceType::Unknown)
{
}

bool BRepTopologyBinding::isValid() const
{
    return topology &&
           topologyId != InvalidTopologyId &&
           topology->id() == topologyId &&
           resourceId != InvalidResourceId &&
           resourceType != BRepTopologyResourceType::Unknown;
}

bool BRepTopologyBinding::isTopologyUnused() const
{
    return topology && topology->referenceCount() == 1;
}

BRepDisplayManager::BRepDisplayManager()
{
}

/// Instance <-> RenderItem

bool BRepDisplayManager::bindInstance(InstanceId instanceId, RenderItemId itemId)
{
    if (instanceId == InvalidInstanceId || itemId == InvalidRenderItemId) return false;

    ItemByInstanceMap::const_iterator instanceIterator = m_itemByInstance.find(instanceId);

    if (instanceIterator != m_itemByInstance.end())
    {
        return instanceIterator->second == itemId;
    }

    InstanceByItemMap::const_iterator itemIterator = m_instanceByItem.find(itemId);

    if (itemIterator != m_instanceByItem.end())
    {
        return itemIterator->second == instanceId;
    }

    m_itemByInstance[instanceId] = itemId;
    m_instanceByItem[itemId] = instanceId;
    return true;
}

bool BRepDisplayManager::unbindInstance(InstanceId instanceId)
{
    ItemByInstanceMap::iterator iterator = m_itemByInstance.find(instanceId);
    if (iterator == m_itemByInstance.end()) return false;

    m_instanceByItem.erase(iterator->second);
    m_itemByInstance.erase(iterator);
    return true;
}

bool BRepDisplayManager::containsInstance(InstanceId instanceId) const
{
    return m_itemByInstance.find(instanceId) != m_itemByInstance.end();
}

bool BRepDisplayManager::containsItem(RenderItemId itemId) const
{
    return m_instanceByItem.find(itemId) != m_instanceByItem.end();
}

RenderItemId BRepDisplayManager::itemId(InstanceId instanceId) const
{
    ItemByInstanceMap::const_iterator iterator = m_itemByInstance.find(instanceId);
    return iterator != m_itemByInstance.end() ? iterator->second : InvalidRenderItemId;
}

InstanceId BRepDisplayManager::instanceId(RenderItemId itemId) const
{
    InstanceByItemMap::const_iterator iterator = m_instanceByItem.find(itemId);
    return iterator != m_instanceByItem.end() ? iterator->second : InvalidInstanceId;
}

std::size_t BRepDisplayManager::instanceCount() const
{
    return m_itemByInstance.size();
}

/// Topology <-> Geometry Resource

bool BRepDisplayManager::bindTopology(const Topology_Object& topology,
                                      BRepTopologyResourceType resourceType,
                                      ResourceId resourceId)
{
    if (!topology.isValid() ||
        resourceType == BRepTopologyResourceType::Unknown ||
        resourceId == InvalidResourceId)
    {
        return false;
    }

    const TopologyId topologyId = topology.id();

    BindingByTopologyMap::const_iterator topologyIterator = m_bindingByTopology.find(topologyId);

    if (topologyIterator != m_bindingByTopology.end())
    {
        const BRepTopologyBinding& existing = topologyIterator->second;
        return existing.resourceId == resourceId &&existing.resourceType == resourceType;
    }

    TopologyByResourceMap::const_iterator resourceIterator = m_topologyByResource.find(resourceId);

    if (resourceIterator != m_topologyByResource.end()) return false;

    BRepTopologyBinding binding;
    binding.topology = topology.tObject();
    binding.topologyId = topologyId;
    binding.resourceId = resourceId;
    binding.resourceType = resourceType;

    m_bindingByTopology[topologyId] = binding;
    m_topologyByResource[resourceId] = topologyId;

    return true;
}

bool BRepDisplayManager::unbindTopology(TopologyId topologyId)
{
    BindingByTopologyMap::iterator iterator = m_bindingByTopology.find(topologyId);
    if (iterator == m_bindingByTopology.end()) return false;

    m_topologyByResource.erase(iterator->second.resourceId);
    m_bindingByTopology.erase(iterator);
    return true;
}

bool BRepDisplayManager::containsTopology(TopologyId topologyId) const
{
    return m_bindingByTopology.find(topologyId) != m_bindingByTopology.end();
}

bool BRepDisplayManager::containsResource(ResourceId resourceId) const
{
    return m_topologyByResource.find(resourceId) != m_topologyByResource.end();
}

BRepTopologyBinding BRepDisplayManager::binding(TopologyId topologyId) const
{
    BindingByTopologyMap::const_iterator iterator = m_bindingByTopology.find(topologyId);
    return iterator != m_bindingByTopology.end() ? iterator->second : BRepTopologyBinding();
}
const BRepTopologyBinding* BRepDisplayManager::bindingPointer(TopologyId topologyId) const
{
    BindingByTopologyMap::const_iterator iterator = m_bindingByTopology.find(topologyId);
    return iterator != m_bindingByTopology.end() ? &iterator->second : 0;
}
ResourceId BRepDisplayManager::resourceId(TopologyId topologyId) const
{
    BindingByTopologyMap::const_iterator iterator = m_bindingByTopology.find(topologyId);
    return iterator != m_bindingByTopology.end() ? iterator->second.resourceId : InvalidResourceId;
}

BRepTopologyResourceType BRepDisplayManager::resourceType(TopologyId topologyId) const
{
    BindingByTopologyMap::const_iterator iterator = m_bindingByTopology.find(topologyId);
    return iterator != m_bindingByTopology.end() ? iterator->second.resourceType : BRepTopologyResourceType::Unknown;
}

TopologyId BRepDisplayManager::topologyId(ResourceId resourceId) const
{
    TopologyByResourceMap::const_iterator iterator = m_topologyByResource.find(resourceId);
    return iterator != m_topologyByResource.end() ? iterator->second : InvalidTopologyId;
}
std::vector<TopologyId> BRepDisplayManager::topologyIds() const
{
    std::vector<TopologyId> result;
    result.reserve(m_bindingByTopology.size());

    for (BindingByTopologyMap::const_iterator iterator = m_bindingByTopology.begin(); iterator != m_bindingByTopology.end(); ++iterator)
    {
        result.push_back(iterator->first);
    }

    return result;
}
std::size_t BRepDisplayManager::topologyCount() const
{
    return m_bindingByTopology.size();
}

/// 整体状态

void BRepDisplayManager::clear()
{
    m_itemByInstance.clear();
    m_instanceByItem.clear();

    m_bindingByTopology.clear();
    m_topologyByResource.clear();
}

}
}