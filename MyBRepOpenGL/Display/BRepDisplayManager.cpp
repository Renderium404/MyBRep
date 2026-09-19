#include "BRepDisplayManager.h"

namespace MyBRep
{
namespace Display
{

BRepPartBinding::BRepPartBinding()
    : topologyId(InvalidTopologyId)
    , resourceId(InvalidResourceId)
    , partId(InvalidRenderPartId)
    , resourceType(BRepTopologyResourceType::Unknown)
{
}

bool BRepPartBinding::isValid() const
{
    return topologyId != InvalidTopologyId &&
           resourceId != InvalidResourceId &&
           partId != InvalidRenderPartId &&
           resourceType != BRepTopologyResourceType::Unknown;
}

BRepDisplayManager::BRepDisplayManager()
{
}

/// Instance <-> RenderItem

bool BRepDisplayManager::bindInstance(InstanceId instanceId, RenderItemId itemId)
{
    if (instanceId == InvalidInstanceId || itemId == InvalidRenderItemId)
    {
        return false;
    }

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

    if (iterator == m_itemByInstance.end())
    {
        return false;
    }

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

/// Topology <-> Resource / RenderPart

bool BRepDisplayManager::bindTopology(TopologyId topologyId, BRepTopologyResourceType resourceType,
                                      ResourceId resourceId, RenderPartId partId)
{
    if (topologyId == InvalidTopologyId || resourceType == BRepTopologyResourceType::Unknown ||
        resourceId == InvalidResourceId || partId == InvalidRenderPartId)
    {
        return false;
    }

    BindingByTopologyMap::const_iterator topologyIterator = m_bindingByTopology.find(topologyId);

    if (topologyIterator != m_bindingByTopology.end())
    {
        const BRepPartBinding& existing = topologyIterator->second;
        return existing.resourceId == resourceId &&
               existing.partId == partId &&
               existing.resourceType == resourceType;
    }

    TopologyByResourceMap::const_iterator resourceIterator = m_topologyByResource.find(resourceId);

    if (resourceIterator != m_topologyByResource.end())
    {
        return resourceIterator->second == topologyId;
    }

    TopologyByPartMap::const_iterator partIterator = m_topologyByPart.find(partId);

    if (partIterator != m_topologyByPart.end())
    {
        return partIterator->second == topologyId;
    }

    BRepPartBinding record;
    record.topologyId = topologyId;
    record.resourceId = resourceId;
    record.partId = partId;
    record.resourceType = resourceType;

    m_bindingByTopology[topologyId] = record;
    m_topologyByResource[resourceId] = topologyId;
    m_topologyByPart[partId] = topologyId;
    return true;
}

bool BRepDisplayManager::unbindTopology(TopologyId topologyId)
{
    BindingByTopologyMap::iterator iterator = m_bindingByTopology.find(topologyId);

    if (iterator == m_bindingByTopology.end())
    {
        return false;
    }

    m_topologyByResource.erase(iterator->second.resourceId);
    m_topologyByPart.erase(iterator->second.partId);
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

bool BRepDisplayManager::containsPart(RenderPartId partId) const
{
    return m_topologyByPart.find(partId) != m_topologyByPart.end();
}

BRepPartBinding BRepDisplayManager::binding(TopologyId topologyId) const
{
    BindingByTopologyMap::const_iterator iterator = m_bindingByTopology.find(topologyId);
    return iterator != m_bindingByTopology.end() ? iterator->second : BRepPartBinding();
}

ResourceId BRepDisplayManager::resourceId(TopologyId topologyId) const
{
    BindingByTopologyMap::const_iterator iterator = m_bindingByTopology.find(topologyId);
    return iterator != m_bindingByTopology.end() ? iterator->second.resourceId : InvalidResourceId;
}

RenderPartId BRepDisplayManager::partId(TopologyId topologyId) const
{
    BindingByTopologyMap::const_iterator iterator = m_bindingByTopology.find(topologyId);
    return iterator != m_bindingByTopology.end() ? iterator->second.partId : InvalidRenderPartId;
}

TopologyId BRepDisplayManager::topologyIdByResource(ResourceId resourceId) const
{
    TopologyByResourceMap::const_iterator iterator = m_topologyByResource.find(resourceId);
    return iterator != m_topologyByResource.end() ? iterator->second : InvalidTopologyId;
}

TopologyId BRepDisplayManager::topologyIdByPart(RenderPartId partId) const
{
    TopologyByPartMap::const_iterator iterator = m_topologyByPart.find(partId);
    return iterator != m_topologyByPart.end() ? iterator->second : InvalidTopologyId;
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
    m_topologyByPart.clear();
}

}
}
