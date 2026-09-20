#ifndef MYBREPOPENGL_DISPLAY_BREPDISPLAYMANAGER_H
#define MYBREPOPENGL_DISPLAY_BREPDISPLAYMANAGER_H

#include <cstddef>
#include <map>
#include <vector>
#include "MyBRep/Instance/Instance.h"
#include "MyBRep/Topology/Topology_Id.h"
#include "MyOpenGL/Core/Resource.h"
#include "MyOpenGL/Item/RenderItem.h"
#include "MyBRep/Topology/Topology_Object.h"
namespace MyBRep
{
namespace Display
{

// B-Rep拓扑在显示层对应的Geometry资源类型。
// Topology_Orientation不参与Topology资源身份。
enum class BRepTopologyResourceType
{
    Unknown,
    Face,
    Edge
};

// 一个Topology实体对应的MyOpenGL Geometry资源绑定。
// 同一个TopologyId在一个显示环境中只对应一份Geometry Resource。
struct BRepTopologyBinding
{
    BRepTopologyBinding();

    bool isValid() const;
    bool isTopologyUnused() const;

    Foundation::RefPtr<Topology_TObject> topology;
    TopologyId topologyId;
    ResourceId resourceId;
    BRepTopologyResourceType resourceType;
};

// 管理MyBRep与MyOpenGL之间的稳定身份映射。
// 本类只保存身份关系，不拥有RenderItem或Resource，也不管理RenderPart组织关系。
class BRepDisplayManager
{
public:
    BRepDisplayManager();

    /// Instance <-> RenderItem

    // 建立InstanceId与RenderItemId的一对一关联；完全相同的重复绑定视为成功。
    bool bindInstance(InstanceId instanceId, RenderItemId itemId);
    // 解除InstanceId对应的RenderItem映射，不删除RenderItem。
    bool unbindInstance(InstanceId instanceId);

    bool containsInstance(InstanceId instanceId) const;
    bool containsItem(RenderItemId itemId) const;

    RenderItemId itemId(InstanceId instanceId) const;
    InstanceId instanceId(RenderItemId itemId) const;

    std::size_t instanceCount() const;

    /// Topology <-> Geometry Resource

    // 建立Topology与Geometry Resource的一对一关联。
    // Topology_Orientation不参与资源身份，同一Topology的不同使用方向共享同一Geometry Resource。
    bool bindTopology(const Topology_Object& topology, BRepTopologyResourceType resourceType, ResourceId resourceId);
    // 解除Topology对应的Geometry Resource映射，不删除Resource。
    bool unbindTopology(TopologyId topologyId);

    bool containsTopology(TopologyId topologyId) const;
    bool containsResource(ResourceId resourceId) const;

    BRepTopologyBinding binding(TopologyId topologyId) const;
    const BRepTopologyBinding* bindingPointer(TopologyId topologyId) const;

    ResourceId resourceId(TopologyId topologyId) const;
    BRepTopologyResourceType resourceType(TopologyId topologyId) const;
    TopologyId topologyId(ResourceId resourceId) const;

    std::vector<TopologyId> topologyIds() const;
    std::size_t topologyCount() const;

    /// 整体状态

    // 只清空身份映射，不删除或修改任何MyOpenGL对象。
    void clear();

private:
    typedef std::map<InstanceId, RenderItemId> ItemByInstanceMap;
    typedef std::map<RenderItemId, InstanceId> InstanceByItemMap;

    typedef std::map<TopologyId, BRepTopologyBinding> BindingByTopologyMap;
    typedef std::map<ResourceId, TopologyId> TopologyByResourceMap;

private:
    ItemByInstanceMap m_itemByInstance;
    InstanceByItemMap m_instanceByItem;

    BindingByTopologyMap m_bindingByTopology;
    TopologyByResourceMap m_topologyByResource;
};

}
}

#endif // MYBREPOPENGL_DISPLAY_BREPDISPLAYMANAGER_H