#ifndef MYBREPOPENGL_DISPLAY_BREPDISPLAYMANAGER_H
#define MYBREPOPENGL_DISPLAY_BREPDISPLAYMANAGER_H

#include <cstddef>
#include <map>

#include "MyBRep/Instance/Instance_Object.h"
#include "MyBRep/Topology/Topology_Id.h"
#include "MyOpenGL/Core/Resource.h"
#include "MyOpenGL/Item/RenderItem.h"
#include "MyOpenGL/Item/RenderPart.h"

namespace MyBRep
{
namespace Display
{

// B-Rep拓扑在显示层对应的资源类型。
// 类型只描述显示资源语义，不参与TopologyId身份。
enum class BRepTopologyResourceType
{
    Unknown,
    Face,
    Curve
};

// 一个Topology实体在MyOpenGL中的稳定显示绑定。
// TopologyId、ResourceId和RenderPartId保持一对一强关联，RenderItem只负责组织这里登记的全局RenderPart。
struct BRepPartBinding
{
    BRepPartBinding();

    bool isValid() const;

    TopologyId topologyId;
    ResourceId resourceId;
    RenderPartId partId;
    BRepTopologyResourceType resourceType;
};

// 管理MyBRep与MyOpenGL之间的稳定身份映射。
//
// 本类只管理映射，不拥有RenderItem、RenderPart或Resource，也不重复记录RenderItem当前组织了哪些Part。
// RenderItem本身是Part组织关系的唯一事实来源；ItemManager和ResourceManager继续负责真实对象生命周期。
class BRepDisplayManager
{
public:
    BRepDisplayManager();

    /// Instance <-> RenderItem

    // 建立InstanceId与RenderItemId的一对一强关联；完全相同的重复绑定视为成功。
    bool bindInstance(InstanceId instanceId, RenderItemId itemId);
    // 解除InstanceId与RenderItemId映射，不删除RenderItem，也不修改RenderItem当前组织的Part。
    bool unbindInstance(InstanceId instanceId);

    bool containsInstance(InstanceId instanceId) const;
    bool containsItem(RenderItemId itemId) const;
    RenderItemId itemId(InstanceId instanceId) const;
    InstanceId instanceId(RenderItemId itemId) const;
    std::size_t instanceCount() const;

    /// Topology <-> Resource / RenderPart

    // 原子建立TopologyId、ResourceId和RenderPartId的一对一强关联。
    // Topology_Orientation不参与该绑定；同一Topology无论使用方向如何都对应同一Resource和同一RenderPart。
    bool bindTopology(TopologyId topologyId, BRepTopologyResourceType resourceType, ResourceId resourceId, RenderPartId partId);
    // 解除指定Topology的显示绑定，不删除Resource或RenderPart，也不修改任何RenderItem的Part组织关系。
    bool unbindTopology(TopologyId topologyId);

    bool containsTopology(TopologyId topologyId) const;
    bool containsResource(ResourceId resourceId) const;
    bool containsPart(RenderPartId partId) const;

    BRepPartBinding binding(TopologyId topologyId) const;
    ResourceId resourceId(TopologyId topologyId) const;
    RenderPartId partId(TopologyId topologyId) const;
    TopologyId topologyIdByResource(ResourceId resourceId) const;
    TopologyId topologyIdByPart(RenderPartId partId) const;
    std::size_t topologyCount() const;

    /// 整体状态

    // 只清空映射，不删除或修改任何MyOpenGL对象。
    void clear();

private:
    typedef std::map<InstanceId, RenderItemId> ItemByInstanceMap;
    typedef std::map<RenderItemId, InstanceId> InstanceByItemMap;

    typedef std::map<TopologyId, BRepPartBinding> BindingByTopologyMap;
    typedef std::map<ResourceId, TopologyId> TopologyByResourceMap;
    typedef std::map<RenderPartId, TopologyId> TopologyByPartMap;

private:
    ItemByInstanceMap m_itemByInstance;
    InstanceByItemMap m_instanceByItem;

    BindingByTopologyMap m_bindingByTopology;
    TopologyByResourceMap m_topologyByResource;
    TopologyByPartMap m_topologyByPart;
};

}
}

#endif // MYBREPOPENGL_DISPLAY_BREPDISPLAYMANAGER_H
