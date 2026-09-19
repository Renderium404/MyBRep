#ifndef MYBREPOPENGL_DISPLAY_BREPSOLIDGEOMETRYRESOURCE_H
#define MYBREPOPENGL_DISPLAY_BREPSOLIDGEOMETRYRESOURCE_H

#include "MyBRep/Topology/Solid/Topology_Solid.h"
#include "MyOpenGL/Core/Resource.h"

namespace MyBRep
{
namespace Display
{

/// BRepViewerWidget内部共享Solid几何资源唯一标识类型。
typedef unsigned int BRepSolidGeometryResourceId;

/// 无效共享Solid几何资源ID。
const BRepSolidGeometryResourceId InvalidBRepSolidGeometryResourceId = 0;

// 记录一个Topology_Solid对应的共享MyOpenGL Geometry资源。
// 同一Viewer统一使用一份全局离散参数，因此资源匹配只比较Topology身份和使用方向。
class BRepSolidGeometryResource
{
public:
    // 构造空共享资源记录。
    BRepSolidGeometryResource();

    // 构造完整共享资源记录。
    BRepSolidGeometryResource(
        BRepSolidGeometryResourceId id,
        const Topology_Solid& topology,
        ResourceId surfaceGeometryId,
        ResourceId wireframeGeometryId);

    // 判断当前记录是否具有完整有效的拓扑身份和两份Geometry资源ID。
    bool isValid() const;

    // 判断指定Topology_Solid是否可以复用当前共享Geometry资源。
    bool matches(const Topology_Solid& topology) const;

    // 返回共享资源ID。
    BRepSolidGeometryResourceId id() const;
    // 返回资源对应的Topology_Solid。
    const Topology_Solid& topology() const;
    // 返回Surface BufferGeometry资源ID。
    ResourceId surfaceGeometryId() const;
    // 返回Boundary BufferGeometry资源ID。
    ResourceId wireframeGeometryId() const;

private:
    BRepSolidGeometryResourceId m_id;
    Topology_Solid m_topology;
    ResourceId m_surfaceGeometryId;
    ResourceId m_wireframeGeometryId;
};

}
}

#endif // MYBREPOPENGL_DISPLAY_BREPSOLIDGEOMETRYRESOURCE_H