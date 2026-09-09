#ifndef MYBREPOPENGL_DISPLAY_BREPDISPLAYOBJECT_H
#define MYBREPOPENGL_DISPLAY_BREPDISPLAYOBJECT_H

#include "MyOpenGL/Core/Resource.h"
#include "MyOpenGL/Item/RenderItem.h"
#include "MyOpenGL/Material/Material.h"

namespace MyBRep
{
namespace Display
{

/// BRepViewerWidget内部显示对象唯一标识类型。
typedef unsigned int BRepDisplayId;

/// 无效BRep显示对象ID。
const BRepDisplayId InvalidBRepDisplayId = 0;

// 记录一个BRepViewerWidget显示对象在MyOpenGL各Manager中的资源身份。
// 一个显示对象可以只包含Wireframe，也可以同时包含Surface和Wireframe。
// 该结构不拥有任何对象，真实生命周期统一由BRepViewerWidget管理。
struct BRepDisplayObject
{
    BRepDisplayObject();

    // 判断当前结构是否描述完整且一致的B-Rep显示对象。
    bool isValid() const;
    // 判断当前显示对象是否包含Face表面资源。
    bool hasSurface() const;
    // 判断当前显示对象是否包含B-Rep边界资源。
    bool hasWireframe() const;
    // 清空全部Manager身份。
    void clear();

    BRepDisplayId id;                    // BRepViewerWidget分配的显示对象ID。
    RenderItemId itemId;                 // ItemManager拥有的RenderItem。
    ResourceId surfaceGeometryId;        // ResourceManager拥有的Face表面BufferGeometry。
    ResourceId wireframeGeometryId;      // ResourceManager拥有的B-Rep边界BufferGeometry。
    MaterialId surfaceMaterialId;        // MaterialManager拥有的Face表面Material。
    MaterialId wireframeMaterialId;      // MaterialManager拥有的B-Rep边界Material。
};

}
}

#endif // MYBREPOPENGL_DISPLAY_BREPDISPLAYOBJECT_H
