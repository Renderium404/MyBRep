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
// 该结构不拥有任何对象，真实生命周期统一由BRepViewerWidget管理。
struct BRepDisplayObject
{
    BRepDisplayObject();

    bool isValid() const;
    void clear();

    BRepDisplayId id;       // BRepViewerWidget分配的显示对象ID。
    RenderItemId itemId;    // ItemManager拥有的RenderItem。
    ResourceId geometryId;  // ResourceManager拥有的BufferGeometry。
    MaterialId materialId;  // MaterialManager拥有的Material。
};

}
}

#endif // MYBREPOPENGL_DISPLAY_BREPDISPLAYOBJECT_H
