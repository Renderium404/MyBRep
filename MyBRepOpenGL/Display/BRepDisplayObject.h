#ifndef MYBREPOPENGL_DISPLAY_BREPDISPLAYOBJECT_H
#define MYBREPOPENGL_DISPLAY_BREPDISPLAYOBJECT_H

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

// 描述一个B-Rep Instance在BRepViewerWidget中的显示对象。
// Geometry Resource由BRepDisplayManager按Topology身份统一管理，本结构只记录当前Instance显示自身拥有的对象。
struct BRepDisplayObject
{
    BRepDisplayObject();

    bool isValid() const;
    bool hasSurface() const;
    bool hasWireframe() const;
    void clear();

    BRepDisplayId id;               // BRepViewerWidget分配的显示对象ID。
    RenderItemId itemId;            // 当前Instance对应的RenderItem。
    MaterialId surfaceMaterialId;   // 当前Instance全部Face Part使用的表面Material。
    MaterialId wireframeMaterialId; // 当前Instance全部Edge Part使用的线框Material。
};

}
}

#endif // MYBREPOPENGL_DISPLAY_BREPDISPLAYOBJECT_H