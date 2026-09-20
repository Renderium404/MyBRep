#include "BRepDisplayObject.h"

namespace MyBRep
{
namespace Display
{

BRepDisplayObject::BRepDisplayObject()
    : id(InvalidBRepDisplayId)
    , itemId(InvalidRenderItemId)
    , surfaceMaterialId(InvalidMaterialId)
    , wireframeMaterialId(InvalidMaterialId)
{
}

bool BRepDisplayObject::isValid() const
{
    if (id == InvalidBRepDisplayId || itemId == InvalidRenderItemId) return false;
    return hasSurface() || hasWireframe();
}

bool BRepDisplayObject::hasSurface() const
{
    return surfaceMaterialId != InvalidMaterialId;
}

bool BRepDisplayObject::hasWireframe() const
{
    return wireframeMaterialId != InvalidMaterialId;
}

void BRepDisplayObject::clear()
{
    id = InvalidBRepDisplayId;
    itemId = InvalidRenderItemId;
    surfaceMaterialId = InvalidMaterialId;
    wireframeMaterialId = InvalidMaterialId;
}

}
}