#include "BRepDisplayObject.h"

namespace MyBRep
{
namespace Display
{

BRepDisplayObject::BRepDisplayObject()
    : id(InvalidBRepDisplayId)
    , itemId(InvalidRenderItemId)
    , geometryId(InvalidResourceId)
    , materialId(InvalidMaterialId)
{
}

bool BRepDisplayObject::isValid() const
{
    return id != InvalidBRepDisplayId && itemId != InvalidRenderItemId && geometryId != InvalidResourceId && materialId != InvalidMaterialId;
}

void BRepDisplayObject::clear()
{
    id = InvalidBRepDisplayId;
    itemId = InvalidRenderItemId;
    geometryId = InvalidResourceId;
    materialId = InvalidMaterialId;
}

}
}
