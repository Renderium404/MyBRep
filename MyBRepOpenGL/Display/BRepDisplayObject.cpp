#include "BRepDisplayObject.h"

namespace MyBRep
{
namespace Display
{

BRepDisplayObject::BRepDisplayObject()
    : id(InvalidBRepDisplayId)
    , itemId(InvalidRenderItemId)
    , surfaceGeometryId(InvalidResourceId)
    , wireframeGeometryId(InvalidResourceId)
    , surfaceMaterialId(InvalidMaterialId)
    , wireframeMaterialId(InvalidMaterialId)
{
}

bool BRepDisplayObject::isValid() const
{
    if (id == InvalidBRepDisplayId || itemId == InvalidRenderItemId)
    {
        return false;
    }

    const bool surfaceEmpty =
        surfaceGeometryId == InvalidResourceId &&
        surfaceMaterialId == InvalidMaterialId;

    const bool surfaceComplete =
        surfaceGeometryId != InvalidResourceId &&
        surfaceMaterialId != InvalidMaterialId;

    const bool wireframeEmpty =
        wireframeGeometryId == InvalidResourceId &&
        wireframeMaterialId == InvalidMaterialId;

    const bool wireframeComplete =
        wireframeGeometryId != InvalidResourceId &&
        wireframeMaterialId != InvalidMaterialId;

    return (surfaceEmpty || surfaceComplete) &&
           (wireframeEmpty || wireframeComplete) &&
           (surfaceComplete || wireframeComplete);
}

bool BRepDisplayObject::hasSurface() const
{
    return surfaceGeometryId != InvalidResourceId &&
           surfaceMaterialId != InvalidMaterialId;
}

bool BRepDisplayObject::hasWireframe() const
{
    return wireframeGeometryId != InvalidResourceId &&
           wireframeMaterialId != InvalidMaterialId;
}

void BRepDisplayObject::clear()
{
    id = InvalidBRepDisplayId;
    itemId = InvalidRenderItemId;
    surfaceGeometryId = InvalidResourceId;
    wireframeGeometryId = InvalidResourceId;
    surfaceMaterialId = InvalidMaterialId;
    wireframeMaterialId = InvalidMaterialId;
}

}
}
