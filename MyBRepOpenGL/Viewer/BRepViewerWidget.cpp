#include "BRepViewerWidget.h"

#include <QDebug>
#include <QOpenGLContext>
#include <QVector3D>

#include "MyBRepOpenGL/Builder/BRepFaceBuilder.h"
#include "MyBRepOpenGL/Builder/BRepShellBuilder.h"
#include "MyBRepOpenGL/Builder/BRepSolidBuilder.h"
#include "MyBRepOpenGL/Builder/BRepWireframeBuilder.h"
#include "MyOpenGL/Item/AxisAlignedBoundingBox.h"
#include "MyOpenGL/Item/RenderPart.h"
#include "MyOpenGL/Material/Material.h"
#include "MyOpenGL/Render/MyOpenGLContext.h"
#include "MyOpenGL/Resource/BufferGeometry.h"

namespace
{

int positionValueOffset(const BufferGeometry& geometry)
{
    const std::vector<GeometryVertexAttribute>& attributes = geometry.attributes();

    for (std::size_t index = 0; index < attributes.size(); ++index)
    {
        if (attributes[index].location == GeometryAttribute::Position && attributes[index].componentCount >= 3)
        {
            return attributes[index].valueOffset;
        }
    }

    return -1;
}

AxisAlignedBoundingBox geometryBounds(const BufferGeometry& geometry)
{
    AxisAlignedBoundingBox bounds;
    const int valuesPerVertex = geometry.valuesPerVertex();
    const int positionOffset = positionValueOffset(geometry);
    const std::vector<GLfloat>& vertices = geometry.vertexData();

    if (valuesPerVertex <= 0 || positionOffset < 0 || positionOffset + 2 >= valuesPerVertex)
    {
        return bounds;
    }

    for (std::size_t vertexOffset = 0; vertexOffset + static_cast<std::size_t>(valuesPerVertex) <= vertices.size();
         vertexOffset += static_cast<std::size_t>(valuesPerVertex))
    {
        const std::size_t positionIndex = vertexOffset + static_cast<std::size_t>(positionOffset);
        bounds.expandToInclude(QVector3D(vertices[positionIndex], vertices[positionIndex + 1], vertices[positionIndex + 2]));
    }

    return bounds;
}

}

namespace MyBRep
{
namespace Display
{

BRepViewerWidget::BRepViewerWidget(QWidget* parent)
    : OpenGLViewerWidget(parent)
    , m_nextDisplayId(1)
{
}

BRepViewerWidget::~BRepViewerWidget()
{
    if (!clearBRepDisplays())
    {
        qWarning() << "BRepViewerWidget destruction: some B-Rep display resources could not be removed before base Viewer teardown.";
    }
}

/// Topology Edge

BRepDisplayId BRepViewerWidget::addWireframe(const Topology_Edge& edge, const QString& name, const BRepDisplayStyle& style)
{
    return addWireframe(edge, MyMath::Matrix4::identity(), name, style);
}

BRepDisplayId BRepViewerWidget::addWireframe(const Topology_Edge& edge, const MyMath::Matrix4& localToWorld, const QString& name,
                                             const BRepDisplayStyle& style)
{
    if (!edge.isValid() || !style.isWireframeValid())
    {
        return InvalidBRepDisplayId;
    }

    return attachWireframe(BRepWireframeBuilder::build(edge, localToWorld, name + "_WireframeGeometry", style.wireframe), name, style);
}

/// Topology Wire

BRepDisplayId BRepViewerWidget::addWireframe(const Topology_Wire& wire, const QString& name, const BRepDisplayStyle& style)
{
    return addWireframe(wire, MyMath::Matrix4::identity(), name, style);
}

BRepDisplayId BRepViewerWidget::addWireframe(const Topology_Wire& wire, const MyMath::Matrix4& localToWorld, const QString& name,
                                             const BRepDisplayStyle& style)
{
    if (!wire.isValid() || !style.isWireframeValid())
    {
        return InvalidBRepDisplayId;
    }

    return attachWireframe(BRepWireframeBuilder::build(wire, localToWorld, name + "_WireframeGeometry", style.wireframe), name, style);
}

/// Topology Face

BRepDisplayId BRepViewerWidget::addWireframe(const Topology_Face& face, const QString& name, const BRepDisplayStyle& style)
{
    return addWireframe(face, MyMath::Matrix4::identity(), name, style);
}

BRepDisplayId BRepViewerWidget::addWireframe(const Topology_Face& face, const MyMath::Matrix4& localToWorld, const QString& name,
                                             const BRepDisplayStyle& style)
{
    if (!face.isValid() || !style.isWireframeValid())
    {
        return InvalidBRepDisplayId;
    }

    return attachWireframe(BRepWireframeBuilder::build(face, localToWorld, name + "_WireframeGeometry", style.wireframe), name, style);
}

BRepDisplayId BRepViewerWidget::addFace(const Topology_Face& face, const QString& name, const BRepDisplayStyle& style)
{
    return addFace(face, MyMath::Matrix4::identity(), name, style);
}

BRepDisplayId BRepViewerWidget::addFace(const Topology_Face& face, const MyMath::Matrix4& localToWorld, const QString& name,
                                        const BRepDisplayStyle& style)
{
    if (!face.isValid() || !style.isFaceValid())
    {
        return InvalidBRepDisplayId;
    }

    BufferGeometry* surfaceGeometry =
        BRepFaceBuilder::build(face, localToWorld, name + "_SurfaceGeometry", style.surface);

    BufferGeometry* wireframeGeometry =
        BRepWireframeBuilder::build(face, localToWorld, name + "_WireframeGeometry", style.wireframe);

    if (surfaceGeometry == 0 || wireframeGeometry == 0)
    {
        delete surfaceGeometry;
        delete wireframeGeometry;
        return InvalidBRepDisplayId;
    }

    return attachSurfaceDisplay(surfaceGeometry, wireframeGeometry, name, style);
}

/// Topology Shell

BRepDisplayId BRepViewerWidget::addWireframe(const Topology_Shell& shell, const QString& name, const BRepDisplayStyle& style)
{
    return addWireframe(shell, MyMath::Matrix4::identity(), name, style);
}

BRepDisplayId BRepViewerWidget::addWireframe(const Topology_Shell& shell, const MyMath::Matrix4& localToWorld, const QString& name,
                                             const BRepDisplayStyle& style)
{
    if (!shell.isValid() || !style.isWireframeValid())
    {
        return InvalidBRepDisplayId;
    }

    return attachWireframe(BRepWireframeBuilder::build(shell, localToWorld, name + "_WireframeGeometry", style.wireframe), name, style);
}

BRepDisplayId BRepViewerWidget::addShell(const Topology_Shell& shell, const QString& name, const BRepDisplayStyle& style)
{
    return addShell(shell, MyMath::Matrix4::identity(), name, style);
}

BRepDisplayId BRepViewerWidget::addShell(const Topology_Shell& shell, const MyMath::Matrix4& localToWorld, const QString& name,
                                         const BRepDisplayStyle& style)
{
    if (!shell.isValid() || !style.isValid())
    {
        return InvalidBRepDisplayId;
    }

    BRepShellBuildOptions options;
    options.surface = style.surface;
    options.wireframe = style.wireframe;

    BufferGeometry* surfaceGeometry =
        BRepShellBuilder::buildSurface(shell, localToWorld, name + "_SurfaceGeometry", options);

    BufferGeometry* wireframeGeometry =
        BRepShellBuilder::buildBoundary(shell, localToWorld, name + "_WireframeGeometry", options);

    if (surfaceGeometry == 0 || wireframeGeometry == 0)
    {
        delete surfaceGeometry;
        delete wireframeGeometry;
        return InvalidBRepDisplayId;
    }

    return attachSurfaceDisplay(surfaceGeometry, wireframeGeometry, name, style);
}

/// Topology Solid

BRepDisplayId BRepViewerWidget::addWireframe(const Topology_Solid& solid, const QString& name, const BRepDisplayStyle& style)
{
    return addWireframe(solid, MyMath::Matrix4::identity(), name, style);
}

BRepDisplayId BRepViewerWidget::addWireframe(const Topology_Solid& solid, const MyMath::Matrix4& localToWorld, const QString& name,
                                             const BRepDisplayStyle& style)
{
    if (!solid.isValid() || !style.isWireframeValid())
    {
        return InvalidBRepDisplayId;
    }

    return attachWireframe(BRepWireframeBuilder::build(solid, localToWorld, name + "_WireframeGeometry", style.wireframe), name, style);
}

BRepDisplayId BRepViewerWidget::addSolid(const Topology_Solid& solid, const QString& name, const BRepDisplayStyle& style)
{
    return addSolid(solid, MyMath::Matrix4::identity(), name, style);
}

BRepDisplayId BRepViewerWidget::addSolid(const Topology_Solid& solid, const MyMath::Matrix4& localToWorld, const QString& name,
                                         const BRepDisplayStyle& style)
{
    if (!solid.isValid() || !style.isValid())
    {
        return InvalidBRepDisplayId;
    }

    BRepSolidBuildOptions options;
    options.surface = style.surface;
    options.wireframe = style.wireframe;

    BufferGeometry* surfaceGeometry =
        BRepSolidBuilder::buildSurface(solid, localToWorld, name + "_SurfaceGeometry", options);

    BufferGeometry* wireframeGeometry =
        BRepSolidBuilder::buildBoundary(solid, localToWorld, name + "_WireframeGeometry", options);

    if (surfaceGeometry == 0 || wireframeGeometry == 0)
    {
        delete surfaceGeometry;
        delete wireframeGeometry;
        return InvalidBRepDisplayId;
    }

    return attachSurfaceDisplay(surfaceGeometry, wireframeGeometry, name, style);
}

/// Instance

BRepDisplayId BRepViewerWidget::addWireframe(const Edge& edge, const QString& name, const BRepDisplayStyle& style)
{
    return edge.isValid() ? addWireframe(edge.topology(), edge.localToWorld(), name, style) : InvalidBRepDisplayId;
}

BRepDisplayId BRepViewerWidget::addWireframe(const Wire& wire, const QString& name, const BRepDisplayStyle& style)
{
    return wire.isValid() ? addWireframe(wire.topology(), wire.localToWorld(), name, style) : InvalidBRepDisplayId;
}

BRepDisplayId BRepViewerWidget::addWireframe(const Face& face, const QString& name, const BRepDisplayStyle& style)
{
    return face.isValid() ? addWireframe(face.topology(), face.localToWorld(), name, style) : InvalidBRepDisplayId;
}

BRepDisplayId BRepViewerWidget::addWireframe(const Shell& shell, const QString& name, const BRepDisplayStyle& style)
{
    return shell.isValid() ? addWireframe(shell.topology(), shell.localToWorld(), name, style) : InvalidBRepDisplayId;
}

BRepDisplayId BRepViewerWidget::addWireframe(const Solid& solid, const QString& name, const BRepDisplayStyle& style)
{
    return solid.isValid() ? addWireframe(solid.topology(), solid.localToWorld(), name, style) : InvalidBRepDisplayId;
}

BRepDisplayId BRepViewerWidget::addFace(const Face& face, const QString& name, const BRepDisplayStyle& style)
{
    return face.isValid() ? addFace(face.topology(), face.localToWorld(), name, style) : InvalidBRepDisplayId;
}

BRepDisplayId BRepViewerWidget::addShell(const Shell& shell, const QString& name, const BRepDisplayStyle& style)
{
    return shell.isValid() ? addShell(shell.topology(), shell.localToWorld(), name, style) : InvalidBRepDisplayId;
}

BRepDisplayId BRepViewerWidget::addSolid(const Solid& solid, const QString& name, const BRepDisplayStyle& style)
{
    return solid.isValid() ? addSolid(solid.topology(), solid.localToWorld(), name, style) : InvalidBRepDisplayId;
}

/// 显示对象管理

bool BRepViewerWidget::containsDisplay(BRepDisplayId id) const
{
    return m_displays.find(id) != m_displays.end();
}

std::size_t BRepViewerWidget::displayCount() const
{
    return m_displays.size();
}

BRepDisplayObject BRepViewerWidget::display(BRepDisplayId id) const
{
    DisplayMap::const_iterator iterator = m_displays.find(id);
    return iterator != m_displays.end() ? iterator->second : BRepDisplayObject();
}

bool BRepViewerWidget::removeDisplay(BRepDisplayId id)
{
    DisplayMap::iterator iterator = m_displays.find(id);

    if (iterator == m_displays.end())
    {
        return false;
    }

    if (!removeDisplayResources(iterator->second))
    {
        return false;
    }

    m_displays.erase(iterator);

    if (m_displays.empty())
    {
        m_nextDisplayId = 1;
    }

    update();
    return true;
}

bool BRepViewerWidget::clearBRepDisplays()
{
    bool result = true;
    DisplayMap::iterator iterator = m_displays.begin();

    while (iterator != m_displays.end())
    {
        DisplayMap::iterator current = iterator;
        ++iterator;

        if (removeDisplayResources(current->second))
        {
            m_displays.erase(current);
        }
        else
        {
            result = false;
        }
    }

    if (m_displays.empty())
    {
        m_nextDisplayId = 1;
    }

    update();
    return result;
}

/// 内部注册

BRepDisplayId BRepViewerWidget::attachWireframe(BufferGeometry* geometry, const QString& name, const BRepDisplayStyle& style)
{
    if (geometry == 0 || !style.isWireframeValid())
    {
        delete geometry;
        return InvalidBRepDisplayId;
    }

    const AxisAlignedBoundingBox bounds = geometryBounds(*geometry);

    if (!bounds.isValid())
    {
        delete geometry;
        return InvalidBRepDisplayId;
    }

    const ResourceId geometryId = resourceManager().adopt(geometry);

    if (geometryId == InvalidResourceId)
    {
        delete geometry;
        return InvalidBRepDisplayId;
    }

    Material* material = materialManager().createMaterial(name + "_WireframeMaterial");

    if (material == 0)
    {
        resourceManager().remove(geometryId);
        return InvalidBRepDisplayId;
    }

    if (!material->setSurfaceMode(SurfaceMode::Color) || !material->setColor(style.wireColor))
    {
        materialManager().remove(material->id());
        resourceManager().remove(geometryId);
        return InvalidBRepDisplayId;
    }

    // 线框Geometry只包含Position，不包含Normal，因此明确关闭光照并走MyOpenGL统一颜色Pipeline。
    material->setLightingEnabled(false);

    RenderItem* item = itemManager().createItem(name);

    if (item == 0)
    {
        materialManager().remove(material->id());
        resourceManager().remove(geometryId);
        return InvalidBRepDisplayId;
    }

    RenderPart* part = item->createPart();

    if (part == 0)
    {
        itemManager().remove(item->id());
        materialManager().remove(material->id());
        resourceManager().remove(geometryId);
        return InvalidBRepDisplayId;
    }

    part->setGeometry(geometry);
    part->setMaterial(material);
    part->setLocalBounds(bounds);

    const BRepDisplayId displayId = allocateDisplayId();

    if (displayId == InvalidBRepDisplayId)
    {
        itemManager().remove(item->id());
        materialManager().remove(material->id());
        resourceManager().remove(geometryId);
        return InvalidBRepDisplayId;
    }

    BRepDisplayObject object;
    object.id = displayId;
    object.itemId = item->id();
    object.wireframeGeometryId = geometryId;
    object.wireframeMaterialId = material->id();

    m_displays[displayId] = object;
    update();

    return displayId;
}

BRepDisplayId BRepViewerWidget::attachSurfaceDisplay(BufferGeometry* surfaceGeometry, BufferGeometry* wireframeGeometry, const QString& name,
                                                     const BRepDisplayStyle& style)
{
    if (surfaceGeometry == 0 || wireframeGeometry == 0 || !style.isValid())
    {
        delete surfaceGeometry;
        delete wireframeGeometry;
        return InvalidBRepDisplayId;
    }

    const AxisAlignedBoundingBox surfaceBounds = geometryBounds(*surfaceGeometry);
    const AxisAlignedBoundingBox wireframeBounds = geometryBounds(*wireframeGeometry);

    if (!surfaceBounds.isValid() || !wireframeBounds.isValid())
    {
        delete surfaceGeometry;
        delete wireframeGeometry;
        return InvalidBRepDisplayId;
    }

    const ResourceId surfaceGeometryId = resourceManager().adopt(surfaceGeometry);

    if (surfaceGeometryId == InvalidResourceId)
    {
        delete surfaceGeometry;
        delete wireframeGeometry;
        return InvalidBRepDisplayId;
    }

    const ResourceId wireframeGeometryId = resourceManager().adopt(wireframeGeometry);

    if (wireframeGeometryId == InvalidResourceId)
    {
        delete wireframeGeometry;
        resourceManager().remove(surfaceGeometryId);
        return InvalidBRepDisplayId;
    }

    Material* surfaceMaterial = materialManager().createMaterial(name + "_SurfaceMaterial");

    if (surfaceMaterial == 0)
    {
        resourceManager().remove(wireframeGeometryId);
        resourceManager().remove(surfaceGeometryId);
        return InvalidBRepDisplayId;
    }

    Material* wireframeMaterial = materialManager().createMaterial(name + "_WireframeMaterial");

    if (wireframeMaterial == 0)
    {
        materialManager().remove(surfaceMaterial->id());
        resourceManager().remove(wireframeGeometryId);
        resourceManager().remove(surfaceGeometryId);
        return InvalidBRepDisplayId;
    }

    if (!surfaceMaterial->setSurfaceMode(SurfaceMode::Color) ||
        !surfaceMaterial->setColor(style.surfaceColor) ||
        !wireframeMaterial->setSurfaceMode(SurfaceMode::Color) ||
        !wireframeMaterial->setColor(style.wireColor))
    {
        materialManager().remove(wireframeMaterial->id());
        materialManager().remove(surfaceMaterial->id());
        resourceManager().remove(wireframeGeometryId);
        resourceManager().remove(surfaceGeometryId);
        return InvalidBRepDisplayId;
    }

    surfaceMaterial->setLightingEnabled(style.surfaceLightingEnabled);
    wireframeMaterial->setLightingEnabled(false);

    RenderItem* item = itemManager().createItem(name);

    if (item == 0)
    {
        materialManager().remove(wireframeMaterial->id());
        materialManager().remove(surfaceMaterial->id());
        resourceManager().remove(wireframeGeometryId);
        resourceManager().remove(surfaceGeometryId);
        return InvalidBRepDisplayId;
    }

    // 边界Part必须先于Surface Part创建。
    // MyOpenGL普通Geometry使用GL_LESS；边界先写入共面深度后，后绘Surface在边界像素处因深度相等而不会覆盖边线，
    // 同时被其他更近Geometry遮挡的边界仍然正常通过Depth Test隐藏。
    RenderPart* wireframePart = item->createPart();

    if (wireframePart == 0)
    {
        itemManager().remove(item->id());
        materialManager().remove(wireframeMaterial->id());
        materialManager().remove(surfaceMaterial->id());
        resourceManager().remove(wireframeGeometryId);
        resourceManager().remove(surfaceGeometryId);
        return InvalidBRepDisplayId;
    }

    wireframePart->setGeometry(wireframeGeometry);
    wireframePart->setMaterial(wireframeMaterial);
    wireframePart->setLocalBounds(wireframeBounds);

    RenderPart* surfacePart = item->createPart();

    if (surfacePart == 0)
    {
        itemManager().remove(item->id());
        materialManager().remove(wireframeMaterial->id());
        materialManager().remove(surfaceMaterial->id());
        resourceManager().remove(wireframeGeometryId);
        resourceManager().remove(surfaceGeometryId);
        return InvalidBRepDisplayId;
    }

    surfacePart->setGeometry(surfaceGeometry);
    surfacePart->setMaterial(surfaceMaterial);
    surfacePart->setLocalBounds(surfaceBounds);

    const BRepDisplayId displayId = allocateDisplayId();

    if (displayId == InvalidBRepDisplayId)
    {
        itemManager().remove(item->id());
        materialManager().remove(wireframeMaterial->id());
        materialManager().remove(surfaceMaterial->id());
        resourceManager().remove(wireframeGeometryId);
        resourceManager().remove(surfaceGeometryId);
        return InvalidBRepDisplayId;
    }

    BRepDisplayObject object;
    object.id = displayId;
    object.itemId = item->id();
    object.surfaceGeometryId = surfaceGeometryId;
    object.wireframeGeometryId = wireframeGeometryId;
    object.surfaceMaterialId = surfaceMaterial->id();
    object.wireframeMaterialId = wireframeMaterial->id();

    m_displays[displayId] = object;
    update();

    return displayId;
}

/// 内部释放

bool BRepViewerWidget::removeDisplayResources(BRepDisplayObject& object)
{
    Resource* surfaceResource =
        object.surfaceGeometryId != InvalidResourceId ? resourceManager().get(object.surfaceGeometryId) : 0;

    Resource* wireframeResource =
        object.wireframeGeometryId != InvalidResourceId ? resourceManager().get(object.wireframeGeometryId) : 0;

    const bool needsContext =
        (surfaceResource != 0 && surfaceResource->isInitialized()) ||
        (wireframeResource != 0 && wireframeResource->isInitialized());

    MyOpenGLContext cleanupContext;
    QOpenGLFunctions_3_3_Core* gl = 0;
    bool contextCurrent = false;

    if (needsContext)
    {
        if (context() == 0)
        {
            return false;
        }

        makeCurrent();

        if (QOpenGLContext::currentContext() != context())
        {
            doneCurrent();
            return false;
        }

        if (!cleanupContext.initialize())
        {
            doneCurrent();
            return false;
        }

        gl = cleanupContext.gl();

        if (gl == 0)
        {
            doneCurrent();
            return false;
        }

        contextCurrent = true;
    }

    // RenderItem/RenderPart只借用Geometry和Material，因此必须先解除Item引用。
    if (object.itemId != InvalidRenderItemId && itemManager().contains(object.itemId))
    {
        if (!itemManager().remove(object.itemId))
        {
            if (contextCurrent)
            {
                doneCurrent();
            }

            return false;
        }
    }

    object.itemId = InvalidRenderItemId;

    bool result = true;

    if (object.wireframeMaterialId != InvalidMaterialId && materialManager().contains(object.wireframeMaterialId))
    {
        if (materialManager().remove(object.wireframeMaterialId))
        {
            object.wireframeMaterialId = InvalidMaterialId;
        }
        else
        {
            result = false;
        }
    }

    if (object.surfaceMaterialId != InvalidMaterialId && materialManager().contains(object.surfaceMaterialId))
    {
        if (materialManager().remove(object.surfaceMaterialId))
        {
            object.surfaceMaterialId = InvalidMaterialId;
        }
        else
        {
            result = false;
        }
    }

    if (object.wireframeGeometryId != InvalidResourceId && resourceManager().contains(object.wireframeGeometryId))
    {
        if (resourceManager().remove(object.wireframeGeometryId, gl))
        {
            object.wireframeGeometryId = InvalidResourceId;
        }
        else
        {
            result = false;
        }
    }

    if (object.surfaceGeometryId != InvalidResourceId && resourceManager().contains(object.surfaceGeometryId))
    {
        if (resourceManager().remove(object.surfaceGeometryId, gl))
        {
            object.surfaceGeometryId = InvalidResourceId;
        }
        else
        {
            result = false;
        }
    }

    if (contextCurrent)
    {
        doneCurrent();
    }

    if (result)
    {
        object.clear();
    }

    return result;
}

/// ID分配

BRepDisplayId BRepViewerWidget::allocateDisplayId()
{
    while (m_nextDisplayId == InvalidBRepDisplayId || containsDisplay(m_nextDisplayId))
    {
        ++m_nextDisplayId;
    }

    const BRepDisplayId id = m_nextDisplayId;
    ++m_nextDisplayId;
    return id;
}

}
}
