#include "BRepViewerWidget.h"

#include <QDebug>
#include <QOpenGLContext>
#include <QVector3D>

#include "MyBRepOpenGL/Builder/BRepWireframeBuilder.h"
#include "MyOpenGL/Item/AxisAlignedBoundingBox.h"
#include "MyOpenGL/Item/RenderPart.h"
#include "MyOpenGL/Material/Material.h"
#include "MyOpenGL/Render/MyOpenGLContext.h"
#include "MyOpenGL/Resource/BufferGeometry.h"

namespace
{

AxisAlignedBoundingBox geometryBounds(const BufferGeometry& geometry)
{
    AxisAlignedBoundingBox bounds;
    const std::vector<GLfloat>& vertices = geometry.vertexData();

    for (std::size_t index = 0; index + 2 < vertices.size(); index += 3)
    {
        bounds.expandToInclude(QVector3D(vertices[index], vertices[index + 1], vertices[index + 2]));
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

BRepDisplayId BRepViewerWidget::addWireframe(const Topology_Edge& edge, const MyMath::Matrix4& localToWorld, const QString& name, const BRepDisplayStyle& style)
{
    if (!edge.isValid() || !style.isValid())
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

BRepDisplayId BRepViewerWidget::addWireframe(const Topology_Wire& wire, const MyMath::Matrix4& localToWorld, const QString& name, const BRepDisplayStyle& style)
{
    if (!wire.isValid() || !style.isValid())
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

BRepDisplayId BRepViewerWidget::addWireframe(const Topology_Face& face, const MyMath::Matrix4& localToWorld, const QString& name, const BRepDisplayStyle& style)
{
    if (!face.isValid() || !style.isValid())
    {
        return InvalidBRepDisplayId;
    }

    return attachWireframe(BRepWireframeBuilder::build(face, localToWorld, name + "_WireframeGeometry", style.wireframe), name, style);
}

/// Topology Shell

BRepDisplayId BRepViewerWidget::addWireframe(const Topology_Shell& shell, const QString& name, const BRepDisplayStyle& style)
{
    return addWireframe(shell, MyMath::Matrix4::identity(), name, style);
}

BRepDisplayId BRepViewerWidget::addWireframe(const Topology_Shell& shell, const MyMath::Matrix4& localToWorld, const QString& name, const BRepDisplayStyle& style)
{
    if (!shell.isValid() || !style.isValid())
    {
        return InvalidBRepDisplayId;
    }

    return attachWireframe(BRepWireframeBuilder::build(shell, localToWorld, name + "_WireframeGeometry", style.wireframe), name, style);
}

/// Topology Solid

BRepDisplayId BRepViewerWidget::addWireframe(const Topology_Solid& solid, const QString& name, const BRepDisplayStyle& style)
{
    return addWireframe(solid, MyMath::Matrix4::identity(), name, style);
}

BRepDisplayId BRepViewerWidget::addWireframe(const Topology_Solid& solid, const MyMath::Matrix4& localToWorld, const QString& name, const BRepDisplayStyle& style)
{
    if (!solid.isValid() || !style.isValid())
    {
        return InvalidBRepDisplayId;
    }

    return attachWireframe(BRepWireframeBuilder::build(solid, localToWorld, name + "_WireframeGeometry", style.wireframe), name, style);
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
    if (geometry == 0 || !style.isValid())
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

    item->setMaterial(material);

    RenderPart* part = item->createPart();

    if (part == 0)
    {
        itemManager().remove(item->id());
        materialManager().remove(material->id());
        resourceManager().remove(geometryId);
        return InvalidBRepDisplayId;
    }

    part->setGeometry(geometry);
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
    object.geometryId = geometryId;
    object.materialId = material->id();

    m_displays[displayId] = object;
    update();

    return displayId;
}

/// 内部释放

bool BRepViewerWidget::removeDisplayResources(BRepDisplayObject& object)
{
    Resource* geometryResource = object.geometryId != InvalidResourceId ? resourceManager().get(object.geometryId) : 0;
    MyOpenGLContext cleanupContext;
    QOpenGLFunctions_3_3_Core* gl = 0;
    bool contextCurrent = false;

    // 只有已经建立GPU状态的Resource才要求当前OpenGL Context；纯CPU阶段可以直接从ResourceManager删除。
    if (geometryResource != 0 && geometryResource->isInitialized())
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

    bool result = true;

    // Item/Part只借用Geometry和Material，因此必须先解除Item引用，再释放Material和Resource。
    if (object.itemId != InvalidRenderItemId && itemManager().contains(object.itemId))
    {
        if (!itemManager().remove(object.itemId))
        {
            result = false;
        }
    }

    if (object.materialId != InvalidMaterialId && materialManager().contains(object.materialId))
    {
        if (!materialManager().remove(object.materialId))
        {
            result = false;
        }
    }

    if (object.geometryId != InvalidResourceId && resourceManager().contains(object.geometryId))
    {
        if (!resourceManager().remove(object.geometryId, gl))
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
