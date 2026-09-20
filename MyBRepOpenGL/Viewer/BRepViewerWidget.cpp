#include "BRepViewerWidget.h"

#include <cmath>

#include <QDebug>
#include <QOpenGLContext>
#include <QQuaternion>
#include <QVector3D>

#include "MyMath/Matrix3.h"
#include "MyMath/Quaternion.h"
#include "MyMath/Vector3.h"

#include "MyOpenGL/Core/Resource.h"
#include "MyOpenGL/Item/AxisAlignedBoundingBox.h"
#include "MyOpenGL/Item/RenderPart.h"
#include "MyOpenGL/Light/Light.h"
#include "MyOpenGL/Material/Material.h"
#include "MyOpenGL/Render/MyOpenGLContext.h"
#include "MyOpenGL/Resource/BufferGeometry.h"
#include "MyOpenGL/Resource/Geometry.h"
namespace MyBRep
{
namespace Display
{

/// Geometry辅助

int BRepViewerWidget::positionValueOffset(const BufferGeometry& geometry)
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

AxisAlignedBoundingBox BRepViewerWidget::geometryBounds(const BufferGeometry& geometry)
{
    AxisAlignedBoundingBox bounds;

    const int valuesPerVertex = geometry.valuesPerVertex();
    const int positionOffset = positionValueOffset(geometry);
    const std::vector<GLfloat>& vertices = geometry.vertexData();

    if (valuesPerVertex <= 0 || positionOffset < 0 || positionOffset + 2 >= valuesPerVertex) return bounds;

    for (std::size_t vertexOffset = 0;
         vertexOffset + static_cast<std::size_t>(valuesPerVertex) <= vertices.size();
         vertexOffset += static_cast<std::size_t>(valuesPerVertex))
    {
        const std::size_t positionIndex = vertexOffset + static_cast<std::size_t>(positionOffset);
        bounds.expandToInclude(QVector3D(vertices[positionIndex], vertices[positionIndex + 1], vertices[positionIndex + 2]));
    }

    return bounds;
}

/// 默认灯光

bool BRepViewerWidget::createDefaultBRepLighting(LightManager& lightManager)
{
    Light* ambientLight = lightManager.createLight("BRepAmbientLight");
    if (ambientLight == 0) return false;

    ambientLight->setAmbient();

    if (!ambientLight->setColor(QVector3D(1.0f, 1.0f, 1.0f)) || !ambientLight->setIntensity(0.18f))
    {
        return false;
    }

    Light* keyLight = lightManager.createLight("BRepKeyLight");
    if (keyLight == 0) return false;

    if (!keyLight->setDirectional(QVector3D(-0.45f, -0.35f, -1.0f)) ||
        !keyLight->setColor(QVector3D(1.0f, 0.97f, 0.92f)) ||
        !keyLight->setIntensity(0.78f))
    {
        return false;
    }

    Light* fillLight = lightManager.createLight("BRepFillLight");
    if (fillLight == 0) return false;

    return fillLight->setDirectional(QVector3D(0.65f, -0.10f, -0.60f)) &&
           fillLight->setColor(QVector3D(0.72f, 0.84f, 1.0f)) &&
           fillLight->setIntensity(0.24f);
}

/// Instance Transform

bool BRepViewerWidget::decomposeItemTransform(const MyMath::Matrix4& localToWorld,
                                              QVector3D& position,
                                              QQuaternion& rotation,
                                              QVector3D& scale)
{
    if (!localToWorld.isAffine() || !localToWorld.isInvertible()) return false;

    MyMath::Vector3 axisX(localToWorld(0, 0), localToWorld(1, 0), localToWorld(2, 0));
    MyMath::Vector3 axisY(localToWorld(0, 1), localToWorld(1, 1), localToWorld(2, 1));
    MyMath::Vector3 axisZ(localToWorld(0, 2), localToWorld(1, 2), localToWorld(2, 2));

    double scaleX = axisX.length();
    double scaleY = axisY.length();
    double scaleZ = axisZ.length();

    if (scaleX <= 0.0 || scaleY <= 0.0 || scaleZ <= 0.0) return false;

    axisX /= scaleX;
    axisY /= scaleY;
    axisZ /= scaleZ;

    const double orthogonalTolerance = 1.0e-10;

    if (std::fabs(MyMath::Vector3::dot(axisX, axisY)) > orthogonalTolerance ||
        std::fabs(MyMath::Vector3::dot(axisY, axisZ)) > orthogonalTolerance ||
        std::fabs(MyMath::Vector3::dot(axisZ, axisX)) > orthogonalTolerance)
    {
        return false;
    }

    if (MyMath::Vector3::dot(MyMath::Vector3::cross(axisX, axisY), axisZ) < 0.0)
    {
        axisZ *= -1.0;
        scaleZ = -scaleZ;
    }

    const MyMath::Matrix3 rotationMatrix = MyMath::Matrix3::fromColumns(axisX, axisY, axisZ);

    if (!rotationMatrix.isRotationMatrix(orthogonalTolerance)) return false;

    const MyMath::Quaternion quaternion =
        MyMath::Quaternion::fromRotationMatrix(rotationMatrix, orthogonalTolerance);

    if (!quaternion.isUnit(orthogonalTolerance)) return false;

    const MyMath::Vector3 translation = localToWorld.translation();

    position = QVector3D(static_cast<float>(translation.x()),
                         static_cast<float>(translation.y()),
                         static_cast<float>(translation.z()));

    rotation = QQuaternion(static_cast<float>(quaternion.w()),
                           static_cast<float>(quaternion.x()),
                           static_cast<float>(quaternion.y()),
                           static_cast<float>(quaternion.z()));

    scale = QVector3D(static_cast<float>(scaleX),
                      static_cast<float>(scaleY),
                      static_cast<float>(scaleZ));

    return true;
}

/// 生命周期

BRepViewerWidget::BRepViewerWidget(QWidget* parent)
    : OpenGLViewerWidget(parent)
    , m_nextDisplayId(1)
{
    if (!createDefaultBRepLighting(lightManager()))
    {
        qWarning() << "BRepViewerWidget construction: unable to create complete default B-Rep lighting.";
    }
}

BRepViewerWidget::~BRepViewerWidget()
{
    if (!clearBRepDisplays())
    {
        qWarning() << "BRepViewerWidget destruction: some B-Rep displays could not be removed.";
    }

    if (!clearTopologyResources())
    {
        qWarning() << "BRepViewerWidget destruction: some topology geometry resources could not be removed.";
    }
}

/// Topology Geometry Resource

ResourceId BRepViewerWidget::acquireFaceResource(const Topology_Face& face, const QString& name)
{
    if (!face.isValid()) return InvalidResourceId;

    const TopologyId topologyId = face.id();
    const ResourceId existing = m_displayManager.resourceId(topologyId);

    if (existing != InvalidResourceId)
    {
        return m_displayManager.resourceType(topologyId) == BRepTopologyResourceType::Face
                   ? existing
                   : InvalidResourceId;
    }

    // Orientation不参与Geometry Resource身份，统一生成Forward Geometry。
    const Topology_Face source = face.isReversed() ? face.reversed() : face;
    const QString resourceName = name + "_Face_" + QString::number(static_cast<qulonglong>(topologyId));

    BufferGeometry* geometry = BRepFaceBuilder::build(source, resourceName, m_buildOptions.face);
    if (geometry == 0) return InvalidResourceId;

    const ResourceId resourceId = resourceManager().adopt(geometry);

    if (resourceId == InvalidResourceId)
    {
        delete geometry;
        return InvalidResourceId;
    }

    if (!m_displayManager.bindTopology(face, BRepTopologyResourceType::Face, resourceId))
    {
        resourceManager().remove(resourceId);
        return InvalidResourceId;
    }

    return resourceId;
}
ResourceId BRepViewerWidget::acquireEdgeResource(const Topology_Edge& edge, const QString& name)
{
    if (!edge.isValid()) return InvalidResourceId;

    const TopologyId topologyId = edge.id();
    const ResourceId existing = m_displayManager.resourceId(topologyId);

    if (existing != InvalidResourceId)
    {
        return m_displayManager.resourceType(topologyId) == BRepTopologyResourceType::Edge
                   ? existing
                   : InvalidResourceId;
    }

    const Topology_Edge source = edge.isReversed() ? edge.reversed() : edge;
    const QString resourceName = name + "_Edge_" + QString::number(static_cast<qulonglong>(topologyId));

    BufferGeometry* geometry = BRepEdgeBuilder::build(source, resourceName, m_buildOptions.edge);
    if (geometry == 0) return InvalidResourceId;

    const ResourceId resourceId = resourceManager().adopt(geometry);

    if (resourceId == InvalidResourceId)
    {
        delete geometry;
        return InvalidResourceId;
    }

    if (!m_displayManager.bindTopology(edge, BRepTopologyResourceType::Edge, resourceId))
    {
        resourceManager().remove(resourceId);
        return InvalidResourceId;
    }

    return resourceId;
}

/// RenderPart组织

bool BRepViewerWidget::attachFaceParts(RenderItem& item,
                                       const std::vector<Topology_Face>& faces,
                                       const Material* material,
                                       const QString& name)
{
    if (material == 0) return false;

    for (std::size_t index = 0; index < faces.size(); ++index)
    {
        const ResourceId resourceId = acquireFaceResource(faces[index], name);
        if (resourceId == InvalidResourceId) return false;

        const BufferGeometry* geometry = static_cast<const BufferGeometry*>(resourceManager().get(resourceId));
        if (geometry == 0) return false;

        const AxisAlignedBoundingBox bounds = geometryBounds(*geometry);
        if (!bounds.isValid()) return false;

        RenderPart* part = itemManager().createPart();
        if (part == 0) return false;

        part->setGeometry(geometry);
        part->setMaterial(material);
        part->setLocalBounds(bounds);

        if (!item.addPart(part))
        {
            itemManager().removePart(part->id());
            return false;
        }
    }

    return true;
}

bool BRepViewerWidget::attachEdgeParts(RenderItem& item,
                                       const std::vector<Topology_Edge>& edges,
                                       const Material* material,
                                       const QString& name)
{
    if (material == 0) return false;

    for (std::size_t index = 0; index < edges.size(); ++index)
    {
        const ResourceId resourceId = acquireEdgeResource(edges[index], name);
        if (resourceId == InvalidResourceId) return false;

        const BufferGeometry* geometry = static_cast<const BufferGeometry*>(resourceManager().get(resourceId));
        if (geometry == 0) return false;

        const AxisAlignedBoundingBox bounds = geometryBounds(*geometry);
        if (!bounds.isValid()) return false;

        RenderPart* part = itemManager().createPart();
        if (part == 0) return false;

        part->setGeometry(geometry);
        part->setMaterial(material);
        part->setLocalBounds(bounds);

        if (!item.addPart(part))
        {
            itemManager().removePart(part->id());
            return false;
        }
    }

    return true;
}
bool BRepViewerWidget::removeItemParts(RenderItem& item)
{
    std::vector<RenderPartId> partIds;
    partIds.reserve(static_cast<std::size_t>(item.partCount()));

    for (int index = 0; index < item.partCount(); ++index)
    {
        const RenderPart* part = item.partAt(index);
        if (part != 0) partIds.push_back(part->id());
    }

    bool result = true;

    for (std::size_t index = 0; index < partIds.size(); ++index)
    {
        if (!item.removePart(partIds[index]))
        {
            result = false;
            continue;
        }

        if (!itemManager().removePart(partIds[index])) result = false;
    }

    return result;
}
/// 统一Instance显示

BRepDisplayId BRepViewerWidget::addInstance(InstanceId instanceId,
                                            const Tool::TopologyCollection& topology,
                                            const MyMath::Matrix4& localToWorld,
                                            const QString& name,
                                            const BRepDisplayStyle& style,
                                            bool showFaces,
                                            bool showEdges)
{
    if (instanceId == InvalidInstanceId || topology.empty() || (!showFaces && !showEdges))
    {
        return InvalidBRepDisplayId;
    }

    if (showFaces)
    {
        if (!style.isFaceValid() || topology.faces.empty()) return InvalidBRepDisplayId;
    }
    else
    {
        if (!style.isWireframeValid()) return InvalidBRepDisplayId;
    }

    if (showEdges && topology.edges.empty()) return InvalidBRepDisplayId;
    if (m_displayManager.containsInstance(instanceId)) return InvalidBRepDisplayId;

    QVector3D position;
    QQuaternion rotation;
    QVector3D scale;

    if (!decomposeItemTransform(localToWorld, position, rotation, scale))
    {
        return InvalidBRepDisplayId;
    }

    Material* surfaceMaterial = 0;
    Material* wireframeMaterial = 0;

    if (showFaces)
    {
        surfaceMaterial = materialManager().createMaterial(name + "_SurfaceMaterial");

        if (surfaceMaterial == 0 ||
            !surfaceMaterial->setSurfaceMode(SurfaceMode::Color) ||
            !surfaceMaterial->setColor(style.surfaceColor))
        {
            if (surfaceMaterial != 0) materialManager().remove(surfaceMaterial->id());
            return InvalidBRepDisplayId;
        }

        // surfaceMaterial->setLightingEnabled(style.surfaceLightingEnabled);
    }

    if (showEdges)
    {
        wireframeMaterial = materialManager().createMaterial(name + "_WireframeMaterial");

        if (wireframeMaterial == 0 ||
            !wireframeMaterial->setSurfaceMode(SurfaceMode::Color) ||
            !wireframeMaterial->setColor(style.wireColor))
        {
            if (wireframeMaterial != 0) materialManager().remove(wireframeMaterial->id());
            if (surfaceMaterial != 0) materialManager().remove(surfaceMaterial->id());
            return InvalidBRepDisplayId;
        }
        wireframeMaterial->setLightingEnabled(false);
    }

    RenderItem* item = itemManager().createItem(name);

    if (item == 0)
    {
        if (wireframeMaterial != 0) materialManager().remove(wireframeMaterial->id());
        if (surfaceMaterial != 0) materialManager().remove(surfaceMaterial->id());
        return InvalidBRepDisplayId;
    }

    item->transform().setPosition(position);
    item->transform().setRotation(rotation);
    item->transform().setScale(scale);

    if (showFaces && !attachFaceParts(*item, topology.faces, surfaceMaterial, name))
    {
        itemManager().remove(item->id());
        if (wireframeMaterial != 0) materialManager().remove(wireframeMaterial->id());
        if (surfaceMaterial != 0) materialManager().remove(surfaceMaterial->id());
        return InvalidBRepDisplayId;
    }

    if (showEdges && !attachEdgeParts(*item, topology.edges, wireframeMaterial, name))
    {
        itemManager().remove(item->id());
        if (wireframeMaterial != 0) materialManager().remove(wireframeMaterial->id());
        if (surfaceMaterial != 0) materialManager().remove(surfaceMaterial->id());
        return InvalidBRepDisplayId;
    }

    if (!m_displayManager.bindInstance(instanceId, item->id()))
    {
        itemManager().remove(item->id());
        if (wireframeMaterial != 0) materialManager().remove(wireframeMaterial->id());
        if (surfaceMaterial != 0) materialManager().remove(surfaceMaterial->id());
        return InvalidBRepDisplayId;
    }

    const BRepDisplayId displayId = allocateDisplayId();

    if (displayId == InvalidBRepDisplayId)
    {
        m_displayManager.unbindInstance(instanceId);
        itemManager().remove(item->id());
        if (wireframeMaterial != 0) materialManager().remove(wireframeMaterial->id());
        if (surfaceMaterial != 0) materialManager().remove(surfaceMaterial->id());
        return InvalidBRepDisplayId;
    }

    BRepDisplayObject object;
    object.id = displayId;
    object.itemId = item->id();
    object.surfaceMaterialId = surfaceMaterial != 0 ? surfaceMaterial->id() : InvalidMaterialId;
    object.wireframeMaterialId = wireframeMaterial != 0 ? wireframeMaterial->id() : InvalidMaterialId;

    m_displays[displayId] = object;
    update();

    return displayId;
}

/// Wireframe Instance

BRepDisplayId BRepViewerWidget::addWireframe(const Edge& edge, const QString& name, const BRepDisplayStyle& style)
{
    if (!edge.isValid()) return InvalidBRepDisplayId;

    return addInstance(edge.id(),
                       Tool::TopologyCollector::collect(edge.topology()),
                       edge.localToWorld(),
                       name,
                       style,
                       false,
                       true);
}

BRepDisplayId BRepViewerWidget::addWireframe(const Wire& wire, const QString& name, const BRepDisplayStyle& style)
{
    if (!wire.isValid()) return InvalidBRepDisplayId;

    return addInstance(wire.id(),
                       Tool::TopologyCollector::collect(wire.topology()),
                       wire.localToWorld(),
                       name,
                       style,
                       false,
                       true);
}

BRepDisplayId BRepViewerWidget::addWireframe(const Face& face, const QString& name, const BRepDisplayStyle& style)
{
    if (!face.isValid()) return InvalidBRepDisplayId;

    return addInstance(face.id(),
                       Tool::TopologyCollector::collect(face.topology()),
                       face.localToWorld(),
                       name,
                       style,
                       false,
                       true);
}

BRepDisplayId BRepViewerWidget::addWireframe(const Shell& shell, const QString& name, const BRepDisplayStyle& style)
{
    if (!shell.isValid()) return InvalidBRepDisplayId;

    return addInstance(shell.id(),
                       Tool::TopologyCollector::collect(shell.topology()),
                       shell.localToWorld(),
                       name,
                       style,
                       false,
                       true);
}

BRepDisplayId BRepViewerWidget::addWireframe(const Solid& solid, const QString& name, const BRepDisplayStyle& style)
{
    if (!solid.isValid()) return InvalidBRepDisplayId;

    return addInstance(solid.id(),
                       Tool::TopologyCollector::collect(solid.topology()),
                       solid.localToWorld(),
                       name,
                       style,
                       false,
                       true);
}

/// Surface + Edge Instance

BRepDisplayId BRepViewerWidget::addFace(const Face& face, const QString& name, const BRepDisplayStyle& style)
{
    if (!face.isValid()) return InvalidBRepDisplayId;

    return addInstance(face.id(),
                       Tool::TopologyCollector::collect(face.topology()),
                       face.localToWorld(),
                       name,
                       style,
                       true,
                       true);
}

BRepDisplayId BRepViewerWidget::addShell(const Shell& shell, const QString& name, const BRepDisplayStyle& style)
{
    if (!shell.isValid()) return InvalidBRepDisplayId;

    return addInstance(shell.id(),
                       Tool::TopologyCollector::collect(shell.topology()),
                       shell.localToWorld(),
                       name,
                       style,
                       true,
                       true);
}

BRepDisplayId BRepViewerWidget::addSolid(const Solid& solid, const QString& name, const BRepDisplayStyle& style)
{
    if (!solid.isValid()) return InvalidBRepDisplayId;

    return addInstance(solid.id(),
                       Tool::TopologyCollector::collect(solid.topology()),
                       solid.localToWorld(),
                       name,
                       style,
                       true,
                       true);
}

/// 全局离散参数

const BRepViewerBuildOptions& BRepViewerWidget::buildOptions() const
{
    return m_buildOptions;
}

bool BRepViewerWidget::setBuildOptions(const BRepViewerBuildOptions& options)
{
    if (!options.isValid() || !m_displays.empty()) return false;
    if (!clearTopologyResources()) return false;

    m_buildOptions = options;
    return true;
}

/// 身份映射

const BRepDisplayManager& BRepViewerWidget::displayManager() const
{
    return m_displayManager;
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
    if (iterator == m_displays.end()) return false;

    if (!removeDisplayResources(iterator->second)) return false;

    m_displays.erase(iterator);

    if (m_displays.empty()) m_nextDisplayId = 1;

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

    if (m_displays.empty()) m_nextDisplayId = 1;

    if (!clearUnusedTopologyResources()) result = false;

    update();
    return result;
}
BRepDisplayId BRepViewerWidget::displayIdByItem(RenderItemId itemId) const
{
    if (itemId == InvalidRenderItemId) return InvalidBRepDisplayId;

    for (DisplayMap::const_iterator iterator = m_displays.begin(); iterator != m_displays.end(); ++iterator)
    {
        if (iterator->second.itemId == itemId) return iterator->first;
    }

    return InvalidBRepDisplayId;
}
bool BRepViewerWidget::refreshPlacement(const Instance_Object& instance)
{
    return refreshPlacement(instance.id(), instance.localToWorld());
}
bool BRepViewerWidget::refreshDisplay(const Instance_Object& instance, const BRepDisplayStyle& style)
{
    const RenderItemId itemId = m_displayManager.itemId(instance.id());
    if (itemId == InvalidRenderItemId) return false;

    const BRepDisplayId displayId = displayIdByItem(itemId);
    if (displayId == InvalidBRepDisplayId) return false;

    return refreshDisplay(displayId, style);
}
/// 内部释放

bool BRepViewerWidget::removeDisplayResources(BRepDisplayObject& object)
{
    const InstanceId instanceId =
        object.itemId != InvalidRenderItemId
            ? m_displayManager.instanceId(object.itemId)
            : InvalidInstanceId;

    if (object.itemId != InvalidRenderItemId && itemManager().contains(object.itemId))
    {
        RenderItem* item = itemManager().get(object.itemId);

        if (item == 0 || !removeItemParts(*item)) return false;
        if (!itemManager().remove(object.itemId)) return false;
    }

    object.itemId = InvalidRenderItemId;

    if (instanceId != InvalidInstanceId)
    {
        m_displayManager.unbindInstance(instanceId);
    }

    bool result = true;

    if (object.wireframeMaterialId != InvalidMaterialId &&
        materialManager().contains(object.wireframeMaterialId))
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

    if (object.surfaceMaterialId != InvalidMaterialId &&
        materialManager().contains(object.surfaceMaterialId))
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

    if (result) object.clear();

    // Display删除后只尝试清理由DisplayManager自己独占的Topology。
    if (!clearUnusedTopologyResources()) result = false;

    return result;
}
bool BRepViewerWidget::clearUnusedTopologyResources()
{
    const std::vector<TopologyId> topologyIds = m_displayManager.topologyIds();

    bool needsContext = false;

    for (std::size_t index = 0; index < topologyIds.size(); ++index)
    {
        const BRepTopologyBinding* binding = m_displayManager.bindingPointer(topologyIds[index]);

        if (binding == 0 || !binding->isValid()) continue;
        if (binding->topology->referenceCount() != 1) continue;

        Resource* resource = resourceManager().contains(binding->resourceId)
                                 ? resourceManager().get(binding->resourceId)
                                 : 0;

        if (resource != 0 && resource->isInitialized())
        {
            needsContext = true;
            break;
        }
    }

    MyOpenGLContext cleanupContext;
    QOpenGLFunctions_3_3_Core* gl = 0;
    bool contextCurrent = false;

    if (needsContext)
    {
        if (context() == 0) return false;

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

    for (std::size_t index = 0; index < topologyIds.size(); ++index)
    {
        const TopologyId topologyId = topologyIds[index];
        const BRepTopologyBinding* binding = m_displayManager.bindingPointer(topologyId);

        if (binding == 0 || !binding->isValid()) continue;

        // 1表示只剩Binding自身持有的这一份RefPtr。
        if (binding->topology->referenceCount() != 1) continue;

        const ResourceId resourceId = binding->resourceId;

        if (resourceManager().contains(resourceId) && !resourceManager().remove(resourceId, gl))
        {
            result = false;
            continue;
        }

        // Resource已经解除后再删除Binding。
        // Binding中的RefPtr释放，Topology_TObject计数1 -> 0并自动析构。
        if (!m_displayManager.unbindTopology(topologyId)) result = false;
    }

    if (contextCurrent) doneCurrent();

    return result;
}

bool BRepViewerWidget::clearTopologyResources()
{
    const std::vector<TopologyId> topologyIds = m_displayManager.topologyIds();

    if (topologyIds.empty()) return true;

    bool needsContext = false;

    for (std::size_t index = 0; index < topologyIds.size(); ++index)
    {
        const BRepTopologyBinding* binding = m_displayManager.bindingPointer(topologyIds[index]);

        if (binding == 0 || !binding->isValid()) continue;

        Resource* resource = resourceManager().contains(binding->resourceId)
                                 ? resourceManager().get(binding->resourceId)
                                 : 0;

        if (resource != 0 && resource->isInitialized())
        {
            needsContext = true;
            break;
        }
    }

    MyOpenGLContext cleanupContext;
    QOpenGLFunctions_3_3_Core* gl = 0;
    bool contextCurrent = false;

    if (needsContext)
    {
        if (context() == 0) return false;

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

    for (std::size_t index = 0; index < topologyIds.size(); ++index)
    {
        const TopologyId topologyId = topologyIds[index];
        const BRepTopologyBinding* binding = m_displayManager.bindingPointer(topologyId);

        if (binding == 0) continue;

        const ResourceId resourceId = binding->resourceId;

        if (resourceId != InvalidResourceId &&
            resourceManager().contains(resourceId) &&
            !resourceManager().remove(resourceId, gl))
        {
            result = false;
            continue;
        }

        if (!m_displayManager.unbindTopology(topologyId)) result = false;
    }

    if (contextCurrent) doneCurrent();

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
bool BRepViewerWidget::refreshDisplay(BRepDisplayId displayId, const BRepDisplayStyle& style)
{
    DisplayMap::iterator displayIterator = m_displays.find(displayId);
    if (displayIterator == m_displays.end()) return false;

    BRepDisplayObject& object = displayIterator->second;

    const bool hasSurface = object.surfaceMaterialId != InvalidMaterialId;
    const bool hasWireframe = object.wireframeMaterialId != InvalidMaterialId;

    if (!hasSurface && !hasWireframe) return false;
    if (hasSurface ? !style.isFaceValid() : !style.isWireframeValid()) return false;

    RenderItem* item = itemManager().get(object.itemId);
    if (item == 0) return false;

    Material* surfaceMaterial = 0;
    Material* wireframeMaterial = 0;

    if (hasSurface)
    {
        surfaceMaterial = materialManager().get(object.surfaceMaterialId);
        if (surfaceMaterial == 0) return false;
    }

    if (hasWireframe)
    {
        wireframeMaterial = materialManager().get(object.wireframeMaterialId);
        if (wireframeMaterial == 0) return false;
    }

    if (surfaceMaterial != 0)
    {
        if (!surfaceMaterial->setColor(style.surfaceColor)) return false;
        surfaceMaterial->setLightingEnabled(style.surfaceLightingEnabled);
    }

    if (wireframeMaterial != 0)
    {
        if (!wireframeMaterial->setColor(style.wireColor)) return false;
        wireframeMaterial->setLightingEnabled(false);
    }

    if (hasWireframe)
    {
        for (int index = 0; index < item->partCount(); ++index)
        {
            RenderPart* part = item->partAt(index);
            if (part == 0 || part->geometry() == 0) continue;

            const RenderType renderType = part->geometry()->renderType();

            if (renderType == RenderType::Lines || renderType == RenderType::LineStrip)
            {
                if (!part->setLineWidth(style.wireWidth)) return false;
            }
        }
    }

    update();
    return true;
}
bool BRepViewerWidget::refreshPlacement(InstanceId instanceId, const MyMath::Matrix4& localToWorld)
{
    if (instanceId == InvalidInstanceId) return false;

    const RenderItemId itemId = m_displayManager.itemId(instanceId);
    if (itemId == InvalidRenderItemId) return false;

    RenderItem* item = itemManager().get(itemId);
    if (item == 0) return false;

    QVector3D position;
    QQuaternion rotation;
    QVector3D scale;

    if (!decomposeItemTransform(localToWorld, position, rotation, scale)) return false;

    item->transform().setPosition(position);
    item->transform().setRotation(rotation);
    item->transform().setScale(scale);

    update();
    return true;
}
}
}