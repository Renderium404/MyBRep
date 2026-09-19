#include "BRepViewerWidget.h"

#include <QDebug>
#include <QOpenGLContext>
#include <QQuaternion>
#include <QVector3D>

#include <cmath>
#include <vector>

#include "MyMath/Matrix3.h"
#include "MyMath/Quaternion.h"

#include "MyBRepOpenGL/Builder/BRepFaceBuilder.h"
#include "MyBRepOpenGL/Builder/BRepShellBuilder.h"
#include "MyBRepOpenGL/Builder/BRepSolidBuilder.h"
#include "MyBRepOpenGL/Builder/BRepWireframeBuilder.h"
#include "MyOpenGL/Item/AxisAlignedBoundingBox.h"
#include "MyOpenGL/Item/RenderPart.h"
#include "MyOpenGL/Light/Light.h"
#include "MyOpenGL/Material/Material.h"
#include "MyOpenGL/Render/MyOpenGLContext.h"
#include "MyOpenGL/Resource/BufferGeometry.h"

namespace MyBRep
{
namespace Display
{

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

bool BRepViewerWidget::createDefaultBRepLighting(LightManager& lightManager)
{
    Light* ambientLight = lightManager.createLight("BRepAmbientLight");

    if (ambientLight == 0)
    {
        return false;
    }

    ambientLight->setAmbient();

    if (!ambientLight->setColor(QVector3D(1.0f, 1.0f, 1.0f)) || !ambientLight->setIntensity(0.18f))
    {
        return false;
    }

    Light* keyLight = lightManager.createLight("BRepKeyLight");

    if (keyLight == 0)
    {
        return false;
    }

    if (!keyLight->setDirectional(QVector3D(-0.45f, -0.35f, -1.0f)) ||
        !keyLight->setColor(QVector3D(1.0f, 0.97f, 0.92f)) ||
        !keyLight->setIntensity(0.78f))
    {
        return false;
    }

    Light* fillLight = lightManager.createLight("BRepFillLight");

    if (fillLight == 0)
    {
        return false;
    }

    return fillLight->setDirectional(QVector3D(0.65f, -0.10f, -0.60f)) &&
           fillLight->setColor(QVector3D(0.72f, 0.84f, 1.0f)) &&
           fillLight->setIntensity(0.24f);
}

bool BRepViewerWidget::decomposeItemTransform(
    const MyMath::Matrix4& localToWorld,
    QVector3D& position,
    QQuaternion& rotation,
    QVector3D& scale)
{
    if (!localToWorld.isAffine() || !localToWorld.isInvertible())
    {
        return false;
    }

    MyMath::Vector3 axisX(localToWorld(0, 0), localToWorld(1, 0), localToWorld(2, 0));
    MyMath::Vector3 axisY(localToWorld(0, 1), localToWorld(1, 1), localToWorld(2, 1));
    MyMath::Vector3 axisZ(localToWorld(0, 2), localToWorld(1, 2), localToWorld(2, 2));

    double scaleX = axisX.length();
    double scaleY = axisY.length();
    double scaleZ = axisZ.length();

    if (scaleX <= 0.0 || scaleY <= 0.0 || scaleZ <= 0.0)
    {
        return false;
    }

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

    if (!rotationMatrix.isRotationMatrix(orthogonalTolerance))
    {
        return false;
    }

    const MyMath::Quaternion quaternion = MyMath::Quaternion::fromRotationMatrix(rotationMatrix, orthogonalTolerance);

    if (!quaternion.isUnit(orthogonalTolerance))
    {
        return false;
    }

    const MyMath::Vector3 translation = localToWorld.translation();
    position = QVector3D(static_cast<float>(translation.x()), static_cast<float>(translation.y()), static_cast<float>(translation.z()));
    rotation = QQuaternion(static_cast<float>(quaternion.w()), static_cast<float>(quaternion.x()),
                           static_cast<float>(quaternion.y()), static_cast<float>(quaternion.z()));
    scale = QVector3D(static_cast<float>(scaleX), static_cast<float>(scaleY), static_cast<float>(scaleZ));
    return true;
}

BRepViewerWidget::BRepViewerWidget(QWidget* parent)
    : OpenGLViewerWidget(parent)
    , m_nextDisplayId(1)
    , m_nextSolidGeometryResourceId(1)
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

    return attachWireframe(BRepWireframeBuilder::build(edge, localToWorld, name + "_WireframeGeometry", m_buildOptions.wireframe), name, style);
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

    return attachWireframe(BRepWireframeBuilder::build(wire, localToWorld, name + "_WireframeGeometry", m_buildOptions.wireframe), name, style);
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

    return attachWireframe(BRepWireframeBuilder::build(face, localToWorld, name + "_WireframeGeometry", m_buildOptions.wireframe), name, style);
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
        BRepFaceBuilder::build(face, localToWorld, name + "_SurfaceGeometry", m_buildOptions.surface);

    BufferGeometry* wireframeGeometry =
        BRepWireframeBuilder::build(face, localToWorld, name + "_WireframeGeometry", m_buildOptions.wireframe);

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

    return attachWireframe(BRepWireframeBuilder::build(shell, localToWorld, name + "_WireframeGeometry", m_buildOptions.wireframe), name, style);
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
    options.surface = m_buildOptions.surface;
    options.wireframe = m_buildOptions.wireframe;

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

    return attachWireframe(BRepWireframeBuilder::build(solid, localToWorld, name + "_WireframeGeometry", m_buildOptions.wireframe), name, style);
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

    const BRepSolidBuildOptions& options = m_buildOptions;

    QVector3D position;
    QQuaternion rotation;
    QVector3D scale;

    if (decomposeItemTransform(localToWorld, position, rotation, scale))
    {
        const BRepSolidGeometryResourceId geometryResourceId = acquireSolidGeometryResource(solid, name);
        return geometryResourceId != InvalidBRepSolidGeometryResourceId
                   ? attachSharedSolidDisplay(geometryResourceId, localToWorld, name, style)
                   : InvalidBRepDisplayId;
    }

    BufferGeometry* surfaceGeometry = BRepSolidBuilder::buildSurface(solid, localToWorld, name + "_SurfaceGeometry", options);
    BufferGeometry* wireframeGeometry = BRepSolidBuilder::buildBoundary(solid, localToWorld, name + "_WireframeGeometry", options);

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

/// 全局离散参数

const BRepSolidBuildOptions& BRepViewerWidget::buildOptions() const
{
    return m_buildOptions;
}

bool BRepViewerWidget::setBuildOptions(const BRepSolidBuildOptions& options)
{
    if (!options.isValid() || !m_displays.empty())
    {
        return false;
    }

    if (!clearSolidGeometryResources())
    {
        return false;
    }

    m_buildOptions = options;
    return true;
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

        if (!clearSolidGeometryResources())
        {
            result = false;
        }
    }

    update();
    return result;
}

/// 共享Solid Geometry资源

BRepSolidGeometryResourceId BRepViewerWidget::findSolidGeometryResource(const Topology_Solid& solid) const
{
    for (SolidGeometryResourceMap::const_iterator iterator = m_solidGeometryResources.begin();
         iterator != m_solidGeometryResources.end(); ++iterator)
    {
        if (iterator->second.matches(solid))
        {
            return iterator->first;
        }
    }

    return InvalidBRepSolidGeometryResourceId;
}

BRepSolidGeometryResourceId BRepViewerWidget::acquireSolidGeometryResource(const Topology_Solid& solid,const QString& name)
{
    const BRepSolidGeometryResourceId existing = findSolidGeometryResource(solid);

    if (existing != InvalidBRepSolidGeometryResourceId)
    {
        return existing;
    }

    BufferGeometry* surfaceGeometry = BRepSolidBuilder::buildSurface(solid, name + "_SharedSurfaceGeometry", m_buildOptions);
    BufferGeometry* wireframeGeometry = BRepSolidBuilder::buildBoundary(solid, name + "_SharedWireframeGeometry", m_buildOptions);

    if (surfaceGeometry == 0 || wireframeGeometry == 0)
    {
        delete surfaceGeometry;
        delete wireframeGeometry;
        return InvalidBRepSolidGeometryResourceId;
    }

    const ResourceId surfaceGeometryId = resourceManager().adopt(surfaceGeometry);

    if (surfaceGeometryId == InvalidResourceId)
    {
        delete surfaceGeometry;
        delete wireframeGeometry;
        return InvalidBRepSolidGeometryResourceId;
    }

    const ResourceId wireframeGeometryId = resourceManager().adopt(wireframeGeometry);

    if (wireframeGeometryId == InvalidResourceId)
    {
        delete wireframeGeometry;
        resourceManager().remove(surfaceGeometryId);
        return InvalidBRepSolidGeometryResourceId;
    }

    const BRepSolidGeometryResourceId id = allocateSolidGeometryResourceId();
    m_solidGeometryResources[id] = BRepSolidGeometryResource(id, solid, surfaceGeometryId, wireframeGeometryId);
    return id;
}

const BRepSolidGeometryResource* BRepViewerWidget::solidGeometryResource(BRepSolidGeometryResourceId id) const
{
    SolidGeometryResourceMap::const_iterator iterator = m_solidGeometryResources.find(id);
    return iterator != m_solidGeometryResources.end() ? &iterator->second : 0;
}

BRepDisplayId BRepViewerWidget::attachSharedSolidDisplay(
    BRepSolidGeometryResourceId geometryResourceId,
    const MyMath::Matrix4& localToWorld,
    const QString& name,
    const BRepDisplayStyle& style)
{
    const BRepSolidGeometryResource* shared = solidGeometryResource(geometryResourceId);

    if (shared == 0 || !shared->isValid() || !style.isValid())
    {
        return InvalidBRepDisplayId;
    }

    BufferGeometry* surfaceGeometry = static_cast<BufferGeometry*>(resourceManager().get(shared->surfaceGeometryId()));
    BufferGeometry* wireframeGeometry = static_cast<BufferGeometry*>(resourceManager().get(shared->wireframeGeometryId()));

    if (surfaceGeometry == 0 || wireframeGeometry == 0)
    {
        return InvalidBRepDisplayId;
    }

    const AxisAlignedBoundingBox surfaceBounds = geometryBounds(*surfaceGeometry);
    const AxisAlignedBoundingBox wireframeBounds = geometryBounds(*wireframeGeometry);

    if (!surfaceBounds.isValid() || !wireframeBounds.isValid())
    {
        return InvalidBRepDisplayId;
    }

    QVector3D position;
    QQuaternion rotation;
    QVector3D scale;

    if (!decomposeItemTransform(localToWorld, position, rotation, scale))
    {
        return InvalidBRepDisplayId;
    }

    Material* surfaceMaterial = materialManager().createMaterial(name + "_SurfaceMaterial");

    if (surfaceMaterial == 0)
    {
        return InvalidBRepDisplayId;
    }

    Material* wireframeMaterial = materialManager().createMaterial(name + "_WireframeMaterial");

    if (wireframeMaterial == 0)
    {
        materialManager().remove(surfaceMaterial->id());
        return InvalidBRepDisplayId;
    }

    if (!surfaceMaterial->setSurfaceMode(SurfaceMode::Color) ||
        !surfaceMaterial->setColor(style.surfaceColor) ||
        !wireframeMaterial->setSurfaceMode(SurfaceMode::Color) ||
        !wireframeMaterial->setColor(style.wireColor))
    {
        materialManager().remove(wireframeMaterial->id());
        materialManager().remove(surfaceMaterial->id());
        return InvalidBRepDisplayId;
    }

    surfaceMaterial->setLightingEnabled(style.surfaceLightingEnabled);
    wireframeMaterial->setLightingEnabled(false);

    RenderItem* item = itemManager().createItem(name);

    if (item == 0)
    {
        materialManager().remove(wireframeMaterial->id());
        materialManager().remove(surfaceMaterial->id());
        return InvalidBRepDisplayId;
    }

    item->transform().setPosition(position);
    item->transform().setRotation(rotation);
    item->transform().setScale(scale);

    RenderPart* wireframePart = itemManager().createPart();
    item->addPart(wireframePart);
    if (wireframePart == 0)
    {
        itemManager().remove(item->id());
        materialManager().remove(wireframeMaterial->id());
        materialManager().remove(surfaceMaterial->id());
        return InvalidBRepDisplayId;
    }

    wireframePart->setGeometry(wireframeGeometry);
    wireframePart->setMaterial(wireframeMaterial);
    wireframePart->setLocalBounds(wireframeBounds);

    RenderPart* surfacePart = itemManager().createPart();
    item->addPart(surfacePart);
    if (surfacePart == 0)
    {
        itemManager().remove(item->id());
        materialManager().remove(wireframeMaterial->id());
        materialManager().remove(surfaceMaterial->id());
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
        return InvalidBRepDisplayId;
    }

    BRepDisplayObject object;
    object.id = displayId;
    object.itemId = item->id();
    object.solidGeometryResourceId = geometryResourceId;
    object.surfaceGeometryId = shared->surfaceGeometryId();
    object.wireframeGeometryId = shared->wireframeGeometryId();
    object.surfaceMaterialId = surfaceMaterial->id();
    object.wireframeMaterialId = wireframeMaterial->id();

    m_displays[displayId] = object;
    update();
    return displayId;
}

bool BRepViewerWidget::removeGeometryResources(ResourceId surfaceGeometryId,ResourceId wireframeGeometryId)
{
    Resource* surfaceResource =
        surfaceGeometryId != InvalidResourceId && resourceManager().contains(surfaceGeometryId) ? resourceManager().get(surfaceGeometryId) : 0;
    Resource* wireframeResource =
        wireframeGeometryId != InvalidResourceId && resourceManager().contains(wireframeGeometryId) ? resourceManager().get(wireframeGeometryId) : 0;

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

    bool result = true;

    if (wireframeGeometryId != InvalidResourceId && resourceManager().contains(wireframeGeometryId))
    {
        result = resourceManager().remove(wireframeGeometryId, gl) && result;
    }

    if (surfaceGeometryId != InvalidResourceId && resourceManager().contains(surfaceGeometryId))
    {
        result = resourceManager().remove(surfaceGeometryId, gl) && result;
    }

    if (contextCurrent)
    {
        doneCurrent();
    }

    return result;
}

bool BRepViewerWidget::clearSolidGeometryResources()
{
    bool result = true;
    SolidGeometryResourceMap::iterator iterator = m_solidGeometryResources.begin();

    while (iterator != m_solidGeometryResources.end())
    {
        SolidGeometryResourceMap::iterator current = iterator;
        ++iterator;

        if (removeGeometryResources(current->second.surfaceGeometryId(), current->second.wireframeGeometryId()))
        {
            m_solidGeometryResources.erase(current);
        }
        else
        {
            result = false;
        }
    }

    if (m_solidGeometryResources.empty())
    {
        m_nextSolidGeometryResourceId = 1;
    }

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

    RenderPart* part = itemManager().createPart();
    item->addPart(part);
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
    RenderPart* wireframePart = itemManager().createPart();
    item->addPart(wireframePart);
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

    RenderPart* surfacePart = itemManager().createPart();
    item->addPart(surfacePart);
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
    if (object.itemId != InvalidRenderItemId && itemManager().contains(object.itemId))
    {
        if (!itemManager().remove(object.itemId))
        {
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

    if (object.solidGeometryResourceId == InvalidBRepSolidGeometryResourceId)
    {
        if (!removeGeometryResources(object.surfaceGeometryId, object.wireframeGeometryId))
        {
            result = false;
        }
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

BRepSolidGeometryResourceId BRepViewerWidget::allocateSolidGeometryResourceId()
{
    while (m_nextSolidGeometryResourceId == InvalidBRepSolidGeometryResourceId ||
           m_solidGeometryResources.find(m_nextSolidGeometryResourceId) != m_solidGeometryResources.end())
    {
        ++m_nextSolidGeometryResourceId;
    }

    const BRepSolidGeometryResourceId id = m_nextSolidGeometryResourceId;
    ++m_nextSolidGeometryResourceId;
    return id;
}

}
}