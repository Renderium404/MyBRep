#include "NavigationTool.h"

#include <QDebug>
#include <QMouseEvent>
#include <QObject>
#include <QOpenGLContext>
#include <QOpenGLFunctions_3_3_Core>
#include <QVector4D>
#include <QWheelEvent>
#include <cmath>
#include <vector>

#include "MyOpenGL/Camera/Camera.h"
#include "MyOpenGL/Material/Material.h"
#include "MyOpenGL/Render/RenderContext.h"
#include "MyOpenGL/Render/Renderer.h"
#include "MyOpenGL/Render/RenderState.h"
#include "MyOpenGL/Viewer/OpenGLViewerWidget.h"

NavigationTool::NavigationTool()
    : m_viewer(0)
    , m_hasNavigationAnchor(false)
    , m_anchorGeometry("NavigationAnchor", BufferUsage::Static, RenderType::Lines)
    , m_anchorMaterial(0)
    , m_anchorVisible(false)
    , m_anchorPixelSize(28)
    , m_resourcesReady(false)
{
    m_anchorHideTimer.setSingleShot(true);

    QObject::connect(&m_anchorHideTimer, &QTimer::timeout, [this]()
    {
        m_anchorVisible = false;
        if (m_viewer != 0) m_viewer->update();
    });
}

NavigationTool::~NavigationTool()
{
    if (m_resourcesReady && m_viewer != 0)
        releaseResources(m_viewer);
}

void NavigationTool::activate(OpenGLViewerWidget* viewer)
{
    m_viewer = viewer;
    reset();

    if (!createResources(viewer))
        qWarning() << "NavigationTool activate failed: unable to create resources.";
}

void NavigationTool::deactivate(OpenGLViewerWidget* viewer)
{
    reset();
    releaseResources(viewer);
    m_viewer = 0;
}

void NavigationTool::reset()
{
    m_anchorHideTimer.stop();
    m_lastMousePosition = QPointF();
    m_navigationAnchor = QVector3D();
    m_hasNavigationAnchor = false;
    m_anchorVisible = false;

    if (m_viewer != 0)
        m_viewer->update();
}

bool NavigationTool::mousePressEvent(OpenGLViewerWidget* viewer, QMouseEvent* event)
{
    if (viewer == 0 || event == 0)
        return false;

    m_lastMousePosition = event->pos();

    if (event->button() == Qt::LeftButton)
    {
        RenderContext context;

        if (viewer->buildRenderContext(context))
        {
            ViewNavigationFace face;

            if (viewer->viewNavigation().hitTest(event->pos(), context, face))
            {
                QVector3D forward;
                QVector3D up;
                QVector3D anchor;

                if (viewer->viewNavigation().viewDirection(face, forward, up) && navigationAnchor(viewer, anchor) &&
                    viewer->cameraManager().setViewDirection(anchor, forward, up))
                {
                    viewer->update();
                }

                return true;
            }
        }

        m_navigationAnchor = screenPointToAnchor(viewer, event->pos());
        m_hasNavigationAnchor = true;
        m_anchorHideTimer.stop();
        m_anchorVisible = true;
        viewer->update();
        return true;
    }

    if (event->button() == Qt::MiddleButton)
    {
        m_navigationAnchor = screenPointToAnchor(viewer, event->pos());
        m_hasNavigationAnchor = true;
        m_anchorHideTimer.stop();
        m_anchorVisible = true;
        viewer->update();
        return true;
    }

    return false;
}

bool NavigationTool::mouseMoveEvent(OpenGLViewerWidget* viewer, QMouseEvent* event)
{
    if (viewer == 0 || event == 0)
        return false;

    const QPointF currentPosition = event->pos();
    const QPointF delta = currentPosition - m_lastMousePosition;
    m_lastMousePosition = currentPosition;

    if (!m_hasNavigationAnchor)
        return false;

    if (event->buttons() & Qt::LeftButton)
    {
        const float degreesPerPixel = 0.3f;

        if (viewer->cameraManager().orbitAround(m_navigationAnchor, -delta.x() * degreesPerPixel, -delta.y() * degreesPerPixel))
            viewer->update();

        return true;
    }

    if (event->buttons() & Qt::MiddleButton)
    {
        if (viewer->cameraManager().panAt(m_navigationAnchor, delta.x(), delta.y(), viewer->width(), viewer->height()))
            viewer->update();

        return true;
    }

    return false;
}

bool NavigationTool::mouseReleaseEvent(OpenGLViewerWidget* viewer, QMouseEvent* event)
{
    if (viewer == 0 || event == 0 || !m_hasNavigationAnchor)
        return false;

    if (event->button() != Qt::LeftButton && event->button() != Qt::MiddleButton)
        return false;

    m_hasNavigationAnchor = false;
    m_anchorHideTimer.stop();
    m_anchorVisible = false;
    viewer->update();
    return true;
}

bool NavigationTool::wheelEvent(OpenGLViewerWidget* viewer, QWheelEvent* event)
{
    if (viewer == 0 || event == 0)
        return false;

    const QVector3D anchor = screenPointToZoomAnchor(viewer, event->pos());
    m_navigationAnchor = anchor;
    m_anchorVisible = true;

    const float wheelSteps = static_cast<float>(event->angleDelta().y()) / 120.0f;
    const float factor = static_cast<float>(std::pow(1.15, wheelSteps));

    if (viewer->cameraManager().zoomAt(anchor, factor, viewer->width(), viewer->height()))
        viewer->update();

    m_anchorHideTimer.start(350);
    return true;
}

bool NavigationTool::drawSceneFront(Renderer& renderer, const RenderContext& context) const
{
    if (!m_resourcesReady || !m_anchorVisible || m_anchorMaterial == 0)
        return true;

    RenderState state;

    if (!buildAnchorRenderState(context, state))
        return true;

    const std::vector<const Light*> noLights;

    if (!renderer.clearDepth(state.viewport))
        return false;

    return renderer.drawGeometry(&m_anchorGeometry, m_anchorMaterial, state, noLights);
}

bool NavigationTool::createResources(OpenGLViewerWidget* viewer)
{
    if (viewer == 0)
        return false;

    if (m_resourcesReady)
        return true;

    if (!buildAnchorGeometry())
        return false;

    Material* material = viewer->materialManager().createMaterial("NavigationAnchorMaterial");

    if (material == 0)
        return false;

    if (!material->setSurfaceMode(SurfaceMode::VertexColor))
    {
        viewer->materialManager().remove(material->id());
        return false;
    }

    material->setLightingEnabled(false);

    if (viewer->resourceManager().borrow(&m_anchorGeometry) == InvalidResourceId)
    {
        viewer->materialManager().remove(material->id());
        return false;
    }

    m_anchorMaterial = material;
    m_resourcesReady = true;
    return true;
}

void NavigationTool::releaseResources(OpenGLViewerWidget* viewer)
{
    if (!m_resourcesReady || viewer == 0)
        return;

    m_anchorHideTimer.stop();
    m_anchorVisible = false;

    bool contextCurrent = false;
    QOpenGLFunctions_3_3_Core* gl = 0;

    if (viewer->context() != 0 && viewer->m_openGLContext.isInitialized())
    {
        viewer->makeCurrent();

        if (QOpenGLContext::currentContext() == viewer->context())
        {
            gl = viewer->m_openGLContext.gl();
            contextCurrent = true;
        }
    }

    const ResourceId geometryId = m_anchorGeometry.id();

    if (geometryId != InvalidResourceId && viewer->resourceManager().contains(geometryId) &&
        !viewer->resourceManager().remove(geometryId, gl))
    {
        qWarning() << "NavigationTool releaseResources failed: NavigationAnchor Geometry.";
    }

    if (m_anchorMaterial != 0)
    {
        const MaterialId materialId = m_anchorMaterial->id();

        if (materialId != InvalidMaterialId && viewer->materialManager().contains(materialId) &&
            !viewer->materialManager().remove(materialId))
        {
            qWarning() << "NavigationTool releaseResources failed: NavigationAnchor Material.";
        }

        m_anchorMaterial = 0;
    }

    if (contextCurrent)
        viewer->doneCurrent();

    m_resourcesReady = false;
}

bool NavigationTool::buildAnchorGeometry()
{
    const QVector3D color(1.0f, 0.75f, 0.1f);
    const float length = 0.78f;

    const std::vector<GLfloat> vertices =
    {
        -length, 0.0f, 0.0f, color.x(), color.y(), color.z(),
         length, 0.0f, 0.0f, color.x(), color.y(), color.z(),
         0.0f, -length, 0.0f, color.x(), color.y(), color.z(),
         0.0f,  length, 0.0f, color.x(), color.y(), color.z()
    };

    const std::vector<GLuint> indices = { 0, 1, 2, 3 };
    std::vector<GeometryVertexAttribute> attributes;

    GeometryVertexAttribute position;
    position.location = GeometryAttribute::Position;
    position.componentCount = 3;
    position.valueOffset = 0;
    attributes.push_back(position);

    GeometryVertexAttribute colorAttribute;
    colorAttribute.location = GeometryAttribute::Color;
    colorAttribute.componentCount = 3;
    colorAttribute.valueOffset = 3;
    attributes.push_back(colorAttribute);

    m_anchorGeometry.setVertexLayout(6, attributes);
    m_anchorGeometry.setVertexData(vertices);
    m_anchorGeometry.setIndexData(indices);
    return true;
}

bool NavigationTool::buildAnchorRenderState(const RenderContext& context, RenderState& state) const
{
    if (!m_anchorVisible || !context.isValid())
        return false;

    const QVector4D clip = context.projection * context.view * QVector4D(m_navigationAnchor, 1.0f);

    if (clip.w() <= 1.0e-8f)
        return false;

    const float ndcX = clip.x() / clip.w();
    const float ndcY = clip.y() / clip.w();
    const float ndcZ = clip.z() / clip.w();

    if (ndcX < -1.0f || ndcX > 1.0f || ndcY < -1.0f || ndcY > 1.0f || ndcZ < -1.0f || ndcZ > 1.0f)
        return false;

    const float pixelX = (ndcX * 0.5f + 0.5f) * context.viewportWidth;
    const float pixelY = (ndcY * 0.5f + 0.5f) * context.viewportHeight;
    const int halfSize = m_anchorPixelSize / 2;

    state = RenderState();
    state.model.setToIdentity();
    state.view.setToIdentity();
    state.view.lookAt(QVector3D(0.0f, 0.0f, 3.0f), QVector3D(0.0f, 0.0f, 0.0f), QVector3D(0.0f, 1.0f, 0.0f));
    state.projection.setToIdentity();
    state.projection.ortho(-1.0f, 1.0f, -1.0f, 1.0f, 0.1f, 10.0f);
    state.viewport = RenderViewport(static_cast<int>(pixelX) - halfSize, static_cast<int>(pixelY) - halfSize, m_anchorPixelSize, m_anchorPixelSize);
    state.depthTestEnabled = true;
    state.depthWriteEnabled = true;
    return state.viewport.isValid();
}

bool NavigationTool::navigationAnchor(OpenGLViewerWidget* viewer, QVector3D& anchor) const
{
    if (viewer == 0)
        return false;

    if (viewer->cameraManager().hasViewBounds())
    {
        anchor = viewer->cameraManager().viewBounds().center();
        return true;
    }

    AxisAlignedBoundingBox bounds;

    if (!viewer->itemManager().worldBounds(bounds, true))
        return false;

    anchor = bounds.center();
    return true;
}

QVector3D NavigationTool::screenPointToZoomAnchor(OpenGLViewerWidget* viewer, const QPointF& position) const
{
    if (viewer == 0)
        return QVector3D();

    const Camera* camera = viewer->cameraManager().activeCamera();

    if (camera == 0 || viewer->width() <= 0 || viewer->height() <= 0)
        return viewer->coordinateSystem().worldOrigin();

    QVector3D scenePoint;

    if (viewer->scenePointAtWorld(position, scenePoint))
        return scenePoint;

    QVector3D rayOrigin;
    QVector3D rayDirection;

    if (!camera->screenPointToRay(position.x(), position.y(), viewer->width(), viewer->height(), rayOrigin, rayDirection))
        return viewer->coordinateSystem().worldOrigin();

    const QVector3D forward = camera->forward();
    const float middleDepth = (camera->nearPlane() + camera->farPlane()) * 0.5f;
    const QVector3D planePoint = camera->position() + forward * middleDepth;
    const float denominator = QVector3D::dotProduct(rayDirection, forward);

    if (qAbs(denominator) <= 1.0e-8f)
        return planePoint;

    const float distance = QVector3D::dotProduct(planePoint - rayOrigin, forward) / denominator;

    if (distance < 0.0f)
        return planePoint;

    return rayOrigin + rayDirection * distance;
}

QVector3D NavigationTool::screenPointToAnchor(OpenGLViewerWidget* viewer, const QPointF& position) const
{
    if (viewer == 0)
        return QVector3D();

    QVector3D scenePoint;

    if (viewer->scenePointAtWorld(position, scenePoint))
        return scenePoint;

    QVector3D anchor;

    if (navigationAnchor(viewer, anchor))
        return anchor;

    return viewer->coordinateSystem().worldOrigin();
}