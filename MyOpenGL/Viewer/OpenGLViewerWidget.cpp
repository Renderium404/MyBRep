#include "OpenGLViewerWidget.h"

#include <QAction>
#include <QContextMenuEvent>
#include <QCursor>
#include <QDebug>
#include <QKeyEvent>
#include <QMenu>
#include <QMouseEvent>
#include <QOpenGLContext>
#include <QSurfaceFormat>
#include <QVector4D>
#include <QWheelEvent>
#include <QPainter>
#include <QPaintEvent>
#include <QResizeEvent>
#include <QPoint>
#include <QPointF>
#include "MyOpenGL/Camera/Camera.h"
#include "MyOpenGL/Item/RenderItem.h"
#include "MyOpenGL/Item/RenderPart.h"
#include "MyOpenGL/Material/Material.h"
#include "MyOpenGL/Resource/Geometry.h"
#include "MyOpenGL/Viewer/Modeling/PrimitiveMeshBuilder.h"
class ViewportOverlayWidget : public QWidget
{
public:
    explicit ViewportOverlayWidget(OpenGLViewerWidget* viewer)
        : QWidget(viewer)
        , m_viewer(viewer)
    {
        setAttribute(Qt::WA_TransparentForMouseEvents, true);//鼠标穿透
        setAttribute(Qt::WA_NoSystemBackground, true);       //不自动清背景
        setAttribute(Qt::WA_TranslucentBackground, true);    //背景允许透明
        setAutoFillBackground(false);                        //不自动用调色板填背景
    }

protected:
    void paintEvent(QPaintEvent* event) override
    {
        Q_UNUSED(event);

        if (m_viewer == 0)
            return;

        QPainter painter(this);

        painter.save();
        m_viewer->drawViewportOverlay(painter);
        painter.restore();
    }

private:
    OpenGLViewerWidget* m_viewer;
};
OpenGLViewerWidget::OpenGLViewerWidget(QWidget* parent)
    : QOpenGLWidget(parent)
    , m_viewportOverlay(0)
    , m_systemVertexColorMaterial(0)
    , m_glReady(false)
    , m_releasePerformed(false)
    , m_sceneDepthWidth(0)
    , m_sceneDepthHeight(0)
    , m_sceneDepthValid(false)
{

    //OpenGL资源申请
    QSurfaceFormat format;
    format.setRenderableType(QSurfaceFormat::OpenGL);
    format.setVersion(3, 3);
    format.setOption(QSurfaceFormat::DeprecatedFunctions); //启用OpenGL的弃用功能宽线，后续可使用三角网格自行绘制宽线
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    setFormat(format);

    setFocusPolicy(Qt::StrongFocus);

    // Viewer 内统一使用十字鼠标样式。
    setCursor(QCursor(Qt::CrossCursor));
    setMouseTracking(true);
    /// 默认 Camera

    Camera* camera = m_cameraManager.createCamera("MainCamera");
    if (camera == 0)
        qWarning() << "OpenGLViewerWidget construction failed: unable to create MainCamera.";
    //坐标系，导航等系统资源构建
    buildViewerResources();
    //Tool层激活并申请内建工具资源。
    m_toolManager.activate(this);
    //悬浮层配置
    m_viewportOverlay = new ViewportOverlayWidget(this);
    m_viewportOverlay->setGeometry(rect());//将悬浮层塞满父窗口
    m_viewportOverlay->show();
    m_viewportOverlay->raise();
}

OpenGLViewerWidget::~OpenGLViewerWidget()
{
    releaseViewerGL();
    m_toolManager.deactivate(this);
    unregisterViewerResources();
}

/// Viewer 数据

ResourceManager& OpenGLViewerWidget::resourceManager()
{
    return m_resourceManager;
}

const ResourceManager& OpenGLViewerWidget::resourceManager() const
{
    return m_resourceManager;
}

MaterialManager& OpenGLViewerWidget::materialManager()
{
    return m_materialManager;
}

const MaterialManager& OpenGLViewerWidget::materialManager() const
{
    return m_materialManager;
}

LightManager& OpenGLViewerWidget::lightManager()
{
    return m_lightManager;
}

const LightManager& OpenGLViewerWidget::lightManager() const
{
    return m_lightManager;
}

CameraManager& OpenGLViewerWidget::cameraManager()
{
    return m_cameraManager;
}

const CameraManager& OpenGLViewerWidget::cameraManager() const
{
    return m_cameraManager;
}

ItemManager& OpenGLViewerWidget::itemManager()
{
    return m_itemManager;
}

const ItemManager& OpenGLViewerWidget::itemManager() const
{
    return m_itemManager;
}
ItemManager& OpenGLViewerWidget::toolItemManager()
{
    return m_toolItemManager;
}

const ItemManager& OpenGLViewerWidget::toolItemManager() const
{
    return m_toolItemManager;
}

ToolManager& OpenGLViewerWidget::toolManager()
{
    return m_toolManager;
}

const ToolManager& OpenGLViewerWidget::toolManager() const
{
    return m_toolManager;
}

void OpenGLViewerWidget::setActiveTool(ViewerTool* tool)
{
    m_toolManager.setActiveTool(this, tool);
    update();
}

ViewerTool* OpenGLViewerWidget::activeTool()
{
    return m_toolManager.activeTool();
}

const ViewerTool* OpenGLViewerWidget::activeTool() const
{
    return m_toolManager.activeTool();
}
/// Viewer 系统显示

CoordinateSystem& OpenGLViewerWidget::coordinateSystem()
{
    return m_coordinateSystem;
}

const CoordinateSystem& OpenGLViewerWidget::coordinateSystem() const
{
    return m_coordinateSystem;
}

ViewNavigation& OpenGLViewerWidget::viewNavigation()
{
    return m_viewNavigation;
}

const ViewNavigation& OpenGLViewerWidget::viewNavigation() const
{
    return m_viewNavigation;
}

/// Viewer 状态

bool OpenGLViewerWidget::viewerGLReady() const
{
    return m_glReady;
}

/// OpenGL

void OpenGLViewerWidget::initializeGL()
{
    m_glReady = false;
    m_releasePerformed = false;

    if (!m_openGLContext.initialize())
    {
        qWarning() << "OpenGLViewerWidget initializeGL failed: MyOpenGLContext initialization failed.";
        return;
    }

    // QOpenGLWidget 的 Context 可能在 Widget 生命周期中发生重建。
    // Context 销毁前必须先释放 Resource 和 Renderer 持有的 GPU Object。
    if (context() != 0)
    {
        connect(context(), &QOpenGLContext::aboutToBeDestroyed, this, [this]()
        {
            releaseViewerGL();
        }, Qt::DirectConnection);
    }

    if (!m_renderer.initialize(&m_openGLContext))
    {
        qWarning() << "OpenGLViewerWidget initializeGL failed: Renderer initialization failed.";
        releaseViewerGL();
        return;
    }

    QOpenGLFunctions_3_3_Core* gl = m_openGLContext.gl();

    if (gl == 0)
    {
        qWarning() << "OpenGLViewerWidget initializeGL failed: OpenGL functions are unavailable.";
        releaseViewerGL();
        return;
    }

    if (!m_resourceManager.syncAll(gl))
    {
        qWarning() << "OpenGLViewerWidget initializeGL failed: Resource synchronization failed.";
        releaseViewerGL();
        return;
    }

    m_renderer.setClearColor(QVector4D(192/255.0, 192/255.0, 192/255.0, 1.0f));

    m_glReady = true;

    // Viewer 初次建立 OpenGL 后，如果已经存在 Item，则自动适配一次观察范围。
    fitItemsToView();
}

void OpenGLViewerWidget::resizeGL(int width, int height)
{
    Q_UNUSED(width);
    Q_UNUSED(height);

    clearSceneDepthCache();
}
void OpenGLViewerWidget::paintGL()
{
    if (!m_glReady)
        return;
    QOpenGLFunctions_3_3_Core* gl = m_openGLContext.gl();

    if (gl == 0)
        return;
    /// Resource GPU 同步。
    if (!m_resourceManager.syncAll(gl))
    {
        qWarning() << "OpenGLViewerWidget paintGL failed: Resource synchronization failed.";
        return;
    }

    /// Frame Context。
    RenderContext renderContext;
    if (!buildRenderContext(renderContext))
    {
        qWarning() << "OpenGLViewerWidget paintGL failed: unable to build RenderContext.";
        return;
    }
    if (!m_renderer.beginFrame(renderContext))
        return;
    /// OpenGL 绘制阶段。
    drawSceneBackground(m_renderer, renderContext);
    drawOpenGLFrame(m_renderer, renderContext);
    /// 缓存主场景 Depth，前景 Viewer 对象不会影响拾取。
    cacheSceneDepth(renderContext);
    drawSceneFront(m_renderer, renderContext);
    m_renderer.endFrame();
    /// 2D Viewport Overlay。
    if (m_viewportOverlay != 0)
        m_viewportOverlay->update();
}
void OpenGLViewerWidget::drawSceneBackground(Renderer& renderer, const RenderContext& context)
{
    Q_UNUSED(renderer);
    Q_UNUSED(context);
}
void OpenGLViewerWidget::drawOpenGLFrame(Renderer& renderer, const RenderContext& context)
{
    /// 场景灯光。
    ///
    /// Viewer 只在这里根据 Light::isEnabled() 构造正常场景使用的灯光集合。
    /// Renderer 不检查 Light::isEnabled()，只使用调用者明确传入的 Light。
    std::vector<const Light*> lights;
    m_lightManager.enabledLights(lights);

    /// 用户 Item。
    if (!drawItems(renderer, m_itemManager, context, lights))
        qWarning() << "OpenGLViewerWidget paintGL failed: Item drawing failed.";
}

void OpenGLViewerWidget::drawSceneFront(Renderer& renderer, const RenderContext& context)
{
    /// Viewer 系统显示
    ///
    /// 系统 Material 禁用光照，因此这里显式传入空灯光集合。
    /// 系统对象不会依赖当前场景 Light Selection。

    const std::vector<const Light*> noLights;
    /// 工具辅助对象。

    if (!drawItems(renderer, m_toolItemManager, context, noLights))
        qWarning() << "OpenGLViewerWidget drawSceneFront failed: Tool Item drawing failed.";
    /// 世界坐标系
    ///
    /// CoordinateSystem 使用世界原点确定屏幕位置，
    /// 作为固定 Pixel 大小的 Viewer 系统显示对象绘制。

    if (m_coordinateSystem.isVisible() && m_systemVertexColorMaterial != 0)
    {
        RenderState coordinateState;

        if (m_coordinateSystem.buildRenderState(context, coordinateState))
        {
            if (!renderer.clearDepth(coordinateState.viewport))
            {
                qWarning() << "OpenGLViewerWidget paintGL failed: CoordinateSystem depth clearing failed.";
            }
            else if (!renderer.drawGeometry(&m_coordinateSystem.geometry(), m_systemVertexColorMaterial, coordinateState, noLights))
            {
                qWarning() << "OpenGLViewerWidget paintGL failed: CoordinateSystem drawing failed.";
            }
        }
    }

    /// Tool前景显示。
    if (!m_toolManager.drawSceneFront(this, renderer, context))
        qWarning() << "OpenGLViewerWidget drawSceneFront failed: Tool drawing failed.";

    /// 右上角视图导航
    ///
    /// ViewNavigation 是独立 Overlay，拥有自己的局部 Viewport。
    /// 为避免被主场景 Depth 遮挡，只清除它自己的 Viewport Depth。

    if (m_viewNavigation.isVisible() && m_systemVertexColorMaterial != 0)
    {
        RenderState navigationState;

        if (m_viewNavigation.buildRenderState(context, navigationState))
        {
            if (!renderer.clearDepth(navigationState.viewport))
            {
                qWarning() << "OpenGLViewerWidget paintGL failed: ViewNavigation depth clearing failed.";
            }
            else
            {
                if (!renderer.drawGeometry(&m_viewNavigation.faceGeometry(), m_systemVertexColorMaterial, navigationState, noLights))
                    qWarning() << "OpenGLViewerWidget paintGL failed: ViewNavigation face drawing failed.";

                if (!renderer.drawGeometry(&m_viewNavigation.axisGeometry(), m_systemVertexColorMaterial, navigationState, noLights))
                    qWarning() << "OpenGLViewerWidget paintGL failed: ViewNavigation axis drawing failed.";
            }
        }
    }
    
}
/// OpenGL 生命周期

void OpenGLViewerWidget::releaseViewerGL()
{
    if (m_releasePerformed)
        return;

    m_releasePerformed = true;
    m_glReady = false;

    QOpenGLContext* viewerContext = context();

    if (viewerContext == 0 || !m_openGLContext.isInitialized())
        return;

    makeCurrent();

    if (QOpenGLContext::currentContext() != viewerContext)
    {
        doneCurrent();
        return;
    }

    QOpenGLFunctions_3_3_Core* gl = m_openGLContext.gl();

    if (gl != 0)
    {
        if (!m_resourceManager.releaseGL(gl))
            qWarning() << "OpenGLViewerWidget releaseViewerGL: some Resources failed to release.";

        m_renderer.release();
    }

    doneCurrent();
}

/// Viewer 系统资源

void OpenGLViewerWidget::buildViewerResources()
{
    /// 系统 Material
    ///
    /// 坐标系和导航器统一使用 VertexColor，
    /// 且 Viewer 系统显示不参与场景光照。

    m_systemVertexColorMaterial = m_materialManager.createMaterial("ViewerSystemVertexColor");

    if (m_systemVertexColorMaterial == 0)
    {
        qWarning() << "OpenGLViewerWidget buildViewerResources failed: unable to create system Material.";
    }
    else
    {
        if (!m_systemVertexColorMaterial->setSurfaceMode(SurfaceMode::VertexColor))
            qWarning() << "OpenGLViewerWidget buildViewerResources failed: unable to configure system Material.";

        m_systemVertexColorMaterial->setLightingEnabled(false);
    }

    /// 系统 Geometry
    ///
    /// CoordinateSystem / ViewNavigation 自己拥有 Geometry。
    /// ResourceManager 只借用这些对象并管理它们的 GPU 生命周期。

    if (m_resourceManager.borrow(&m_coordinateSystem.geometry()) == InvalidResourceId)
        qWarning() << "OpenGLViewerWidget buildViewerResources failed: unable to borrow CoordinateSystem Geometry.";

    if (m_resourceManager.borrow(&m_viewNavigation.faceGeometry()) == InvalidResourceId)
        qWarning() << "OpenGLViewerWidget buildViewerResources failed: unable to borrow ViewNavigation Face Geometry.";

    if (m_resourceManager.borrow(&m_viewNavigation.axisGeometry()) == InvalidResourceId)
        qWarning() << "OpenGLViewerWidget buildViewerResources failed: unable to borrow ViewNavigation Axis Geometry.";

    /// Viewer 系统显示规则

    m_coordinateSystem.setWorldOrigin(QVector3D(0.0f, 0.0f, 0.0f));
    m_coordinateSystem.setPixelLength(90.0f);

    m_viewNavigation.setPixelSize(128);
    m_viewNavigation.setMargin(12);
}

void OpenGLViewerWidget::unregisterViewerResources()
{
    bool contextCurrent = false;
    QOpenGLFunctions_3_3_Core* gl = 0;

    // 正常情况下 releaseViewerGL() 已经释放全部 GPU 状态，
    // 因此 remove() 不需要 OpenGL Functions。
    // 这里仍尝试取得当前 Context，用于异常释放路径的保护。
    if (context() != 0 && m_openGLContext.isInitialized())
    {
        makeCurrent();

        if (QOpenGLContext::currentContext() == context())
        {
            gl = m_openGLContext.gl();
            contextCurrent = true;
        }
    }

    ResourceId id = m_coordinateSystem.geometry().id();

    if (id != InvalidResourceId && !m_resourceManager.remove(id, gl))
        qWarning() << "OpenGLViewerWidget unregisterViewerResources failed: CoordinateSystem Geometry.";

    id = m_viewNavigation.faceGeometry().id();

    if (id != InvalidResourceId && !m_resourceManager.remove(id, gl))
        qWarning() << "OpenGLViewerWidget unregisterViewerResources failed: ViewNavigation Face Geometry.";

    id = m_viewNavigation.axisGeometry().id();

    if (id != InvalidResourceId && !m_resourceManager.remove(id, gl))
        qWarning() << "OpenGLViewerWidget unregisterViewerResources failed: ViewNavigation Axis Geometry.";

    if (contextCurrent)
        doneCurrent();
}

/// 渲染编排

bool OpenGLViewerWidget::buildRenderContext(RenderContext& context) const
{
    const Camera* camera = m_cameraManager.activeCamera();

    if (camera == 0 || width() <= 0 || height() <= 0)
        return false;

    const float aspect = static_cast<float>(width()) / static_cast<float>(height());

    context.view = camera->viewMatrix();
    context.projection = camera->projectionMatrix(aspect);
    context.cameraPosition = camera->position();
    context.cameraForward = camera->forward();
    context.cameraUp = camera->up();
    context.viewportWidth = width();
    context.viewportHeight = height();

    return context.isValid();
}


bool OpenGLViewerWidget::drawItems(Renderer& renderer,
                                   const ItemManager& itemManager,
                                   const RenderContext& context,
                                   const std::vector<const Light*>& lights)
{
    struct PartEntry
    {
        PartEntry() : item(0), part(0){}
        const RenderItem* item;
        const RenderPart* part;
    };

    const int itemCount = static_cast<int>(itemManager.count());
    std::vector<PartEntry> opaqueParts;
    std::vector<PartEntry> transparentParts;

    /// 第一阶段：收集普通 Part。
    ///
    /// Opaque 保持原 Item / Part 顺序。
    /// Transparent 不再做 CPU 前后排序，而是统一进入 Weighted Blended OIT。
    for (int itemIndex = 0; itemIndex < itemCount; ++itemIndex)
    {
        const RenderItem* item = itemManager.itemAt(itemIndex);

        if (item == 0)
        {
            qWarning() << "OpenGLViewerWidget drawItems failed: null RenderItem:"
                       << "Index=" << itemIndex;
            return false;
        }

        if (!item->isVisible()) continue;

        for (int partIndex = 0; partIndex < item->partCount(); ++partIndex)
        {
            const RenderPart* part = item->partAt(partIndex);

            if (part == 0)
            {
                qWarning() << "OpenGLViewerWidget drawItems failed: null RenderPart:"
                           << "Item=" << item->name()
                           << "Index=" << partIndex;
                return false;
            }

            if (part->geometry() == 0) continue;

            PartEntry entry;
            entry.item = item;
            entry.part = part;

            if (part->isTransparent(*item))
                transparentParts.push_back(entry);
            else
                opaqueParts.push_back(entry);
        }
    }

    /// 第二阶段：先绘制全部不透明 Part。
    for (std::size_t index = 0; index < opaqueParts.size(); ++index)
    {
        const PartEntry& entry = opaqueParts[index];

        if (!entry.part->draw(renderer, *entry.item, context, lights))
        {
            qWarning() << "OpenGLViewerWidget drawItems failed while drawing opaque RenderPart:"
                       << "Item=" << entry.item->name()
                       << "PartId=" << static_cast<qulonglong>(entry.part->id());
            return false;
        }
    }

    /// 第三阶段：Weighted Blended OIT。
    ///
    /// OpenGL 3.3 下采用两个透明 Geometry Pass：
    /// 1. Accumulation：加法累积颜色和权重；
    /// 2. Revealage：乘法累积透过率；
    /// 最后一次全屏 Composite 合成回主 Framebuffer。
    if (!transparentParts.empty())
    {
        if (!renderer.beginWeightedOIT(context))
        {
            /// OIT 初始化失败时保留普通 Alpha Blend 兜底，不影响基本显示。
            for (std::size_t index = 0; index < transparentParts.size(); ++index)
            {
                const PartEntry& entry = transparentParts[index];

                if (!entry.part->draw(renderer, *entry.item, context, lights))
                    return false;
            }
        }
        else
        {
            if (!renderer.beginWeightedOITAccumulation())
            {
                renderer.cancelWeightedOIT();
                return false;
            }

            for (std::size_t index = 0; index < transparentParts.size(); ++index)
            {
                const PartEntry& entry = transparentParts[index];

                if (!entry.part->draw(renderer, *entry.item, context, lights))
                {
                    renderer.cancelWeightedOIT();
                    return false;
                }
            }

            if (!renderer.beginWeightedOITRevealage())
            {
                renderer.cancelWeightedOIT();
                return false;
            }

            for (std::size_t index = 0; index < transparentParts.size(); ++index)
            {
                const PartEntry& entry = transparentParts[index];

                if (!entry.part->draw(renderer, *entry.item, context, lights))
                {
                    renderer.cancelWeightedOIT();
                    return false;
                }
            }

            if (!renderer.compositeWeightedOIT())
            {
                renderer.cancelWeightedOIT();
                return false;
            }
        }
    }

    /// 第四阶段：绘制全部持久化 Screen Label。
    for (int itemIndex = 0; itemIndex < itemCount; ++itemIndex)
    {
        const RenderItem* item = itemManager.itemAt(itemIndex);

        if (item == 0)
            return false;

        if (!item->drawLabels(renderer, context, lights))
        {
            qWarning() << "OpenGLViewerWidget drawItems failed while drawing Item Labels:"
                       << item->name();
            return false;
        }
    }

    return true;
}

/// Camera

bool OpenGLViewerWidget::fitItemsToView(float margin)
{
    if (width() <= 0 || height() <= 0)
        return false;

    AxisAlignedBoundingBox bounds;

    if (!m_itemManager.worldBounds(bounds, true))
        return false;

    if (!m_cameraManager.fitBounds(bounds, width(), height(), margin))
        return false;

    update();
    return true;
}

void OpenGLViewerWidget::toggleProjection()
{
    Camera* camera = m_cameraManager.activeCamera();

    if (camera == 0)
        return;

    bool changed = false;

    if (camera->projectionType() == ProjectionType::Perspective)
        changed = camera->setParallel(10.0f, camera->nearPlane(), camera->farPlane());
    else
        changed = camera->setPerspective(45.0f, camera->nearPlane(), camera->farPlane());

    if (!changed)
        return;

    // 有 Item 时重新 Fit，保证投影切换后模型仍完整可见。
    // 没有 Item 时只刷新当前 Camera。
    if (!fitItemsToView())
        update();
}

bool OpenGLViewerWidget::scenePointAtWorld(const QPointF& scene, QVector3D& world) const
{
    if (width() <= 0 || height() <= 0)
        return false;
    if(scene.x() >= 0.0 &&scene.x() < width() &&scene.y() >= 0.0 &&scene.y() < height())
        return scenePointAtWorldFromDepth(scene, world);
    return scenePointAtWorldFromRay(scene, world);
}
bool OpenGLViewerWidget::worldPointAtScene(const QVector3D& world, QPointF& scene) const
{
    return projectWorldPointToScene(world,scene);
}




bool OpenGLViewerWidget::scenePointAtWorldFromDepth(const QPointF& scene, QVector3D& world) const
{
    if (!m_sceneDepthValid || m_sceneDepthWidth <= 0 || m_sceneDepthHeight <= 0)
        return false;

    if (width() <= 0 || height() <= 0)
        return false;

    if (scene.x() < 0.0 || scene.x() >= width() || scene.y() < 0.0 || scene.y() >= height())
        return false;

    const int pixelX = static_cast<int>((scene.x() + 0.5) * m_sceneDepthWidth / width());
    const int pixelY = m_sceneDepthHeight - 1 - static_cast<int>((scene.y() + 0.5) * m_sceneDepthHeight / height());

    if (pixelX < 0 || pixelX >= m_sceneDepthWidth || pixelY < 0 || pixelY >= m_sceneDepthHeight)
        return false;

    const float depth = m_sceneDepthBuffer[pixelY * m_sceneDepthWidth + pixelX];

    if (depth >= 1.0f - 1.0e-7f)
        return false;

    const float ndcX = (static_cast<float>(pixelX) + 0.5f) / static_cast<float>(m_sceneDepthWidth) * 2.0f - 1.0f;
    const float ndcY = (static_cast<float>(pixelY) + 0.5f) / static_cast<float>(m_sceneDepthHeight) * 2.0f - 1.0f;
    const float ndcZ = depth * 2.0f - 1.0f;

    const QVector4D worldPoint = m_sceneDepthInverseViewProjection * QVector4D(ndcX, ndcY, ndcZ, 1.0f);

    if (qAbs(worldPoint.w()) <= 1.0e-8f)
        return false;

    world = QVector3D(worldPoint.x() / worldPoint.w(), worldPoint.y() / worldPoint.w(), worldPoint.z() / worldPoint.w());

    return true;
}
bool OpenGLViewerWidget::scenePointAtWorldFromRay(const QPointF& scene, QVector3D& world) const
{
    const Camera* camera = m_cameraManager.activeCamera();
    if (camera == 0 || width() <= 0 || height() <= 0)
        return false;
    //提取射线
    QVector3D rayOrigin;
    QVector3D rayDirection;
    if (!camera->screenPointToRay(scene.x(), scene.y(), width(), height(), rayOrigin, rayDirection))
        return false;

    bool found = false;
    float nearestDistance = 0.0f;
    QVector3D nearestPoint;

    const int itemCount = static_cast<int>(m_itemManager.count());
    //遍历Item检查是否存在Item与光线相交
    for (int index = 0; index < itemCount; ++index)
    {
        const RenderItem* item = m_itemManager.itemAt(index);
        if (item == 0 || !item->isVisible())
            continue;
        RenderItemRayHit hit;
        if (!item->raycast(rayOrigin, rayDirection, hit))
            continue;
        //找到最小的相交点
        if (!found || hit.distance < nearestDistance)
        {
            found = true;
            nearestDistance = hit.distance;
            nearestPoint = hit.position;
        }
    }

    if (!found)
        return false;

    world = nearestPoint;
    return true;
}
bool OpenGLViewerWidget::projectWorldPointToScene(const QVector3D& world, QPointF& scene) const
{
    if (width() <= 0 || height() <= 0)
        return false;

    const Camera* camera = m_cameraManager.activeCamera();

    if (camera == 0)
        return false;

    const float aspect = static_cast<float>(width()) / static_cast<float>(height());
    const QVector4D clip = camera->projectionMatrix(aspect) * camera->viewMatrix() * QVector4D(world, 1.0f);

    if (qAbs(clip.w()) <= 1.0e-8f)
        return false;

    if (camera->projectionType() == ProjectionType::Perspective && clip.w() <= 0.0f)
        return false;

    const double ndcX = static_cast<double>(clip.x() / clip.w());
    const double ndcY = static_cast<double>(clip.y() / clip.w());

    scene.setX((ndcX * 0.5 + 0.5) * width());
    scene.setY((0.5 - ndcY * 0.5) * height());

    return true;
}

bool OpenGLViewerWidget::setStandardView(ViewNavigationFace face)
{
    if (width() <= 0 || height() <= 0)
        return false;

    QVector3D forward;
    QVector3D up;

    if (!m_viewNavigation.viewDirection(face, forward, up))
        return false;

    AxisAlignedBoundingBox bounds;
    if(m_itemManager.worldBounds(bounds))
    {
        m_cameraManager.setViewBounds(bounds);
    }
    else
    {
        // 使用世界原点为中心的默认包围盒。
        bounds.set(QVector3D(-1.0f, -1.0f, -1.0f), QVector3D(1.0f, 1.0f, 1.0f));
    }

    if (!m_cameraManager.setViewDirection(bounds.center(), forward, up))
        return false;

    if (!m_cameraManager.fitBounds(bounds, width(), height()))
        return false;

    update();
    return true;
}

bool OpenGLViewerWidget::cacheSceneDepth(const RenderContext& context)
{
    QOpenGLFunctions_3_3_Core* gl = m_openGLContext.gl();

    if (gl == 0 || !context.isValid())
    {
        clearSceneDepthCache();
        return false;
    }

    bool invertible = false;
    const QMatrix4x4 inverse = (context.projection * context.view).inverted(&invertible);

    if (!invertible)
    {
        clearSceneDepthCache();
        return false;
    }

    GLint viewport[4] = { 0, 0, 0, 0 };
    gl->glGetIntegerv(GL_VIEWPORT, viewport);

    if (viewport[2] <= 0 || viewport[3] <= 0)
    {
        clearSceneDepthCache();
        return false;
    }

    m_sceneDepthWidth = viewport[2];
    m_sceneDepthHeight = viewport[3];
    m_sceneDepthInverseViewProjection = inverse;
    m_sceneDepthBuffer.resize(m_sceneDepthWidth * m_sceneDepthHeight);

    gl->glReadPixels(viewport[0], viewport[1], m_sceneDepthWidth, m_sceneDepthHeight, GL_DEPTH_COMPONENT, GL_FLOAT, m_sceneDepthBuffer.data());

    m_sceneDepthValid = true;

    return true;
}
void OpenGLViewerWidget::clearSceneDepthCache()
{
    m_sceneDepthBuffer.clear();
    m_sceneDepthInverseViewProjection.setToIdentity();
    m_sceneDepthWidth = 0;
    m_sceneDepthHeight = 0;
    m_sceneDepthValid = false;
}

/// Mouse

void OpenGLViewerWidget::mousePressEvent(QMouseEvent* event)
{
    if (m_toolManager.mousePressEvent(this, event))
    {
        event->accept();
        update();
        return;
    }

    QOpenGLWidget::mousePressEvent(event);
}

void OpenGLViewerWidget::mouseMoveEvent(QMouseEvent* event)
{
    if (m_toolManager.mouseMoveEvent(this, event))
    {
        event->accept();
        update();
        return;
    }

    QOpenGLWidget::mouseMoveEvent(event);
}

void OpenGLViewerWidget::mouseReleaseEvent(QMouseEvent* event)
{
    ViewerTool* tool = m_toolManager.activeTool();
    const bool toolHandled = m_toolManager.mouseReleaseEvent(this, event);

    if (tool != 0 && tool->isFinished())
        emit toolFinished();

    if (toolHandled)
    {
        event->accept();
        update();
        return;
    }

    QOpenGLWidget::mouseReleaseEvent(event);
}

void OpenGLViewerWidget::wheelEvent(QWheelEvent* event)
{
    if (m_toolManager.wheelEvent(this, event))
    {
        event->accept();
        update();
        return;
    }

    QOpenGLWidget::wheelEvent(event);
}

/// Keyboard

void OpenGLViewerWidget::keyPressEvent(QKeyEvent* event)
{
    ViewerTool* tool = m_toolManager.activeTool();
    const bool toolHandled = m_toolManager.keyPressEvent(this, event);

    if (tool != 0 && tool->isFinished())
        emit toolFinished();

    if (toolHandled)
    {
        event->accept();
        update();
        return;
    }

    if (handleKeyPress(event))
    {
        event->accept();
        return;
    }

    if (event->key() == Qt::Key_F)
    {
        fitItemsToView();
        event->accept();
        return;
    }

    if (event->key() == Qt::Key_P)
    {
        toggleProjection();
        event->accept();
        return;
    }

    QOpenGLWidget::keyPressEvent(event);
}
/// Context Menu

void OpenGLViewerWidget::contextMenuEvent(QContextMenuEvent* event)
{
    QMenu menu(this);
    /// 子类扩展
    populateContextMenu(menu);
    if (!menu.isEmpty())
        menu.exec(event->globalPos());
    event->accept();
}

void OpenGLViewerWidget::resizeEvent(QResizeEvent* event)
{
    QOpenGLWidget::resizeEvent(event);

    if (m_viewportOverlay != 0)
    {
        m_viewportOverlay->setGeometry(rect());
        m_viewportOverlay->raise();
    }
}


/// 子类扩展

void OpenGLViewerWidget::populateContextMenu(QMenu& menu)
{
    Q_UNUSED(menu);
}

bool OpenGLViewerWidget::handleKeyPress(QKeyEvent* event)
{
    Q_UNUSED(event);
    return false;
}
void OpenGLViewerWidget::drawViewportOverlay(QPainter& painter)
{
    m_toolManager.drawOverlay(this, painter);
}
void OpenGLViewerWidget::updateViewportOverlay()
{
    if (m_viewportOverlay != 0)
        m_viewportOverlay->update();
}