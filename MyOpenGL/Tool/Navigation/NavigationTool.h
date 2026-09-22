#ifndef MYOPENGL_TOOL_NAVIGATION_NAVIGATIONTOOL_H
#define MYOPENGL_TOOL_NAVIGATION_NAVIGATIONTOOL_H

#include <QPointF>
#include <QTimer>
#include <QVector3D>

#include "MyOpenGL/Resource/BufferGeometry.h"
#include "MyOpenGL/Tool/ViewerTool.h"

class Material;
class OpenGLViewerWidget;
struct RenderContext;
class Renderer;
struct RenderState;

/// Viewer基础导航工具。
/// 负责ViewNavigation点击、Orbit、Pan、Wheel Zoom以及导航锚点显示。
/// NavigationTool拥有自身显示资源，但不拥有Viewer。
class NavigationTool : public ViewerTool
{
public:
    NavigationTool();
    ~NavigationTool() override;

    void activate(OpenGLViewerWidget* viewer) override;
    void deactivate(OpenGLViewerWidget* viewer) override;
    void reset() override;

    bool mousePressEvent(OpenGLViewerWidget* viewer, QMouseEvent* event) override;
    bool mouseMoveEvent(OpenGLViewerWidget* viewer, QMouseEvent* event) override;
    bool mouseReleaseEvent(OpenGLViewerWidget* viewer, QMouseEvent* event) override;
    bool wheelEvent(OpenGLViewerWidget* viewer, QWheelEvent* event) override;

    bool drawSceneFront(Renderer& renderer, const RenderContext& context) const;

private:
    bool createResources(OpenGLViewerWidget* viewer);
    void releaseResources(OpenGLViewerWidget* viewer);
    bool buildAnchorGeometry();
    bool buildAnchorRenderState(const RenderContext& context, RenderState& state) const;

    bool navigationAnchor(OpenGLViewerWidget* viewer, QVector3D& anchor) const;
    QVector3D screenPointToZoomAnchor(OpenGLViewerWidget* viewer, const QPointF& position) const;
    QVector3D screenPointToAnchor(OpenGLViewerWidget* viewer, const QPointF& position) const;

private:
    OpenGLViewerWidget* m_viewer;                    // 非拥有Viewer指针，用于Timer刷新。
    QPointF m_lastMousePosition;                     // 上一次鼠标位置。
    QVector3D m_navigationAnchor;                    // 当前Orbit/Pan/Zoom锚点。
    bool m_hasNavigationAnchor;                      // 当前是否存在拖动导航锚点。

    BufferGeometry m_anchorGeometry;                 // 导航十字Geometry。
    Material* m_anchorMaterial;                      // 导航十字Material，由Viewer MaterialManager拥有。
    bool m_anchorVisible;                            // 是否显示导航十字。
    int m_anchorPixelSize;                           // 导航十字固定屏幕Pixel尺寸。
    QTimer m_anchorHideTimer;                        // Wheel Zoom后延迟隐藏导航十字。
    bool m_resourcesReady;                          // Tool显示资源是否已经注册。
};

#endif // MYOPENGL_TOOL_NAVIGATION_NAVIGATIONTOOL_H
