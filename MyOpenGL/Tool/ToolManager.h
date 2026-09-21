#ifndef MYOPENGL_TOOL_TOOLMANAGER_H
#define MYOPENGL_TOOL_TOOLMANAGER_H

#include "MyOpenGL/Tool/Navigation/NavigationTool.h"

class QKeyEvent;
class QMouseEvent;
class QPainter;
class QWheelEvent;
class OpenGLViewerWidget;
struct RenderContext;
class Renderer;
class ViewerTool;

/// Viewer工具管理器。
/// ToolManager拥有内建NavigationTool；外部Active ViewerTool只借用，不负责生命周期。
class ToolManager
{
public:
    ToolManager();
    ~ToolManager();

    void activate(OpenGLViewerWidget* viewer);
    void deactivate(OpenGLViewerWidget* viewer);

    NavigationTool& navigationTool();
    const NavigationTool& navigationTool() const;

    ViewerTool* activeTool();
    const ViewerTool* activeTool() const;

    void setActiveTool(OpenGLViewerWidget* viewer, ViewerTool* tool);
    void clearActiveTool(OpenGLViewerWidget* viewer);
    void resetActiveTool();

    bool mousePressEvent(OpenGLViewerWidget* viewer, QMouseEvent* event);
    bool mouseMoveEvent(OpenGLViewerWidget* viewer, QMouseEvent* event);
    bool mouseReleaseEvent(OpenGLViewerWidget* viewer, QMouseEvent* event);
    bool wheelEvent(OpenGLViewerWidget* viewer, QWheelEvent* event);
    bool keyPressEvent(OpenGLViewerWidget* viewer, QKeyEvent* event);

    bool drawSceneFront(OpenGLViewerWidget* viewer, Renderer& renderer, const RenderContext& context) const;
    void drawOverlay(OpenGLViewerWidget* viewer, QPainter& painter) const;

private:
    NavigationTool m_navigationTool;
    ViewerTool* m_activeTool;
};

#endif // MYOPENGL_TOOL_TOOLMANAGER_H