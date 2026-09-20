#ifndef MYOPENGL_TOOL_TOOLMANAGER_H
#define MYOPENGL_TOOL_TOOLMANAGER_H

class QKeyEvent;
class QMouseEvent;
class QPainter;
class OpenGLViewerWidget;
class ViewerTool;

/// Viewer当前交互工具管理器。
/// ToolManager不拥有Tool，只管理当前激活工具。
class ToolManager
{
public:
    ToolManager();
    ~ToolManager();

    ViewerTool* activeTool();
    const ViewerTool* activeTool() const;

    void setActiveTool(OpenGLViewerWidget* viewer, ViewerTool* tool);
    void clearActiveTool(OpenGLViewerWidget* viewer);
    void resetActiveTool();

    bool mousePressEvent(OpenGLViewerWidget* viewer, QMouseEvent* event);
    bool mouseMoveEvent(OpenGLViewerWidget* viewer, QMouseEvent* event);
    bool mouseReleaseEvent(OpenGLViewerWidget* viewer, QMouseEvent* event);
    bool keyPressEvent(OpenGLViewerWidget* viewer, QKeyEvent* event);

    void drawOverlay(OpenGLViewerWidget* viewer, QPainter& painter) const;

private:
    ViewerTool* m_activeTool; // 当前激活工具，不拥有。
};

#endif // MYOPENGL_TOOL_TOOLMANAGER_H