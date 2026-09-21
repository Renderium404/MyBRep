#ifndef MYOPENGL_TOOL_VIEWERTOOL_H
#define MYOPENGL_TOOL_VIEWERTOOL_H

class QKeyEvent;
class QMouseEvent;
class QPainter;
class QWheelEvent;
class OpenGLViewerWidget;

/// Viewer交互工具基类。
/// ViewerTool不拥有Viewer，也不负责自身生命周期。
class ViewerTool
{
public:
    ViewerTool();
    virtual ~ViewerTool();

    /// 生命周期
    virtual void activate(OpenGLViewerWidget* viewer);
    virtual void deactivate(OpenGLViewerWidget* viewer);
    virtual void reset() = 0;
    virtual bool isFinished() const;

    /// 输入
    virtual bool mousePressEvent(OpenGLViewerWidget* viewer, QMouseEvent* event);
    virtual bool mouseMoveEvent(OpenGLViewerWidget* viewer, QMouseEvent* event);
    virtual bool mouseReleaseEvent(OpenGLViewerWidget* viewer, QMouseEvent* event);
    virtual bool wheelEvent(OpenGLViewerWidget* viewer, QWheelEvent* event);
    virtual bool keyPressEvent(OpenGLViewerWidget* viewer, QKeyEvent* event);

    /// Overlay
    virtual void drawOverlay(OpenGLViewerWidget* viewer, QPainter& painter) const;
};

#endif // MYOPENGL_TOOL_VIEWERTOOL_H