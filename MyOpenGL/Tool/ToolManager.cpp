#include "ToolManager.h"

#include "MyOpenGL/Tool/ViewerTool.h"

ToolManager::ToolManager()
    : m_activeTool(0)
{
}

ToolManager::~ToolManager()
{
}

ViewerTool* ToolManager::activeTool()
{
    return m_activeTool;
}

const ViewerTool* ToolManager::activeTool() const
{
    return m_activeTool;
}

void ToolManager::setActiveTool(OpenGLViewerWidget* viewer, ViewerTool* tool)
{
    if (m_activeTool == tool)
    {
        if (m_activeTool != 0) m_activeTool->reset();
        return;
    }

    if (m_activeTool != 0)
    {
        m_activeTool->deactivate(viewer);
        m_activeTool->reset();
    }

    m_activeTool = tool;

    if (m_activeTool != 0)
    {
        m_activeTool->reset();
        m_activeTool->activate(viewer);
    }
}

void ToolManager::clearActiveTool(OpenGLViewerWidget* viewer)
{
    setActiveTool(viewer, 0);
}

void ToolManager::resetActiveTool()
{
    if (m_activeTool != 0) m_activeTool->reset();
}

bool ToolManager::mousePressEvent(OpenGLViewerWidget* viewer, QMouseEvent* event)
{
    return m_activeTool != 0 && m_activeTool->mousePressEvent(viewer, event);
}

bool ToolManager::mouseMoveEvent(OpenGLViewerWidget* viewer, QMouseEvent* event)
{
    return m_activeTool != 0 && m_activeTool->mouseMoveEvent(viewer, event);
}

bool ToolManager::mouseReleaseEvent(OpenGLViewerWidget* viewer, QMouseEvent* event)
{
    return m_activeTool != 0 && m_activeTool->mouseReleaseEvent(viewer, event);
}

bool ToolManager::keyPressEvent(OpenGLViewerWidget* viewer, QKeyEvent* event)
{
    return m_activeTool != 0 && m_activeTool->keyPressEvent(viewer, event);
}

void ToolManager::drawOverlay(OpenGLViewerWidget* viewer, QPainter& painter) const
{
    if (m_activeTool != 0) m_activeTool->drawOverlay(viewer, painter);
}