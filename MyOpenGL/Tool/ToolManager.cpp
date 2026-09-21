#include "ToolManager.h"

#include <QKeyEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>

#include "MyOpenGL/Tool/ViewerTool.h"

ToolManager::ToolManager()
    : m_activeTool(0)
{
}

ToolManager::~ToolManager()
{
}

void ToolManager::activate(OpenGLViewerWidget* viewer)
{
    m_navigationTool.activate(viewer);
}

void ToolManager::deactivate(OpenGLViewerWidget* viewer)
{
    if (m_activeTool != 0)
    {
        m_activeTool->deactivate(viewer);
        m_activeTool->reset();
        m_activeTool = 0;
    }

    m_navigationTool.deactivate(viewer);
}

NavigationTool& ToolManager::navigationTool()
{
    return m_navigationTool;
}

const NavigationTool& ToolManager::navigationTool() const
{
    return m_navigationTool;
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
    if (tool == &m_navigationTool)
        tool = 0;

    if (m_activeTool == tool)
    {
        if (m_activeTool != 0)
            m_activeTool->reset();
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
    if (m_activeTool != 0)
        m_activeTool->reset();
}

bool ToolManager::mousePressEvent(OpenGLViewerWidget* viewer, QMouseEvent* event)
{
    if (m_activeTool != 0 && m_activeTool->mousePressEvent(viewer, event))
        return true;

    return m_navigationTool.mousePressEvent(viewer, event);
}

bool ToolManager::mouseMoveEvent(OpenGLViewerWidget* viewer, QMouseEvent* event)
{
    if (m_activeTool != 0 && m_activeTool->mouseMoveEvent(viewer, event))
        return true;

    return m_navigationTool.mouseMoveEvent(viewer, event);
}

bool ToolManager::mouseReleaseEvent(OpenGLViewerWidget* viewer, QMouseEvent* event)
{
    if (m_activeTool != 0 && m_activeTool->mouseReleaseEvent(viewer, event))
        return true;

    return m_navigationTool.mouseReleaseEvent(viewer, event);
}

bool ToolManager::wheelEvent(OpenGLViewerWidget* viewer, QWheelEvent* event)
{
    if (m_activeTool != 0 && m_activeTool->wheelEvent(viewer, event))
        return true;

    return m_navigationTool.wheelEvent(viewer, event);
}

bool ToolManager::keyPressEvent(OpenGLViewerWidget* viewer, QKeyEvent* event)
{
    return m_activeTool != 0 && m_activeTool->keyPressEvent(viewer, event);
}

bool ToolManager::drawSceneFront(OpenGLViewerWidget* viewer, Renderer& renderer, const RenderContext& context) const
{
    Q_UNUSED(viewer);
    return m_navigationTool.drawSceneFront(renderer, context);
}

void ToolManager::drawOverlay(OpenGLViewerWidget* viewer, QPainter& painter) const
{
    if (m_activeTool != 0)
        m_activeTool->drawOverlay(viewer, painter);
}