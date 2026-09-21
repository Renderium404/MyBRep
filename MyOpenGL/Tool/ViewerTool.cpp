#include "ViewerTool.h"

#include <QKeyEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>

ViewerTool::ViewerTool()
{
}

ViewerTool::~ViewerTool()
{
}

void ViewerTool::activate(OpenGLViewerWidget* viewer)
{
    Q_UNUSED(viewer);
}

void ViewerTool::deactivate(OpenGLViewerWidget* viewer)
{
    Q_UNUSED(viewer);
}

bool ViewerTool::isFinished() const
{
    return false;
}

bool ViewerTool::mousePressEvent(OpenGLViewerWidget* viewer, QMouseEvent* event)
{
    Q_UNUSED(viewer);
    Q_UNUSED(event);
    return false;
}

bool ViewerTool::mouseMoveEvent(OpenGLViewerWidget* viewer, QMouseEvent* event)
{
    Q_UNUSED(viewer);
    Q_UNUSED(event);
    return false;
}

bool ViewerTool::mouseReleaseEvent(OpenGLViewerWidget* viewer, QMouseEvent* event)
{
    Q_UNUSED(viewer);
    Q_UNUSED(event);
    return false;
}

bool ViewerTool::wheelEvent(OpenGLViewerWidget* viewer, QWheelEvent* event)
{
    Q_UNUSED(viewer);
    Q_UNUSED(event);
    return false;
}

bool ViewerTool::keyPressEvent(OpenGLViewerWidget* viewer, QKeyEvent* event)
{
    Q_UNUSED(viewer);
    Q_UNUSED(event);
    return false;
}

void ViewerTool::drawOverlay(OpenGLViewerWidget* viewer, QPainter& painter) const
{
    Q_UNUSED(viewer);
    Q_UNUSED(painter);
}