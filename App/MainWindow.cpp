#include "MainWindow.h"

#include <QPushButton>
#include <QToolButton>

#include "ui_MainWindow.h"

#include "MyBRepOpenGL/Viewer/BRepViewerWidget.h"
#include "MyOpenGL/Tool/Measurement/Angle2DMeasurement.h"
#include "MyOpenGL/Tool/Measurement/Angle3DMeasurement.h"
#include "MyOpenGL/Tool/Measurement/Length2DMeasurement.h"
#include "MyOpenGL/Tool/Measurement/Length3DMeasurement.h"

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , m_ui(new Ui::MainWindow)
    , m_length2DTool(new Length2DMeasurement)
    , m_length3DTool(new Length3DMeasurement)
    , m_angle2DTool(new Angle2DMeasurement)
    , m_angle3DTool(new Angle3DMeasurement)
{
    m_ui->setupUi(this);

    connect(m_ui->buttonMeasureLength2D, &QToolButton::toggled, this, [this](bool checked)
    {
        activateMeasurementTool(m_length2DTool, checked);
    });

    connect(m_ui->buttonMeasureLength3D, &QToolButton::toggled, this, [this](bool checked)
    {
        activateMeasurementTool(m_length3DTool, checked);
    });

    connect(m_ui->buttonMeasureAngle2D, &QToolButton::toggled, this, [this](bool checked)
    {
        activateMeasurementTool(m_angle2DTool, checked);
    });

    connect(m_ui->buttonMeasureAngle3D, &QToolButton::toggled, this, [this](bool checked)
    {
        activateMeasurementTool(m_angle3DTool, checked);
    });

    connect(m_ui->buttonMeasureClose, &QPushButton::clicked, this, [this]()
    {
        clearMeasurementToolButtons();
        viewer()->setActiveTool(0);
        viewer()->setFocus();
    });
}

MainWindow::~MainWindow()
{
    if (viewer() != 0)
        viewer()->setActiveTool(0);

    delete m_length2DTool;
    delete m_length3DTool;
    delete m_angle2DTool;
    delete m_angle3DTool;

    delete m_ui;
}

MyBRep::Display::BRepViewerWidget* MainWindow::viewer()
{
    return m_ui->brepViewerWidget;
}

const MyBRep::Display::BRepViewerWidget* MainWindow::viewer() const
{
    return m_ui->brepViewerWidget;
}

void MainWindow::activateMeasurementTool(ViewerTool* tool, bool checked)
{
    if (tool == 0 || viewer() == 0)
        return;

    if (!checked)
    {
        if (viewer()->activeTool() == tool)
            viewer()->setActiveTool(0);

        viewer()->setFocus();
        return;
    }

    m_ui->buttonMeasureLength2D->setChecked(tool == m_length2DTool);
    m_ui->buttonMeasureLength3D->setChecked(tool == m_length3DTool);
    m_ui->buttonMeasureAngle2D->setChecked(tool == m_angle2DTool);
    m_ui->buttonMeasureAngle3D->setChecked(tool == m_angle3DTool);

    viewer()->setActiveTool(tool);
    viewer()->setFocus();
}

void MainWindow::clearMeasurementToolButtons()
{
    m_ui->buttonMeasureLength2D->setChecked(false);
    m_ui->buttonMeasureLength3D->setChecked(false);
    m_ui->buttonMeasureAngle2D->setChecked(false);
    m_ui->buttonMeasureAngle3D->setChecked(false);
}
