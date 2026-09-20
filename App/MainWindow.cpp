#include "MainWindow.h"

#include "ui_MainWindow.h"

#include "MyBRepOpenGL/Viewer/BRepViewerWidget.h"

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , m_ui(new Ui::MainWindow)
{
    m_ui->setupUi(this);
}

MainWindow::~MainWindow()
{
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