#ifndef APP_MAINWINDOW_H
#define APP_MAINWINDOW_H

#include <QMainWindow>

namespace Ui
{
class MainWindow;
}

namespace MyBRep
{
namespace Display
{
class BRepViewerWidget;
}
}

class ViewerTool;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = 0);
    ~MainWindow() override;

    MyBRep::Display::BRepViewerWidget* viewer();
    const MyBRep::Display::BRepViewerWidget* viewer() const;

private:
    void activateMeasurementTool(ViewerTool* tool, bool checked);
    void clearMeasurementToolButtons();

private:
    Ui::MainWindow* m_ui;

    ViewerTool* m_length2DTool;
    ViewerTool* m_length3DTool;
    ViewerTool* m_angle2DTool;
    ViewerTool* m_angle3DTool;
};

#endif // APP_MAINWINDOW_H