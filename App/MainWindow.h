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

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = 0);
    ~MainWindow() override;

    MyBRep::Display::BRepViewerWidget* viewer();
    const MyBRep::Display::BRepViewerWidget* viewer() const;

private:
    Ui::MainWindow* m_ui;
};

#endif // APP_MAINWINDOW_H