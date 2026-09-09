#include <QApplication>
#include <QDebug>
#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>
#include <QVector4D>
#include <QWidget>

#include "GeneratedSurfaceSolidFixtures.h"

#include "MyBRepOpenGL/Display/BRepDisplayObject.h"
#include "MyBRepOpenGL/Display/BRepDisplayStyle.h"
#include "MyBRepOpenGL/Viewer/BRepViewerWidget.h"

namespace
{

QWidget* createViewerPane(const QString& title, MyBRep::Display::BRepViewerWidget*& viewer)
{
    QWidget* pane = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(pane);
    layout->setContentsMargins(6, 6, 6, 6);
    layout->setSpacing(6);

    QLabel* titleLabel = new QLabel(title, pane);
    titleLabel->setAlignment(Qt::AlignCenter);

    QLabel* hintLabel = new QLabel("Mouse: Orbit / Pan / Zoom    F: Fit    P: Projection", pane);
    hintLabel->setAlignment(Qt::AlignCenter);

    viewer = new MyBRep::Display::BRepViewerWidget(pane);
    viewer->setMinimumSize(560, 520);

    layout->addWidget(titleLabel);
    layout->addWidget(viewer, 1);
    layout->addWidget(hintLabel);

    return pane;
}

bool addExtrusionSolid(MyBRep::Display::BRepViewerWidget& viewer)
{
    const GeneratedSurfaceSolidFixtures::ExtrusionSolidFixture fixture =
        GeneratedSurfaceSolidFixtures::createExtrusionSolid(3.0, -2.0, 2.0);

    MyBRep::Display::BRepDisplayStyle style;
    style.surfaceColor = QVector4D(0.28f, 0.55f, 0.88f, 1.0f);
    style.wireColor = QVector4D(0.08f, 0.08f, 0.10f, 1.0f);
    style.surface.extrudedMeshing.boundaryChordTolerance = 0.03;
    style.surface.extrudedMeshing.surfaceChordTolerance = 0.02;
    style.surface.extrudedMeshing.minimumBoundarySubdivisionDepth = 1;

    const MyBRep::Display::BRepDisplayId id =
        viewer.addSolid(fixture.solid, "ExtrusionSolid", style);

    return id != MyBRep::Display::InvalidBRepDisplayId;
}

bool addRevolutionSolid(MyBRep::Display::BRepViewerWidget& viewer)
{
    const GeneratedSurfaceSolidFixtures::RevolutionSolidFixture fixture =
        GeneratedSurfaceSolidFixtures::createRevolutionTorusSolid(5.0, 1.5);

    MyBRep::Display::BRepDisplayStyle style;
    style.surfaceColor = QVector4D(0.92f, 0.55f, 0.22f, 1.0f);
    style.wireColor = QVector4D(0.08f, 0.08f, 0.10f, 1.0f);
    style.surface.revolvedMeshing.boundaryChordTolerance = 0.04;
    style.surface.revolvedMeshing.surfaceChordTolerance = 0.03;
    style.surface.revolvedMeshing.minimumBoundarySubdivisionDepth = 1;

    const MyBRep::Display::BRepDisplayId id =
        viewer.addSolid(fixture.solid, "RevolutionTorusSolid", style);

    return id != MyBRep::Display::InvalidBRepDisplayId;
}

}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);

    QWidget window;
    window.setWindowTitle("MyBRep Generated Surface Solids");
    window.resize(1280, 720);

    QHBoxLayout* mainLayout = new QHBoxLayout(&window);
    mainLayout->setContentsMargins(8, 8, 8, 8);
    mainLayout->setSpacing(8);

    MyBRep::Display::BRepViewerWidget* extrusionViewer = 0;
    MyBRep::Display::BRepViewerWidget* revolutionViewer = 0;

    QWidget* extrusionPane = createViewerPane("Extrusion Solid", extrusionViewer);
    QWidget* revolutionPane = createViewerPane("Revolution Torus Solid", revolutionViewer);

    mainLayout->addWidget(extrusionPane, 1);
    mainLayout->addWidget(revolutionPane, 1);

    if (!addExtrusionSolid(*extrusionViewer))
    {
        qCritical() << "Unable to add Extrusion Solid to viewer.";
        return 1;
    }

    if (!addRevolutionSolid(*revolutionViewer))
    {
        qCritical() << "Unable to add Revolution Solid to viewer.";
        return 1;
    }

    window.show();

    // 先让Qt完成窗口和OpenGL Widget初始化，再按实际Viewport尺寸重新适配两个模型。
    application.processEvents();
    extrusionViewer->fitItemsToView();
    revolutionViewer->fitItemsToView();

    return application.exec();
}
