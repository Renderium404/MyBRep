#include <QApplication>
#include <QDebug>
#include <QLabel>
#include <QVBoxLayout>
#include <QVector4D>
#include <QWidget>

#include "RevolutionAxisSingularitySolidFixture.h"

#include "MyBRepOpenGL/Display/BRepDisplayObject.h"
#include "MyBRepOpenGL/Display/BRepDisplayStyle.h"
#include "MyBRepOpenGL/Viewer/BRepViewerWidget.h"

namespace
{

bool addAxisSingularitySolid(
    MyBRep::Display::BRepViewerWidget& viewer)
{
    const RevolutionAxisSingularitySolidFixture::Fixture fixture =
        RevolutionAxisSingularitySolidFixture::create(4.0);

    MyBRep::Display::BRepDisplayStyle style;
    style.surfaceColor = QVector4D(0.30f, 0.66f, 0.88f, 1.0f);
    style.wireColor = QVector4D(0.08f, 0.08f, 0.10f, 1.0f);

    style.surface.revolvedMeshing.boundaryChordTolerance = 0.05;
    style.surface.revolvedMeshing.surfaceChordTolerance = 0.04;
    style.surface.revolvedMeshing.geometricTolerance = 1.0e-10;
    style.surface.revolvedMeshing.minimumBoundarySubdivisionDepth = 1;
    style.surface.revolvedMeshing.maximumBoundarySubdivisionDepth = 12;
    style.surface.revolvedMeshing.maximumSurfaceSubdivisionRounds = 12;

    const MyBRep::Display::BRepDisplayId id =
        viewer.addSolid(
            fixture.solid,
            "RevolutionAxisSingularitySolid",
            style);

    return id != MyBRep::Display::InvalidBRepDisplayId;
}

}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);

    QWidget window;
    window.setWindowTitle(
        "MyBRep Revolution Axis Singularity Solid");
    window.resize(920, 760);

    QVBoxLayout* layout = new QVBoxLayout(&window);
    layout->setContentsMargins(8, 8, 8, 8);
    layout->setSpacing(6);

    QLabel* title = new QLabel(
        "Sphere-like Solid built from Geometry_SurfaceOfRevolution",
        &window);
    title->setAlignment(Qt::AlignCenter);

    QLabel* description = new QLabel(
        "2 pole vertices / 1 seam TEdge / 1 Revolution Face / 1 closed Shell",
        &window);
    description->setAlignment(Qt::AlignCenter);

    MyBRep::Display::BRepViewerWidget* viewer =
        new MyBRep::Display::BRepViewerWidget(&window);
    viewer->setMinimumSize(760, 620);

    QLabel* hint = new QLabel(
        "Mouse: Orbit / Pan / Zoom    F: Fit    P: Projection",
        &window);
    hint->setAlignment(Qt::AlignCenter);

    layout->addWidget(title);
    layout->addWidget(description);
    layout->addWidget(viewer, 1);
    layout->addWidget(hint);

    if (!addAxisSingularitySolid(*viewer))
    {
        qCritical()
            << "Unable to add axis-singularity Revolution Solid.";
        return 1;
    }

    window.show();

    // 等待Qt完成OpenGL Widget初始化，再按实际Viewport范围适配模型。
    application.processEvents();
    viewer->fitItemsToView();

    return application.exec();
}