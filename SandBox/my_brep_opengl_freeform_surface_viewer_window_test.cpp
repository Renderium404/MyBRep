#include <QApplication>
#include <QDebug>
#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>
#include <QVector4D>
#include <QWidget>

#include "FreeformSurfaceTestFixtures.h"

#include "MyBRepOpenGL/Display/BRepDisplayObject.h"
#include "MyBRepOpenGL/Display/BRepDisplayStyle.h"
#include "MyBRepOpenGL/Viewer/BRepViewerWidget.h"

namespace
{

QWidget* createViewerPane(
    const QString& title,
    const QString& description,
    MyBRep::Display::BRepViewerWidget*& viewer)
{
    QWidget* pane = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(pane);
    layout->setContentsMargins(6, 6, 6, 6);
    layout->setSpacing(6);

    QLabel* titleLabel = new QLabel(title, pane);
    titleLabel->setAlignment(Qt::AlignCenter);

    QLabel* descriptionLabel = new QLabel(description, pane);
    descriptionLabel->setAlignment(Qt::AlignCenter);

    viewer = new MyBRep::Display::BRepViewerWidget(pane);
    viewer->setMinimumSize(560, 520);

    QLabel* hintLabel = new QLabel(
        "Mouse: Orbit / Pan / Zoom    F: Fit    P: Projection",
        pane);
    hintLabel->setAlignment(Qt::AlignCenter);

    layout->addWidget(titleLabel);
    layout->addWidget(descriptionLabel);
    layout->addWidget(viewer, 1);
    layout->addWidget(hintLabel);

    return pane;
}

bool addBezierFace(MyBRep::Display::BRepViewerWidget& viewer)
{
    const MyBRep::Topology_Face face =
        FreeformSurfaceTestFixtures::createBezierFace();

    MyBRep::Display::BRepDisplayStyle style;
    style.surfaceColor = QVector4D(0.28f, 0.58f, 0.90f, 1.0f);
    style.wireColor = QVector4D(0.08f, 0.08f, 0.10f, 1.0f);
    style.surface.bezierMeshing.boundaryChordTolerance = 0.02;
    style.surface.bezierMeshing.surfaceChordTolerance = 0.02;
    style.surface.bezierMeshing.minimumBoundarySubdivisionDepth = 1;

    const MyBRep::Display::BRepDisplayId id =
        viewer.addFace(face, "BezierFreeformFace", style);

    return id != MyBRep::Display::InvalidBRepDisplayId;
}

bool addBSplineFace(MyBRep::Display::BRepViewerWidget& viewer)
{
    const MyBRep::Topology_Face face =
        FreeformSurfaceTestFixtures::createBSplineFace();

    MyBRep::Display::BRepDisplayStyle style;
    style.surfaceColor = QVector4D(0.92f, 0.55f, 0.24f, 1.0f);
    style.wireColor = QVector4D(0.08f, 0.08f, 0.10f, 1.0f);
    style.surface.bsplineMeshing.boundaryChordTolerance = 0.02;
    style.surface.bsplineMeshing.surfaceChordTolerance = 0.02;
    style.surface.bsplineMeshing.minimumBoundarySubdivisionDepth = 1;

    const MyBRep::Display::BRepDisplayId id =
        viewer.addFace(face, "BSplineFreeformFace", style);

    return id != MyBRep::Display::InvalidBRepDisplayId;
}

}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);

    QWidget window;
    window.setWindowTitle("MyBRep Freeform Surfaces");
    window.resize(1280, 720);

    QHBoxLayout* mainLayout = new QHBoxLayout(&window);
    mainLayout->setContentsMargins(8, 8, 8, 8);
    mainLayout->setSpacing(8);

    MyBRep::Display::BRepViewerWidget* bezierViewer = 0;
    MyBRep::Display::BRepViewerWidget* bsplineViewer = 0;

    QWidget* bezierPane = createViewerPane(
        "Bezier Surface",
        "Cubic 4x4 control net / curved interior / straight trimming boundary",
        bezierViewer);

    QWidget* bsplinePane = createViewerPane(
        "B-Spline Surface",
        "Quadratic 4x4 control net / internal knots U=V=0.5 / two spans each direction",
        bsplineViewer);

    mainLayout->addWidget(bezierPane, 1);
    mainLayout->addWidget(bsplinePane, 1);

    if (!addBezierFace(*bezierViewer))
    {
        qCritical() << "Unable to add Bezier freeform Face.";
        return 1;
    }

    if (!addBSplineFace(*bsplineViewer))
    {
        qCritical() << "Unable to add B-Spline freeform Face.";
        return 1;
    }

    window.show();

    // 等待Qt完成两个OpenGL Widget初始化，再按实际Viewport范围分别适配曲面。
    application.processEvents();
    bezierViewer->fitItemsToView();
    bsplineViewer->fitItemsToView();

    return application.exec();
}