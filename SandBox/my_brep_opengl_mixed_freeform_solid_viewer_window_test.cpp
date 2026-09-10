#include <QApplication>
#include <QDebug>
#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>
#include <QVector4D>
#include <QWidget>

#include "MixedFreeformSolidFixture.h"
#include "MyBRepOpenGL/Display/BRepDisplayObject.h"
#include "MyBRepOpenGL/Display/BRepDisplayStyle.h"
#include "MyBRepOpenGL/Viewer/BRepViewerWidget.h"

namespace
{

MyBRep::Display::BRepDisplayStyle style(const QVector4D& color)
{
    MyBRep::Display::BRepDisplayStyle s;
    s.surfaceColor = color;
    s.wireColor = QVector4D(0.06f, 0.06f, 0.07f, 1.0f);
    s.surfaceLightingEnabled = true;
    s.surface.bezierMeshing.surfaceChordTolerance = 0.02;
    s.surface.bsplineMeshing.surfaceChordTolerance = 0.02;
    s.surface.extrudedMeshing.surfaceChordTolerance = 0.02;
    return s;
}

QWidget* pane(const QString& title, const QString& text, MyBRep::Display::BRepViewerWidget*& viewer)
{
    QWidget* w = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(w);
    QLabel* a = new QLabel(title, w);
    QLabel* b = new QLabel(text, w);
    a->setAlignment(Qt::AlignCenter);
    b->setAlignment(Qt::AlignCenter);
    viewer = new MyBRep::Display::BRepViewerWidget(w);
    viewer->setMinimumSize(560, 520);
    layout->addWidget(a);
    layout->addWidget(b);
    layout->addWidget(viewer, 1);
    return w;
}

bool addColoredFaces(MyBRep::Display::BRepViewerWidget& viewer, const MixedFreeformSolidFixture::Fixture& f)
{
    if (viewer.addFace(f.topFace, "BezierTop", style(QVector4D(0.25f, 0.56f, 0.92f, 1.0f))) == MyBRep::Display::InvalidBRepDisplayId)
        return false;
    if (viewer.addFace(f.bottomFace, "BSplineBottom", style(QVector4D(0.94f, 0.50f, 0.20f, 1.0f))) == MyBRep::Display::InvalidBRepDisplayId)
        return false;
    const QVector4D colors[4] = {
        QVector4D(0.30f,0.72f,0.42f,1.0f), QVector4D(0.40f,0.76f,0.48f,1.0f),
        QVector4D(0.26f,0.64f,0.36f,1.0f), QVector4D(0.36f,0.68f,0.40f,1.0f)};
    for (std::size_t i = 0; i < f.sides.size(); ++i)
        if (viewer.addFace(f.sides[i], QString("ExtrusionSide_%1").arg(i), style(colors[i])) == MyBRep::Display::InvalidBRepDisplayId)
            return false;
    return true;
}

}

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    const MixedFreeformSolidFixture::Fixture f = MixedFreeformSolidFixture::create();
    QWidget window;
    window.setWindowTitle("MyBRep Mixed Surface Solid");
    window.resize(1320, 760);
    QHBoxLayout* layout = new QHBoxLayout(&window);
    MyBRep::Display::BRepViewerWidget* solidViewer = 0;
    MyBRep::Display::BRepViewerWidget* faceViewer = 0;
    layout->addWidget(pane("Actual Topology_Solid", "Bezier top + B-Spline bottom + 4 Extrusion sides", solidViewer), 1);
    layout->addWidget(pane("Same Shell Colored by Face Type", "Blue=Bezier / Orange=B-Spline / Green=Extrusion", faceViewer), 1);

    if (!f.solid.isValid() || solidViewer->addSolid(f.solid, "MixedSolid", style(QVector4D(0.66f,0.72f,0.80f,1.0f))) == MyBRep::Display::InvalidBRepDisplayId)
    {
        qCritical() << "Unable to add mixed Topology_Solid.";
        return 1;
    }
    if (!addColoredFaces(*faceViewer, f))
    {
        qCritical() << "Unable to add colored mixed Faces.";
        return 1;
    }

    window.show();
    app.processEvents();
    solidViewer->fitItemsToView();
    faceViewer->fitItemsToView();
    return app.exec();
}