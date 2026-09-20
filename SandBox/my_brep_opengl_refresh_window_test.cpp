#include <QApplication>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QVector4D>
#include <QWidget>

#include "MyMath/Matrix4.h"
#include "MyMath/Vector3.h"

#include "MyBRep/Instance/Face.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Modeling/Wire/WireModeling.h"

#include "MyBRepOpenGL/Display/BRepDisplayObject.h"
#include "MyBRepOpenGL/Display/BRepDisplayStyle.h"
#include "MyBRepOpenGL/Viewer/BRepViewerWidget.h"

namespace
{

MyBRep::Face createRectangleFace(const MyMath::Matrix4& localToWorld)
{
    const MyBRep::Topology_Wire wire = MyBRep::Modeling::createRectangle(8.0, 5.0);
    return MyBRep::Face(MyBRep::Modeling::createPlanarFace(wire), localToWorld);
}

MyBRep::Face createCircleFace(const MyMath::Matrix4& localToWorld)
{
    const MyBRep::Topology_Wire wire = MyBRep::Modeling::createCircle(3.2);
    return MyBRep::Face(MyBRep::Modeling::createPlanarFace(wire), localToWorld);
}

QString statusText(const MyBRep::Display::BRepViewerWidget& viewer,
                   bool alternateDisplay,
                   bool translated,
                   bool circleTopology)
{
    return QString("Display: %1    Placement: %2    Topology: %3    Topology Resources: %4")
        .arg(alternateDisplay ? "Orange / 4.0" : "Blue / 2.0")
        .arg(translated ? "Translated" : "Identity")
        .arg(circleTopology ? "Circle Face" : "Rectangle Face")
        .arg(static_cast<qulonglong>(viewer.displayManager().topologyCount()));
}

}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);

    QWidget window;
    window.setWindowTitle("MyBRep OpenGL Refresh Window Test");
    window.resize(1000, 780);

    QVBoxLayout* mainLayout = new QVBoxLayout(&window);
    mainLayout->setContentsMargins(8, 8, 8, 8);
    mainLayout->setSpacing(6);

    QLabel* title = new QLabel("BRepViewerWidget - Display / Placement / Topology Refresh", &window);
    title->setAlignment(Qt::AlignCenter);

    MyBRep::Display::BRepViewerWidget* viewer = new MyBRep::Display::BRepViewerWidget(&window);
    viewer->setMinimumSize(760, 600);

    QHBoxLayout* buttonLayout = new QHBoxLayout;
    QPushButton* displayButton = new QPushButton("Refresh Display", &window);
    QPushButton* placementButton = new QPushButton("Refresh Placement", &window);
    QPushButton* topologyButton = new QPushButton("Refresh Topology", &window);
    QPushButton* resetButton = new QPushButton("Reset", &window);
    QPushButton* fitButton = new QPushButton("Fit", &window);

    buttonLayout->addWidget(displayButton);
    buttonLayout->addWidget(placementButton);
    buttonLayout->addWidget(topologyButton);
    buttonLayout->addWidget(resetButton);
    buttonLayout->addWidget(fitButton);

    QLabel* statusLabel = new QLabel(&window);
    statusLabel->setAlignment(Qt::AlignCenter);

    QLabel* hintLabel = new QLabel(
        "Display: color/line width   Placement: translation   Topology: rectangle <-> circle",
        &window);
    hintLabel->setAlignment(Qt::AlignCenter);

    mainLayout->addWidget(title);
    mainLayout->addWidget(viewer, 1);
    mainLayout->addLayout(buttonLayout);
    mainLayout->addWidget(statusLabel);
    mainLayout->addWidget(hintLabel);

    MyBRep::Face face = createRectangleFace(MyMath::Matrix4::identity());

    MyBRep::Display::BRepDisplayStyle initialStyle;
    initialStyle.surfaceColor = QVector4D(0.25f, 0.62f, 0.86f, 1.0f);
    initialStyle.wireColor = QVector4D(0.08f, 0.09f, 0.12f, 1.0f);
    initialStyle.surfaceLightingEnabled = true;
    initialStyle.wireWidth = 2.0f;

    MyBRep::Display::BRepDisplayStyle alternateStyle = initialStyle;
    alternateStyle.surfaceColor = QVector4D(0.92f, 0.48f, 0.16f, 1.0f);
    alternateStyle.wireColor = QVector4D(0.28f, 0.08f, 0.03f, 1.0f);
    alternateStyle.wireWidth = 4.0f;

    const MyBRep::Display::BRepDisplayId displayId =
        viewer->addFace(face, "RefreshWindowFace", initialStyle);

    if (displayId == MyBRep::Display::InvalidBRepDisplayId) return 1;

    bool alternateDisplay = false;
    bool translated = false;
    bool circleTopology = false;

    statusLabel->setText(statusText(*viewer, alternateDisplay, translated, circleTopology));

    QObject::connect(displayButton, &QPushButton::clicked, [&]()
    {
        alternateDisplay = !alternateDisplay;

        const MyBRep::Display::BRepDisplayStyle& style =
            alternateDisplay ? alternateStyle : initialStyle;

        if (!viewer->refreshDisplay(face, style))
        {
            statusLabel->setText("Refresh Display FAILED");
            alternateDisplay = !alternateDisplay;
            return;
        }

        statusLabel->setText(statusText(*viewer, alternateDisplay, translated, circleTopology));
    });

    QObject::connect(placementButton, &QPushButton::clicked, [&]()
    {
        translated = !translated;

        const MyMath::Matrix4 localToWorld =
            translated
                ? MyMath::Matrix4::fromTranslation(MyMath::Vector3(3.0, 2.0, 1.5))
                : MyMath::Matrix4::identity();

        if (!face.setLocalToWorld(localToWorld) || !viewer->refreshPlacement(face))
        {
            statusLabel->setText("Refresh Placement FAILED");
            translated = !translated;
            return;
        }

        statusLabel->setText(statusText(*viewer, alternateDisplay, translated, circleTopology));
    });

    QObject::connect(topologyButton, &QPushButton::clicked, [&]()
    {
        circleTopology = !circleTopology;

        const MyMath::Matrix4 localToWorld = face.localToWorld();

        face = circleTopology
                   ? createCircleFace(localToWorld)
                   : createRectangleFace(localToWorld);

        if (!viewer->refreshTopology(face))
        {
            statusLabel->setText("Refresh Topology FAILED");
            circleTopology = !circleTopology;
            return;
        }

        statusLabel->setText(statusText(*viewer, alternateDisplay, translated, circleTopology));
    });

    QObject::connect(resetButton, &QPushButton::clicked, [&]()
    {
        alternateDisplay = false;
        translated = false;
        circleTopology = false;

        face = createRectangleFace(MyMath::Matrix4::identity());

        if (!viewer->refreshTopology(face) ||
            !viewer->refreshPlacement(face) ||
            !viewer->refreshDisplay(face, initialStyle))
        {
            statusLabel->setText("Reset FAILED");
            return;
        }

        statusLabel->setText(statusText(*viewer, alternateDisplay, translated, circleTopology));
        viewer->fitItemsToView();
    });

    QObject::connect(fitButton, &QPushButton::clicked, [&]()
    {
        viewer->fitItemsToView();
    });

    window.show();

    application.processEvents();
    viewer->fitItemsToView();

    return application.exec();
}