#include <QApplication>
#include <QDebug>
#include <QLabel>
#include <QVector3D>
#include <QVector4D>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "MyMath/Vector2.h"
#include "MyMath/Vector3.h"

#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Construction/Geometry_Revolved.h"
#include "MyBRep/Geometry/Curve/Geometry_Circle.h"
#include "MyBRep/Geometry/Curve/Geometry_Curve.h"
#include "MyBRep/Geometry/Curve/Geometry_Line.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Circle2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Curve2D.h"
#include "MyBRep/Instance/Shape.h"
#include "MyBRep/Modeling/Shape/RevolvedModeling.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

#include "MyBRepOpenGL/Viewer/BRepViewerWidget.h"

#include "MyOpenGL/Core/Resource.h"
#include "MyOpenGL/Item/AxisAlignedBoundingBox.h"
#include "MyOpenGL/Item/RenderItem.h"
#include "MyOpenGL/Item/RenderPart.h"
#include "MyOpenGL/Material/Material.h"
#include "MyOpenGL/Resource/BufferGeometry.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795;
const double TwoPi = Pi * 2.0;
const double HalfPi = Pi * 0.5;
const double TestTolerance = 1.0e-8;
const unsigned int RevolutionSegmentCount = 96;
const unsigned int FullCircleProfileSegmentCount = 64;

struct DisplayMesh
{
    std::vector<GLfloat> vertices;
    std::vector<GLuint> indices;
    AxisAlignedBoundingBox bounds;

    bool isValid() const
    {
        return !vertices.empty() && !indices.empty() && bounds.isValid();
    }
};

MyBRep::Topology_Edge createLineEdge(const MyBRep::Topology_Vertex& startVertex,
                                     const MyBRep::Topology_Vertex& endVertex,
                                     const MyMath::Vector3& origin,
                                     const MyMath::Vector3& direction,
                                     double firstParameter,
                                     double lastParameter)
{
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> curve(new MyBRep::Geometry_Line(origin, direction));
    return MyBRep::Topology_Edge(startVertex, endVertex, curve, firstParameter, lastParameter, TestTolerance);
}

MyBRep::Topology_Wire createProfile()
{
    const MyMath::Vector3 p0(1.0, -1.0, 0.0);
    const MyMath::Vector3 p1(3.0, -1.0, 0.0);
    const MyMath::Vector3 p2(3.0, 1.0, 0.0);
    const MyMath::Vector3 p3(1.0, 1.0, 0.0);

    const MyBRep::Topology_Vertex v0(p0);
    const MyBRep::Topology_Vertex v1(p1);
    const MyBRep::Topology_Vertex v2(p2);
    const MyBRep::Topology_Vertex v3(p3);

    const MyBRep::Topology_Edge bottom = createLineEdge(v0, v1, MyMath::Vector3(0.0, -1.0, 0.0),
                                                        MyMath::Vector3::unitX(), 1.0, 3.0);

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> circle(
        new MyBRep::Geometry_Circle(MyMath::Vector3(3.0, 0.0, 0.0), 1.0,
                                    MyMath::Vector3::unitX(), MyMath::Vector3::unitY()));
    const MyBRep::Topology_Edge rounded(v1, v2, circle, -HalfPi, HalfPi, TestTolerance);

    const MyBRep::Topology_Edge top = createLineEdge(v2, v3, MyMath::Vector3(5.0, 1.0, 0.0),
                                                     MyMath::Vector3(-1.0, 0.0, 0.0), 2.0, 4.0);
    const MyBRep::Topology_Edge left = createLineEdge(v3, v0, MyMath::Vector3(1.0, 4.0, 0.0),
                                                      MyMath::Vector3(0.0, -1.0, 0.0), 3.0, 5.0);

    std::vector<MyBRep::Topology_Edge> edges;
    edges.push_back(bottom);
    edges.push_back(rounded);
    edges.push_back(top);
    edges.push_back(left);
    return MyBRep::Topology_Wire(edges);
}

std::vector<MyMath::Vector2> sampleProfile(const MyBRep::Geometry_Revolved& revolved)
{
    std::vector<MyMath::Vector2> points;
    if (revolved.profileSegmentCount() == 0) return points;

    const MyBRep::Geometry_Revolved::ProfileSegment& first = revolved.profileSegment(0);
    points.push_back(first.curve->pointAt(first.firstParameter));

    for (std::size_t segmentIndex = 0; segmentIndex < revolved.profileSegmentCount(); ++segmentIndex)
    {
        const MyBRep::Geometry_Revolved::ProfileSegment& segment = revolved.profileSegment(segmentIndex);
        unsigned int subdivisionCount = 1;

        if (segment.curve->kind() == MyBRep::CurveKind::Circle)
        {
            const double sweep = std::fabs(segment.lastParameter - segment.firstParameter);
            subdivisionCount = static_cast<unsigned int>(std::ceil(sweep / TwoPi * FullCircleProfileSegmentCount));
            subdivisionCount = (std::max)(4U, subdivisionCount);
        }

        for (unsigned int subdivisionIndex = 1; subdivisionIndex <= subdivisionCount; ++subdivisionIndex)
        {
            const double ratio = static_cast<double>(subdivisionIndex) / static_cast<double>(subdivisionCount);
            const double parameter = segment.firstParameter + (segment.lastParameter - segment.firstParameter) * ratio;
            points.push_back(segment.curve->pointAt(parameter));
        }
    }

    return points;
}

MyMath::Vector3 revolvedPoint(const MyMath::Vector2& profilePoint, double angle)
{
    return MyMath::Vector3(profilePoint.x() * std::cos(angle),
                           profilePoint.x() * std::sin(angle),
                           profilePoint.y());
}

void appendTriangle(DisplayMesh& mesh, const MyMath::Vector3& first,
                    const MyMath::Vector3& second, const MyMath::Vector3& third)
{
    MyMath::Vector3 normal = MyMath::Vector3::cross(second - first, third - first);
    if (!normal.normalize(0.0)) return;

    const MyMath::Vector3 points[3] = {first, second, third};

    for (int index = 0; index < 3; ++index)
    {
        const MyMath::Vector3& point = points[index];
        mesh.vertices.push_back(static_cast<GLfloat>(point.x()));
        mesh.vertices.push_back(static_cast<GLfloat>(point.y()));
        mesh.vertices.push_back(static_cast<GLfloat>(point.z()));
        mesh.vertices.push_back(static_cast<GLfloat>(normal.x()));
        mesh.vertices.push_back(static_cast<GLfloat>(normal.y()));
        mesh.vertices.push_back(static_cast<GLfloat>(normal.z()));
        mesh.bounds.expandToInclude(QVector3D(static_cast<float>(point.x()),
                                              static_cast<float>(point.y()),
                                              static_cast<float>(point.z())));
    }

    const GLuint base = static_cast<GLuint>(mesh.indices.size());
    mesh.indices.push_back(base);
    mesh.indices.push_back(base + 1);
    mesh.indices.push_back(base + 2);
}

DisplayMesh buildRevolvedMesh(const MyBRep::Geometry_Revolved& revolved)
{
    DisplayMesh mesh;
    const std::vector<MyMath::Vector2> profile = sampleProfile(revolved);

    if (profile.size() < 2) return mesh;

    const bool forward = revolved.profileSignedArea() > 0.0;

    for (std::size_t profileIndex = 0; profileIndex + 1 < profile.size(); ++profileIndex)
    {
        const MyMath::Vector2& firstProfilePoint = profile[profileIndex];
        const MyMath::Vector2& secondProfilePoint = profile[profileIndex + 1];

        for (unsigned int revolutionIndex = 0; revolutionIndex < RevolutionSegmentCount; ++revolutionIndex)
        {
            const double firstAngle = TwoPi * static_cast<double>(revolutionIndex) /
                                      static_cast<double>(RevolutionSegmentCount);
            const double secondAngle = TwoPi * static_cast<double>(revolutionIndex + 1) /
                                       static_cast<double>(RevolutionSegmentCount);

            const MyMath::Vector3 p00 = revolvedPoint(firstProfilePoint, firstAngle);
            const MyMath::Vector3 p10 = revolvedPoint(firstProfilePoint, secondAngle);
            const MyMath::Vector3 p11 = revolvedPoint(secondProfilePoint, secondAngle);
            const MyMath::Vector3 p01 = revolvedPoint(secondProfilePoint, firstAngle);

            if (forward)
            {
                appendTriangle(mesh, p00, p10, p11);
                appendTriangle(mesh, p00, p11, p01);
            }
            else
            {
                appendTriangle(mesh, p00, p11, p10);
                appendTriangle(mesh, p00, p01, p11);
            }
        }
    }

    return mesh;
}

BufferGeometry* createSurfaceGeometry(const DisplayMesh& mesh)
{
    if (!mesh.isValid()) return 0;

    BufferGeometry* geometry = new BufferGeometry("Revolved2DProfileSurface", BufferUsage::Static, RenderType::Triangles);

    GeometryVertexAttribute position;
    position.location = GeometryAttribute::Position;
    position.componentCount = 3;
    position.valueOffset = 0;

    GeometryVertexAttribute normal;
    normal.location = GeometryAttribute::Normal;
    normal.componentCount = 3;
    normal.valueOffset = 3;

    std::vector<GeometryVertexAttribute> attributes;
    attributes.push_back(position);
    attributes.push_back(normal);

    geometry->setVertexLayout(6, attributes);
    geometry->setVertexData(mesh.vertices);
    geometry->setIndexData(mesh.indices);
    return geometry;
}

BufferGeometry* createLineGeometry(const QString& name, const std::vector<MyMath::Vector3>& linePoints,
                                   AxisAlignedBoundingBox& bounds)
{
    if (linePoints.size() < 2 || (linePoints.size() & 1U) != 0) return 0;

    std::vector<GLfloat> vertices;
    std::vector<GLuint> indices;
    vertices.reserve(linePoints.size() * 3);
    indices.reserve(linePoints.size());

    for (std::size_t index = 0; index < linePoints.size(); ++index)
    {
        const MyMath::Vector3& point = linePoints[index];
        vertices.push_back(static_cast<GLfloat>(point.x()));
        vertices.push_back(static_cast<GLfloat>(point.y()));
        vertices.push_back(static_cast<GLfloat>(point.z()));
        indices.push_back(static_cast<GLuint>(index));
        bounds.expandToInclude(QVector3D(static_cast<float>(point.x()),
                                         static_cast<float>(point.y()),
                                         static_cast<float>(point.z())));
    }

    BufferGeometry* geometry = new BufferGeometry(name, BufferUsage::Static, RenderType::Lines);

    GeometryVertexAttribute position;
    position.location = GeometryAttribute::Position;
    position.componentCount = 3;
    position.valueOffset = 0;

    std::vector<GeometryVertexAttribute> attributes;
    attributes.push_back(position);

    geometry->setVertexLayout(3, attributes);
    geometry->setVertexData(vertices);
    geometry->setIndexData(indices);
    return geometry;
}

std::vector<MyMath::Vector3> profileReferenceLines(const MyBRep::Geometry_Revolved& revolved)
{
    const std::vector<MyMath::Vector2> profile = sampleProfile(revolved);
    std::vector<MyMath::Vector3> lines;

    if (profile.size() < 2) return lines;

    lines.reserve((profile.size() - 1) * 2);

    for (std::size_t index = 0; index + 1 < profile.size(); ++index)
    {
        lines.push_back(MyMath::Vector3(profile[index].x(), 0.0, profile[index].y()));
        lines.push_back(MyMath::Vector3(profile[index + 1].x(), 0.0, profile[index + 1].y()));
    }

    return lines;
}

bool addGeometry(MyBRep::Display::BRepViewerWidget& viewer, RenderItem& item,
                 BufferGeometry* geometry, const AxisAlignedBoundingBox& bounds,
                 Material* material, float lineWidth = 1.0f)
{
    if (geometry == 0 || !bounds.isValid() || material == 0) return false;

    const ResourceId resourceId = viewer.resourceManager().adopt(geometry);

    if (resourceId == InvalidResourceId)
    {
        delete geometry;
        return false;
    }

    RenderPart* part = viewer.itemManager().createPart();
    item.addPart(part);
    if (part == 0)
    {
        viewer.resourceManager().remove(resourceId);
        return false;
    }

    part->setGeometry(geometry);
    part->setMaterial(material);
    part->setLocalBounds(bounds);

    if (geometry->renderType() == RenderType::Lines) part->setLineWidth(lineWidth);

    return true;
}

bool buildScene(MyBRep::Display::BRepViewerWidget& viewer, MyBRep::Shape& shape)
{
    const MyBRep::Topology_Wire profile = createProfile();
    shape = MyBRep::Modeling::makeRevolved(profile, TestTolerance);

    if (!shape.isValid())
    {
        qCritical() << "Revolved 2D profile window test failed: Shape is invalid.";
        return false;
    }

    const MyBRep::Geometry_Revolved& revolved = static_cast<const MyBRep::Geometry_Revolved&>(shape.geometry());
    const DisplayMesh mesh = buildRevolvedMesh(revolved);

    if (!mesh.isValid())
    {
        qCritical() << "Revolved 2D profile window test failed: display mesh is invalid.";
        return false;
    }

    Material* surfaceMaterial = viewer.materialManager().createMaterial("Revolved2DProfileSurfaceMaterial");
    Material* profileMaterial = viewer.materialManager().createMaterial("Revolved2DProfileReferenceMaterial");
    Material* axisMaterial = viewer.materialManager().createMaterial("Revolved2DProfileAxisMaterial");

    if (surfaceMaterial == 0 || profileMaterial == 0 || axisMaterial == 0)
    {
        qCritical() << "Revolved 2D profile window test failed: unable to create materials.";
        return false;
    }

    if (!surfaceMaterial->setSurfaceMode(SurfaceMode::Color) ||
        !surfaceMaterial->setColor(QVector4D(0.38f, 0.64f, 0.82f, 1.0f)) ||
        !profileMaterial->setSurfaceMode(SurfaceMode::Color) ||
        !profileMaterial->setColor(QVector4D(0.95f, 0.18f, 0.12f, 1.0f)) ||
        !axisMaterial->setSurfaceMode(SurfaceMode::Color) ||
        !axisMaterial->setColor(QVector4D(0.95f, 0.75f, 0.10f, 1.0f)))
    {
        qCritical() << "Revolved 2D profile window test failed: unable to configure materials.";
        return false;
    }

    surfaceMaterial->setLightingEnabled(true);
    profileMaterial->setLightingEnabled(false);
    axisMaterial->setLightingEnabled(false);

    RenderItem* item = viewer.itemManager().createItem("Revolved2DProfileWindowTest");

    if (item == 0)
    {
        qCritical() << "Revolved 2D profile window test failed: unable to create RenderItem.";
        return false;
    }

    // 参考线先写入深度，后绘Surface在共面像素处使用GL_LESS不会覆盖红色母线。
    AxisAlignedBoundingBox profileBounds;
    BufferGeometry* profileGeometry = createLineGeometry("Revolved2DProfileReference",
                                                         profileReferenceLines(revolved), profileBounds);

    if (!addGeometry(viewer, *item, profileGeometry, profileBounds, profileMaterial, 3.0f))
    {
        qCritical() << "Revolved 2D profile window test failed: unable to add profile reference.";
        return false;
    }

    const MyBRep::Bounds3& localBounds = revolved.localBounds();
    const double axisMargin = (localBounds.maximum().z() - localBounds.minimum().z()) * 0.2 + 0.25;
    std::vector<MyMath::Vector3> axisPoints;
    axisPoints.push_back(MyMath::Vector3(0.0, 0.0, localBounds.minimum().z() - axisMargin));
    axisPoints.push_back(MyMath::Vector3(0.0, 0.0, localBounds.maximum().z() + axisMargin));

    AxisAlignedBoundingBox axisBounds;
    BufferGeometry* axisGeometry = createLineGeometry("Revolved2DProfileAxis", axisPoints, axisBounds);

    if (!addGeometry(viewer, *item, axisGeometry, axisBounds, axisMaterial, 2.5f))
    {
        qCritical() << "Revolved 2D profile window test failed: unable to add rotation axis.";
        return false;
    }

    BufferGeometry* surfaceGeometry = createSurfaceGeometry(mesh);

    if (!addGeometry(viewer, *item, surfaceGeometry, mesh.bounds, surfaceMaterial))
    {
        qCritical() << "Revolved 2D profile window test failed: unable to add surface geometry.";
        return false;
    }

    qDebug() << "Revolved 2D profile window scene built:"
             << "Profile segments =" << static_cast<qulonglong>(revolved.profileSegmentCount())
             << "Profile samples =" << static_cast<qulonglong>(sampleProfile(revolved).size())
             << "Triangles =" << static_cast<qulonglong>(mesh.indices.size() / 3);

    return true;
}

}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);

    QWidget window;
    window.setWindowTitle("MyBRep Revolved 2D Profile Window Test");
    window.resize(1000, 780);

    QVBoxLayout* layout = new QVBoxLayout(&window);
    layout->setContentsMargins(8, 8, 8, 8);
    layout->setSpacing(6);

    QLabel* title = new QLabel("Geometry_Revolved: 3D Wire -> 2D Profile -> Revolved Solid", &window);
    title->setAlignment(Qt::AlignCenter);

    QLabel* description = new QLabel("Blue: revolved surface    Red: actual 2D profile on Y=0    Yellow: local Z rotation axis", &window);
    description->setAlignment(Qt::AlignCenter);

    MyBRep::Display::BRepViewerWidget* viewer = new MyBRep::Display::BRepViewerWidget(&window);
    viewer->setMinimumSize(820, 640);

    QLabel* hint = new QLabel("Mouse: Orbit / Pan / Zoom    F: Fit    P: Projection", &window);
    hint->setAlignment(Qt::AlignCenter);

    layout->addWidget(title);
    layout->addWidget(description);
    layout->addWidget(viewer, 1);
    layout->addWidget(hint);

    MyBRep::Shape shape;

    if (!buildScene(*viewer, shape))
    {
        qCritical() << "Failed to build Revolved 2D profile window test.";
        return 1;
    }

    window.show();
    application.processEvents();
    viewer->fitItemsToView(1.15f);

    return application.exec();
}