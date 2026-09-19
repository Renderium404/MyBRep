#include "MyBRepOpenGL/Viewer/BRepViewerWidget.h"
#include "MyBRep/Instance/Solid.h"
#include "MyBRep/Instance/Edge.h"
#include "MyBRep/Instance/Face.h"
#include "MyBRep/Instance/Wire.h"
#include <QApplication>

#include <cmath>
#include <vector>

#include "MyMath/CoordinateSystem.h"
#include "MyMath/MathUtils.h"
#include "MyMath/Vector3.h"

#include "MyBRep/Instance/Solid.h"
#include "MyBRep/Modeling/Edge/EdgeModeling.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Modeling/Shell/ShellModeling.h"
#include "MyBRep/Modeling/Solid/SolidModeling.h"
#include "MyBRep/Modeling/Wire/WireModeling.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Face/Topology_Face.h"
#include "MyBRep/Topology/Shell/Topology_Shell.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"

namespace
{

const double SolidTolerance = 1.0e-8;

struct RevolvedSection
{
    RevolvedSection(double radiusValue, double zValue) : radius(radiusValue), z(zValue) {}

    double radius;
    double z;
};

// 使用离散圆周和轴向轮廓创建闭合回转B-Rep Solid。
// sections按Z从小到大排列，radius必须始终大于0。
MyBRep::Solid createRevolvedSolid(const std::vector<RevolvedSection>& sections, int sideCount)
{
    const std::size_t sectionCount = sections.size();

    std::vector<std::vector<MyBRep::Topology_Vertex> > vertices(sectionCount);

    for (std::size_t sectionIndex = 0; sectionIndex < sectionCount; ++sectionIndex)
    {
        vertices[sectionIndex].reserve(sideCount);

        for (int sideIndex = 0; sideIndex < sideCount; ++sideIndex)
        {
            const double angle = MyMath::TwoPi * static_cast<double>(sideIndex) / static_cast<double>(sideCount);
            const double x = sections[sectionIndex].radius * std::cos(angle);
            const double y = sections[sectionIndex].radius * std::sin(angle);

            vertices[sectionIndex].push_back(
                MyBRep::Topology_Vertex(
                    MyMath::Vector3(x, y, sections[sectionIndex].z)));
        }
    }

    // 每个截面的圆周Edge。
    std::vector<std::vector<MyBRep::Topology_Edge> > ringEdges(sectionCount);

    for (std::size_t sectionIndex = 0; sectionIndex < sectionCount; ++sectionIndex)
    {
        ringEdges[sectionIndex].reserve(sideCount);

        for (int sideIndex = 0; sideIndex < sideCount; ++sideIndex)
        {
            const int nextIndex = (sideIndex + 1) % sideCount;

            ringEdges[sectionIndex].push_back(
                MyBRep::Modeling::createLine(
                    vertices[sectionIndex][sideIndex],
                    vertices[sectionIndex][nextIndex]));
        }
    }

    // 相邻截面之间的轴向Edge。
    std::vector<std::vector<MyBRep::Topology_Edge> > longitudinalEdges(sectionCount - 1);

    for (std::size_t sectionIndex = 0; sectionIndex + 1 < sectionCount; ++sectionIndex)
    {
        longitudinalEdges[sectionIndex].reserve(sideCount);

        for (int sideIndex = 0; sideIndex < sideCount; ++sideIndex)
        {
            longitudinalEdges[sectionIndex].push_back(
                MyBRep::Modeling::createLine(
                    vertices[sectionIndex][sideIndex],
                    vertices[sectionIndex + 1][sideIndex]));
        }
    }

    std::vector<MyBRep::Topology_Face> faces;
    faces.reserve((sectionCount - 1) * sideCount + 2);

    // 外侧面。
    for (std::size_t sectionIndex = 0; sectionIndex + 1 < sectionCount; ++sectionIndex)
    {
        for (int sideIndex = 0; sideIndex < sideCount; ++sideIndex)
        {
            const int nextIndex = (sideIndex + 1) % sideCount;

            std::vector<MyBRep::Topology_Edge> edges;
            edges.reserve(4);

            edges.push_back(ringEdges[sectionIndex][sideIndex]);
            edges.push_back(longitudinalEdges[sectionIndex][nextIndex]);
            edges.push_back(ringEdges[sectionIndex + 1][sideIndex].reversed());
            edges.push_back(longitudinalEdges[sectionIndex][sideIndex].reversed());

            const MyMath::Vector3 p0 = vertices[sectionIndex][sideIndex].point();
            const MyMath::Vector3 p1 = vertices[sectionIndex][nextIndex].point();
            const MyMath::Vector3 p3 = vertices[sectionIndex + 1][sideIndex].point();

            const MyMath::CoordinateSystem coordinateSystem =
                MyMath::CoordinateSystem::fromXY(p0, p1 - p0, p3 - p0);

            faces.push_back(
                MyBRep::Modeling::createPlanarFace(
                    coordinateSystem,
                    MyBRep::Modeling::createWire(edges),
                    SolidTolerance));
        }
    }

    // 底面，外法向-Z。
    {
        std::vector<MyBRep::Topology_Edge> edges;
        edges.reserve(sideCount);

        for (int sideIndex = sideCount - 1; sideIndex >= 0; --sideIndex)
        {
            edges.push_back(ringEdges.front()[sideIndex].reversed());
        }

        const MyMath::CoordinateSystem coordinateSystem =
            MyMath::CoordinateSystem::fromAxes(
                MyMath::Vector3(0.0, 0.0, sections.front().z),
                MyMath::Vector3::unitX(),
                MyMath::Vector3(0.0, -1.0, 0.0),
                MyMath::Vector3(0.0, 0.0, -1.0));

        faces.push_back(
            MyBRep::Modeling::createPlanarFace(
                coordinateSystem,
                MyBRep::Modeling::createWire(edges),
                SolidTolerance));
    }

    // 顶面，外法向+Z。
    {
        std::vector<MyBRep::Topology_Edge> edges;
        edges.reserve(sideCount);

        for (int sideIndex = 0; sideIndex < sideCount; ++sideIndex)
        {
            edges.push_back(ringEdges.back()[sideIndex]);
        }

        const MyMath::CoordinateSystem coordinateSystem =
            MyMath::CoordinateSystem::fromAxes(
                MyMath::Vector3(0.0, 0.0, sections.back().z),
                MyMath::Vector3::unitX(),
                MyMath::Vector3::unitY(),
                MyMath::Vector3::unitZ());

        faces.push_back(
            MyBRep::Modeling::createPlanarFace(
                coordinateSystem,
                MyBRep::Modeling::createWire(edges),
                SolidTolerance));
    }

    const MyBRep::Topology_Shell shell = MyBRep::Modeling::createShell(faces);
    return MyBRep::Modeling::makeSolid(shell);
}

}
MyBRep::Solid createTool()
{
    std::vector<RevolvedSection> sections;
    sections.reserve(8);

    // 刀尖。
    sections.push_back(RevolvedSection(6.0, 0.0));

    // 切削刃部分，直径12。
    sections.push_back(RevolvedSection(6.0, 45.0));

    // 颈部收缩。
    sections.push_back(RevolvedSection(5.2, 50.0));
    sections.push_back(RevolvedSection(5.2, 65.0));

    // 颈部向刀柄过渡。
    sections.push_back(RevolvedSection(6.5, 70.0));
    sections.push_back(RevolvedSection(8.0, 76.0));

    // 刀柄，直径16。
    sections.push_back(RevolvedSection(8.0, 115.0));
    sections.push_back(RevolvedSection(8.0, 125.0));

    return createRevolvedSolid(sections, 48);
}
MyBRep::Solid createGrindingWheel()
{
    std::vector<RevolvedSection> sections;
    sections.reserve(6);

    // 左侧内缩面。
    sections.push_back(RevolvedSection(34.0, -8.0));

    // 左侧工作面过渡。
    sections.push_back(RevolvedSection(40.0, -5.0));

    // 主外圆磨削面。
    sections.push_back(RevolvedSection(40.0, -2.5));
    sections.push_back(RevolvedSection(40.0, 2.5));

    // 右侧工作面过渡。
    sections.push_back(RevolvedSection(38.0, 5.0));

    // 右侧内缩面。
    sections.push_back(RevolvedSection(34.0, 8.0));

    return createRevolvedSolid(sections, 64);
}
int main(int argc, char* argv[])
{
    QApplication application(argc, argv);
    MyBRep::Display::BRepViewerWidget viewer;
    viewer.addSolid(createTool());
    viewer.addSolid(createGrindingWheel());
    viewer.show();
    return application.exec();
}