#include <algorithm>
#include <cmath>
#include <vector>

#include <QApplication>

#include "MyMath/CoordinateSystem.h"
#include "MyMath/MathUtils.h"
#include "MyMath/Quaternion.h"
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

#include "MyBRepOpenGL/Viewer/BRepViewerWidget.h"

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
            vertices[sectionIndex].push_back(MyBRep::Topology_Vertex(MyMath::Vector3(x, y, sections[sectionIndex].z)));
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
            ringEdges[sectionIndex].push_back(MyBRep::Modeling::createLine(vertices[sectionIndex][sideIndex], vertices[sectionIndex][nextIndex]));
        }
    }

    // 相邻截面之间的轴向Edge。
    std::vector<std::vector<MyBRep::Topology_Edge> > longitudinalEdges(sectionCount - 1);

    for (std::size_t sectionIndex = 0; sectionIndex + 1 < sectionCount; ++sectionIndex)
    {
        longitudinalEdges[sectionIndex].reserve(sideCount);

        for (int sideIndex = 0; sideIndex < sideCount; ++sideIndex)
        {
            longitudinalEdges[sectionIndex].push_back(MyBRep::Modeling::createLine(vertices[sectionIndex][sideIndex], vertices[sectionIndex + 1][sideIndex]));
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
            const MyMath::CoordinateSystem coordinateSystem = MyMath::CoordinateSystem::fromXY(p0, p1 - p0, p3 - p0);

            faces.push_back(MyBRep::Modeling::createPlanarFace(coordinateSystem, MyBRep::Modeling::createWire(edges), SolidTolerance));
        }
    }

    // 底面，外法向-Z。
    {
        std::vector<MyBRep::Topology_Edge> edges;
        edges.reserve(sideCount);
        for (int sideIndex = sideCount - 1; sideIndex >= 0; --sideIndex) edges.push_back(ringEdges.front()[sideIndex].reversed());

        const MyMath::CoordinateSystem coordinateSystem = MyMath::CoordinateSystem::fromAxes(
            MyMath::Vector3(0.0, 0.0, sections.front().z), MyMath::Vector3::unitX(),
            MyMath::Vector3(0.0, -1.0, 0.0), MyMath::Vector3(0.0, 0.0, -1.0));
        faces.push_back(MyBRep::Modeling::createPlanarFace(coordinateSystem, MyBRep::Modeling::createWire(edges), SolidTolerance));
    }

    // 顶面，外法向+Z。
    {
        std::vector<MyBRep::Topology_Edge> edges;
        edges.reserve(sideCount);
        for (int sideIndex = 0; sideIndex < sideCount; ++sideIndex) edges.push_back(ringEdges.back()[sideIndex]);

        const MyMath::CoordinateSystem coordinateSystem = MyMath::CoordinateSystem::fromAxes(
            MyMath::Vector3(0.0, 0.0, sections.back().z), MyMath::Vector3::unitX(), MyMath::Vector3::unitY(), MyMath::Vector3::unitZ());
        faces.push_back(MyBRep::Modeling::createPlanarFace(coordinateSystem, MyBRep::Modeling::createWire(edges), SolidTolerance));
    }

    const MyBRep::Topology_Shell shell = MyBRep::Modeling::createShell(faces);
    return MyBRep::Modeling::makeSolid(shell);
}

struct EnvelopeCandidatePoint
{
    EnvelopeCandidatePoint(double thetaValue, const MyMath::Vector3& localPointValue, const MyMath::Vector3& localNormalValue,
                           const MyMath::Vector3& worldPointValue, const MyMath::Vector3& worldNormalValue)
        : theta(thetaValue), localPoint(localPointValue), localNormal(localNormalValue), worldPoint(worldPointValue), worldNormal(worldNormalValue) {}

    double theta;
    MyMath::Vector3 localPoint;
    MyMath::Vector3 localNormal;
    MyMath::Vector3 worldPoint;
    MyMath::Vector3 worldNormal;
};

enum class SweptPointState
{
    Outside,
    Boundary,
    Inside
};

double normalizeAngle(double angle)
{
    while (angle < 0.0) angle += MyMath::TwoPi;
    while (angle >= MyMath::TwoPi) angle -= MyMath::TwoPi;
    return angle;
}

// 获取两个点位之间运动参数t对应的插值坐标系。
MyMath::CoordinateSystem interpolateCoordinateSystem(const MyMath::CoordinateSystem& start, const MyMath::CoordinateSystem& end, double t)
{
    MyMath::Quaternion startOrientation;
    MyMath::Quaternion endOrientation;
    start.orientation(startOrientation);
    end.orientation(endOrientation);

    const MyMath::Vector3 origin = start.origin() * (1.0 - t) + end.origin() * t;
    const MyMath::Quaternion orientation = MyMath::Quaternion::slerp(startOrientation, endOrientation, t);
    return MyMath::CoordinateSystem::fromQuaternion(origin, orientation);
}

// 返回SLERP运动对应的局部旋转参数导数w，使R(t)^T*R'(t)*q=w×q。
MyMath::Vector3 rotationParameterDerivative(const MyMath::CoordinateSystem& start, const MyMath::CoordinateSystem& end)
{
    MyMath::Quaternion startOrientation;
    MyMath::Quaternion endOrientation;
    start.orientation(startOrientation);
    end.orientation(endOrientation);

    const MyMath::Quaternion relativeOrientation = startOrientation.inverted() * endOrientation;
    MyMath::Vector3 axis;
    double angle = 0.0;
    relativeOrientation.toAxisAngle(axis, angle);
    return axis * angle;
}

// 计算回转面上给定(r,h,g')在运动参数t处的普通包络候选点。
// 正常情况返回0、1或2个候选点；D≈0且C≈0属于退化情况，暂不生成离散候选点。
void calculateEnvelopeCandidatePoints(double radius, double h, double radialDerivative,
                                      const MyMath::CoordinateSystem& start, const MyMath::CoordinateSystem& end,
                                      double t, std::vector<EnvelopeCandidatePoint>& result)
{
    result.clear();

    const double epsilon = 1.0e-10;
    const MyMath::CoordinateSystem current = interpolateCoordinateSystem(start, end, t);

    // p'(t)=p2-p1，转换到当前砂轮局部坐标系得到u(t)=R(t)^T*p'(t)。
    const MyMath::Vector3 translationDerivativeWorld = end.origin() - start.origin();
    const MyMath::Vector3 u(MyMath::Vector3::dot(current.xAxis(), translationDerivativeWorld),
                            MyMath::Vector3::dot(current.yAxis(), translationDerivativeWorld),
                            MyMath::Vector3::dot(current.zAxis(), translationDerivativeWorld));
    const MyMath::Vector3 w = rotationParameterDerivative(start, end);

    const double A = u.x() + (h + radius * radialDerivative) * w.y();
    const double B = u.y() - (h + radius * radialDerivative) * w.x();
    const double C = -radialDerivative * u.z();
    const double D = std::sqrt(A * A + B * B);

    // D≈0时，C!=0无解；C≈0时所有θ均满足，为退化情况。
    if (D <= epsilon) return;

    double value = -C / D;
    if (value < -1.0 - epsilon || value > 1.0 + epsilon) return;
    if (value < -1.0) value = -1.0;
    if (value > 1.0) value = 1.0;

    const double phi = std::atan2(B, A);
    const double alpha = std::acos(value);

    const double thetaPlus = normalizeAngle(phi + alpha);
    const MyMath::Vector3 localPlus(radius * std::cos(thetaPlus), radius * std::sin(thetaPlus), h);
    const MyMath::Vector3 localNormalPlus(std::cos(thetaPlus), std::sin(thetaPlus), -radialDerivative);
    const MyMath::Vector3 worldNormalPlus = current.mapVector(localNormalPlus).normalized();
    result.push_back(EnvelopeCandidatePoint(thetaPlus, localPlus, localNormalPlus, current.toGlobal(localPlus), worldNormalPlus));

    // |C|=D时两条分支重合，只保留一个候选点。
    if (alpha <= epsilon || std::fabs(alpha - MyMath::Pi) <= epsilon) return;

    const double thetaMinus = normalizeAngle(phi - alpha);
    const MyMath::Vector3 localMinus(radius * std::cos(thetaMinus), radius * std::sin(thetaMinus), h);
    const MyMath::Vector3 localNormalMinus(std::cos(thetaMinus), std::sin(thetaMinus), -radialDerivative);
    const MyMath::Vector3 worldNormalMinus = current.mapVector(localNormalMinus).normalized();
    result.push_back(EnvelopeCandidatePoint(thetaMinus, localMinus, localNormalMinus, current.toGlobal(localMinus), worldNormalMinus));
}

// 返回回转体在指定Z位置的半径。
double revolvedRadiusAt(const std::vector<RevolvedSection>& sections, double z)
{
    if (z <= sections.front().z) return sections.front().radius;
    if (z >= sections.back().z) return sections.back().radius;

    for (std::size_t index = 0; index + 1 < sections.size(); ++index)
    {
        const RevolvedSection& first = sections[index];
        const RevolvedSection& second = sections[index + 1];
        if (z < first.z || z > second.z) continue;

        const double t = (z - first.z) / (second.z - first.z);
        return first.radius + (second.radius - first.radius) * t;
    }

    return sections.back().radius;
}

// 返回砂轮实体内外分类值：value<0为内部，value=0为边界，value>0为外部。
double revolvedSolidValue(const std::vector<RevolvedSection>& sections, const MyMath::Vector3& point)
{
    const double minimumZ = sections.front().z;
    const double maximumZ = sections.back().z;
    const double clampedZ = (std::max)(minimumZ, (std::min)(maximumZ, point.z()));
    const double radius = revolvedRadiusAt(sections, clampedZ);
    const double radialDistance = std::sqrt(point.x() * point.x() + point.y() * point.y());

    const double radialValue = radialDistance - radius;
    const double bottomValue = minimumZ - point.z();
    const double topValue = point.z() - maximumZ;
    return (std::max)(radialValue, (std::max)(bottomValue, topValue));
}

// 返回固定世界点在运动参数s对应砂轮实体下的内外分类值。
double movingWheelValue(const MyMath::Vector3& worldPoint, const std::vector<RevolvedSection>& sections,
                        const MyMath::CoordinateSystem& start, const MyMath::CoordinateSystem& end, double s)
{
    const MyMath::CoordinateSystem current = interpolateCoordinateSystem(start, end, s);
    return revolvedSolidValue(sections, current.toLocal(worldPoint));
}

// 在整个运动参数区间内搜索固定世界点对应的最小砂轮实体分类值。
double sweptSolidMinimumValue(const MyMath::Vector3& worldPoint, const std::vector<RevolvedSection>& sections,
                              const MyMath::CoordinateSystem& start, const MyMath::CoordinateSystem& end,
                              double candidateT, int sampleCount)
{
    double minimumValue = movingWheelValue(worldPoint, sections, start, end, candidateT);

    for (int index = 0; index <= sampleCount; ++index)
    {
        const double s = static_cast<double>(index) / static_cast<double>(sampleCount);
        const double value = movingWheelValue(worldPoint, sections, start, end, s);
        if (value < minimumValue) minimumValue = value;
    }

    return minimumValue;
}

// 判断世界点相对于整个扫掠实体的位置。
SweptPointState classifySweptPoint(const MyMath::Vector3& worldPoint, const std::vector<RevolvedSection>& sections,
                                   const MyMath::CoordinateSystem& start, const MyMath::CoordinateSystem& end,
                                   double candidateT, int sampleCount, double tolerance)
{
    const double value = sweptSolidMinimumValue(worldPoint, sections, start, end, candidateT, sampleCount);
    if (value < -tolerance) return SweptPointState::Inside;
    if (value > tolerance) return SweptPointState::Outside;
    return SweptPointState::Boundary;
}

// 判断普通候选包络点是否真正位于最终扫掠实体边界。
bool isFinalEnvelopePoint(const EnvelopeCandidatePoint& candidate, const std::vector<RevolvedSection>& sections,
                          const MyMath::CoordinateSystem& start, const MyMath::CoordinateSystem& end,
                          double candidateT, int sampleCount, double normalOffset, double tolerance)
{
    // 候选点被其他运动位置的砂轮严格覆盖时，一定不是最终包络点。
    if (classifySweptPoint(candidate.worldPoint, sections, start, end, candidateT, sampleCount, tolerance) == SweptPointState::Inside) return false;

    const MyMath::Vector3 normal = candidate.worldNormal.normalized();
    const MyMath::Vector3 positivePoint = candidate.worldPoint + normal * normalOffset;
    const MyMath::Vector3 negativePoint = candidate.worldPoint - normal * normalOffset;

    const SweptPointState positiveState = classifySweptPoint(positivePoint, sections, start, end, candidateT, sampleCount, tolerance);
    const SweptPointState negativeState = classifySweptPoint(negativePoint, sections, start, end, candidateT, sampleCount, tolerance);

    if (positiveState == SweptPointState::Inside && negativeState == SweptPointState::Outside) return true;
    if (positiveState == SweptPointState::Outside && negativeState == SweptPointState::Inside) return true;
    return false;
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
    QApplication app(argc, argv);
    MyBRep::Display::BRepViewerWidget window;
    window.resize(800, 600);

    MyBRep::Solid tool = createTool();
    MyBRep::Solid grindingWheel = createGrindingWheel();
    // window.addSolid(tool);
    window.addSolid(grindingWheel);

    window.show();
    return app.exec();
}