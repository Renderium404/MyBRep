#include <algorithm>
#include <cmath>
#include <vector>

#include <QApplication>
#include <QDebug>
#include <QElapsedTimer>
#include <QTimer>
#include <QVector3D>
#include <QVector4D>

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

#include "MyBRepOpenGL/Display/BRepDisplayObject.h"
#include "MyBRepOpenGL/Display/BRepDisplayStyle.h"
#include "MyBRepOpenGL/Viewer/BRepViewerWidget.h"

#include "MyOpenGL/Core/Resource.h"
#include "MyOpenGL/Item/AxisAlignedBoundingBox.h"
#include "MyOpenGL/Item/RenderItem.h"
#include "MyOpenGL/Item/RenderPart.h"
#include "MyOpenGL/Material/Material.h"
#include "MyOpenGL/Resource/BufferGeometry.h"
#include "MyOpenGL/Resource/Geometry.h"

namespace
{

const double SolidTolerance = 1.0e-8;                 // B-Rep构造与几何退化判断基础容差。
const double EnvelopeTolerance = 1.0e-6;              // 扫掠实体内部、边界和外部分类容差。
const double EnvelopeNormalOffset = 0.05;             // 候选包络点沿法向两侧取样的偏移距离。
const int PoseCount = 100;                            // 测试轨迹点位数量，100个点位形成99个运动段。
const int ProfileSubdivisionCount = 4;                // 每条砂轮母线段在u方向的细分区间数量。
const int MotionSubdivisionCount = 4;                 // 每个运动段在v方向的细分区间数量。
const int CoverageSamplesPerMotionSegment = 4;        // 全局覆盖判断时每个运动段的采样区间数量。
const int TrimBisectionCount = 3;                     // 暴露/隐藏边界二分搜索迭代次数。
const int AnimationIntervalMs = 50;                   // 砂轮动画帧间隔，单位ms。

// 回转体母线上的一个离散截面点，以回转半径和轴向Z坐标描述。
struct RevolvedSection
{
    RevolvedSection(double radiusValue, double zValue) : radius(radiusValue), z(zValue) {}

    double radius;                                               // 到回转轴的半径。
    double z;                                                    // 沿回转轴方向的Z坐标。
};

// 包络方程两个解析角解对应的分支。
enum class EnvelopeBranch
{
    Plus,                                                        // theta=phi+alpha。
    Minus                                                        // theta=phi-alpha。
};

// 一个满足局部包络条件的候选点。
struct EnvelopeCandidatePoint
{
    EnvelopeCandidatePoint() : theta(0.0), worldPoint(), worldNormal() {}
    EnvelopeCandidatePoint(double thetaValue, const MyMath::Vector3& pointValue, const MyMath::Vector3& normalValue)
        : theta(thetaValue), worldPoint(pointValue), worldNormal(normalValue) {}

    double theta;                                                // 候选点在砂轮局部回转圆周上的角参数。
    MyMath::Vector3 worldPoint;                                  // 候选点的世界坐标。
    MyMath::Vector3 worldNormal;                                 // 候选点处砂轮表面的世界外法向。
};

// 参数域(u,v)上的一个包络采样结果。
struct EnvelopeSample
{
    EnvelopeSample() : u(0.0), v(0.0), exposed(false), candidate() {}

    double u;                                                    // firstSection与secondSection之间的归一化采样参数。
    double v;                                                    // 当前两个相邻姿态之间的归一化采样参数。
    bool exposed;                                                // 当前候选点是否属于最终暴露包络。
    EnvelopeCandidatePoint candidate;                            // 当前(u,v,branch)对应的候选包络点。
};

// 最终包络显示网格。
struct EnvelopeMesh
{
    std::vector<GLfloat> vertices;                               // 顶点数据，当前布局为Position+Normal。
    std::vector<GLuint> indices;                                 // 三角形顶点索引。
    AxisAlignedBoundingBox bounds;                               // 整个包络网格的包围盒。

    int triangleCount() const { return static_cast<int>(indices.size() / 3); } // 返回三角形数量。
};

std::vector<RevolvedSection> createGrindingWheelSections()
{
    std::vector<RevolvedSection> sections;
    sections.reserve(6);
    sections.push_back(RevolvedSection(34.0, -8.0));
    sections.push_back(RevolvedSection(40.0, -5.0));
    sections.push_back(RevolvedSection(40.0, -2.5));
    sections.push_back(RevolvedSection(40.0, 2.5));
    sections.push_back(RevolvedSection(38.0, 5.0));
    sections.push_back(RevolvedSection(34.0, 8.0));
    return sections;
}

// 使用离散圆周和轴向轮廓创建闭合回转B-Rep Solid，仅用于Viewer中的砂轮显示。
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

    {
        std::vector<MyBRep::Topology_Edge> edges;
        edges.reserve(sideCount);
        for (int sideIndex = sideCount - 1; sideIndex >= 0; --sideIndex) edges.push_back(ringEdges.front()[sideIndex].reversed());

        const MyMath::CoordinateSystem coordinateSystem = MyMath::CoordinateSystem::fromAxes(
            MyMath::Vector3(0.0, 0.0, sections.front().z), MyMath::Vector3::unitX(),
            MyMath::Vector3(0.0, -1.0, 0.0), MyMath::Vector3(0.0, 0.0, -1.0));

        faces.push_back(MyBRep::Modeling::createPlanarFace(coordinateSystem, MyBRep::Modeling::createWire(edges), SolidTolerance));
    }

    {
        std::vector<MyBRep::Topology_Edge> edges;
        edges.reserve(sideCount);
        for (int sideIndex = 0; sideIndex < sideCount; ++sideIndex) edges.push_back(ringEdges.back()[sideIndex]);

        const MyMath::CoordinateSystem coordinateSystem = MyMath::CoordinateSystem::fromAxes(
            MyMath::Vector3(0.0, 0.0, sections.back().z), MyMath::Vector3::unitX(), MyMath::Vector3::unitY(), MyMath::Vector3::unitZ());

        faces.push_back(MyBRep::Modeling::createPlanarFace(coordinateSystem, MyBRep::Modeling::createWire(edges), SolidTolerance));
    }

    return MyBRep::Modeling::makeSolid(MyBRep::Modeling::createShell(faces));
}

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

double normalizeAngle(double angle)
{
    double result = std::fmod(angle, MyMath::TwoPi);
    if (result < 0.0) result += MyMath::TwoPi;
    return result;
}

// 计算一条回转母线位置在一段连续刚体运动中的指定普通包络分支。
bool calculateEnvelopeCandidatePoint(double radius, double h, double radialDerivative,
                                     const MyMath::CoordinateSystem& start, const MyMath::CoordinateSystem& end,
                                     double t, EnvelopeBranch branch, EnvelopeCandidatePoint& result)
{
    const double epsilon = 1.0e-10;
    const MyMath::CoordinateSystem current = interpolateCoordinateSystem(start, end, t);
    const MyMath::Vector3 translationDerivativeWorld = end.origin() - start.origin();

    const MyMath::Vector3 u(MyMath::Vector3::dot(current.xAxis(), translationDerivativeWorld),
                            MyMath::Vector3::dot(current.yAxis(), translationDerivativeWorld),
                            MyMath::Vector3::dot(current.zAxis(), translationDerivativeWorld));

    const MyMath::Vector3 w = rotationParameterDerivative(start, end);
    const double A = u.x() + (h + radius * radialDerivative) * w.y();
    const double B = u.y() - (h + radius * radialDerivative) * w.x();
    const double C = -radialDerivative * u.z();
    const double D = std::sqrt(A * A + B * B);

    if (D <= epsilon) return false;

    double value = -C / D;
    if (value < -1.0 - epsilon || value > 1.0 + epsilon) return false;
    value = (std::max)(-1.0, (std::min)(1.0, value));

    const double phi = std::atan2(B, A);
    const double alpha = std::acos(value);
    if (branch == EnvelopeBranch::Minus && (alpha <= epsilon || std::fabs(alpha - MyMath::Pi) <= epsilon)) return false;

    const double theta = normalizeAngle(branch == EnvelopeBranch::Plus ? phi + alpha : phi - alpha);
    const MyMath::Vector3 localPoint(radius * std::cos(theta), radius * std::sin(theta), h);
    const MyMath::Vector3 localNormal(std::cos(theta), std::sin(theta), -radialDerivative);
    const MyMath::Vector3 worldNormal = current.mapVector(localNormal).normalized();

    if (!worldNormal.isVector()) return false;

    result = EnvelopeCandidatePoint(theta, current.toGlobal(localPoint), worldNormal);
    return true;
}

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

// value<0为砂轮实体内部，value=0为边界，value>0为外部。
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

double movingWheelValue(const MyMath::Vector3& worldPoint, const std::vector<RevolvedSection>& sections,
                        const MyMath::CoordinateSystem& start, const MyMath::CoordinateSystem& end, double t)
{
    return revolvedSolidValue(sections, interpolateCoordinateSystem(start, end, t).toLocal(worldPoint));
}

// 当前原型使用离散全轨迹搜索；一旦发现严格内部覆盖立即返回。
bool isStrictlyInsideSweptSolid(const MyMath::Vector3& worldPoint, const std::vector<RevolvedSection>& sections,
                                const std::vector<MyMath::CoordinateSystem>& poses)
{
    for (std::size_t segmentIndex = 0; segmentIndex + 1 < poses.size(); ++segmentIndex)
    {
        for (int sampleIndex = 0; sampleIndex <= CoverageSamplesPerMotionSegment; ++sampleIndex)
        {
            const double t = static_cast<double>(sampleIndex) / static_cast<double>(CoverageSamplesPerMotionSegment);
            if (movingWheelValue(worldPoint, sections, poses[segmentIndex], poses[segmentIndex + 1], t) < -EnvelopeTolerance) return true;
        }
    }

    return false;
}

// 普通候选点必须不被全轨迹严格覆盖，并且沿当前砂轮外法向的外侧点不能落入扫掠实体内部。
bool isFinalEnvelopePoint(const EnvelopeCandidatePoint& candidate, const std::vector<RevolvedSection>& sections,
                          const std::vector<MyMath::CoordinateSystem>& poses, std::size_t sourceSegmentIndex, double sourceT)
{
    if (isStrictlyInsideSweptSolid(candidate.worldPoint, sections, poses)) return false;

    const MyMath::Vector3 negativePoint = candidate.worldPoint - candidate.worldNormal * EnvelopeNormalOffset;
    if (movingWheelValue(negativePoint, sections, poses[sourceSegmentIndex], poses[sourceSegmentIndex + 1], sourceT) >= -EnvelopeTolerance) return false;

    const MyMath::Vector3 positivePoint = candidate.worldPoint + candidate.worldNormal * EnvelopeNormalOffset;
    return !isStrictlyInsideSweptSolid(positivePoint, sections, poses);
}

// 构造指定母线参数u、运动参数v和解析分支对应的一个包络采样点，并判断该点是否属于最终暴露包络。
EnvelopeSample buildEnvelopeSample(const RevolvedSection& firstSection, const RevolvedSection& secondSection, double u,
                                   const std::vector<RevolvedSection>& sections, const std::vector<MyMath::CoordinateSystem>& poses,
                                   std::size_t motionSegmentIndex, double v, EnvelopeBranch branch)
{
    EnvelopeSample sample;                                            // 当前(u,v)位置的包络采样结果。
    sample.u = u;                                                     // 母线段参数，范围[0,1]。
    sample.v = v;                                                     // 当前运动段参数，范围[0,1]。

    const double dz = secondSection.z - firstSection.z;               // 当前母线段轴向高度变化量。
    if (std::fabs(dz) <= SolidTolerance) return sample;               // dz≈0时无法按r=g(h)计算dr/dh，当前解析模型不处理。

    const double radius = firstSection.radius + (secondSection.radius - firstSection.radius) * u; // 当前母线位置的回转半径。
    const double h = firstSection.z + dz * u;                         // 当前母线位置的轴向坐标。
    const double radialDerivative = (secondSection.radius - firstSection.radius) / dz; // 当前线性母线段的dr/dh。

    if (!calculateEnvelopeCandidatePoint(radius, h, radialDerivative, poses[motionSegmentIndex], poses[motionSegmentIndex + 1],
                                         v, branch, sample.candidate)) return sample; // 当前(u,v)不存在该解析分支候选点。

    sample.exposed = isFinalEnvelopePoint(sample.candidate, sections, poses, motionSegmentIndex, v); // 判断候选点是否属于最终外包络。
    return sample;
}

// 在一个暴露点和一个非暴露点之间二分查找裁剪边界，返回暴露侧最靠近边界的样本。
EnvelopeSample findTrimBoundary(const EnvelopeSample& exposedSample, const EnvelopeSample& hiddenSample,
                                const RevolvedSection& firstSection, const RevolvedSection& secondSection,
                                const std::vector<RevolvedSection>& sections, const std::vector<MyMath::CoordinateSystem>& poses,
                                std::size_t motionSegmentIndex, EnvelopeBranch branch)
{
    EnvelopeSample exposed = exposedSample;
    EnvelopeSample hidden = hiddenSample;

    for (int iteration = 0; iteration < TrimBisectionCount; ++iteration)
    {
        const double u = (exposed.u + hidden.u) * 0.5;
        const double v = (exposed.v + hidden.v) * 0.5;
        const EnvelopeSample middle = buildEnvelopeSample(firstSection, secondSection, u, sections, poses, motionSegmentIndex, v, branch);

        if (middle.exposed) exposed = middle;
        else hidden = middle;
    }

    return exposed;
}

void appendMeshVertex(EnvelopeMesh& mesh, const EnvelopeCandidatePoint& candidate)
{
    mesh.vertices.push_back(static_cast<GLfloat>(candidate.worldPoint.x()));
    mesh.vertices.push_back(static_cast<GLfloat>(candidate.worldPoint.y()));
    mesh.vertices.push_back(static_cast<GLfloat>(candidate.worldPoint.z()));
    mesh.vertices.push_back(static_cast<GLfloat>(candidate.worldNormal.x()));
    mesh.vertices.push_back(static_cast<GLfloat>(candidate.worldNormal.y()));
    mesh.vertices.push_back(static_cast<GLfloat>(candidate.worldNormal.z()));

    mesh.bounds.expandToInclude(QVector3D(static_cast<float>(candidate.worldPoint.x()),
                                         static_cast<float>(candidate.worldPoint.y()),
                                         static_cast<float>(candidate.worldPoint.z())));
}

void appendMeshTriangle(EnvelopeMesh& mesh, const EnvelopeCandidatePoint& first,
                        const EnvelopeCandidatePoint& second, const EnvelopeCandidatePoint& third)
{
    EnvelopeCandidatePoint p0 = first;
    EnvelopeCandidatePoint p1 = second;
    EnvelopeCandidatePoint p2 = third;

    const MyMath::Vector3 triangleNormal = MyMath::Vector3::cross(p1.worldPoint - p0.worldPoint, p2.worldPoint - p0.worldPoint);
    if (!triangleNormal.isVector(SolidTolerance)) return;

    const MyMath::Vector3 expectedNormal = (p0.worldNormal + p1.worldNormal + p2.worldNormal).normalized();
    if (!expectedNormal.isVector()) return;

    if (MyMath::Vector3::dot(triangleNormal, expectedNormal) < 0.0)
    {
        const EnvelopeCandidatePoint temporary = p1;
        p1 = p2;
        p2 = temporary;
    }

    const GLuint firstIndex = static_cast<GLuint>(mesh.vertices.size() / 6);
    appendMeshVertex(mesh, p0);
    appendMeshVertex(mesh, p1);
    appendMeshVertex(mesh, p2);

    mesh.indices.push_back(firstIndex);
    mesh.indices.push_back(firstIndex + 1);
    mesh.indices.push_back(firstIndex + 2);
}

// 对一个候选三角形按三个顶点的暴露状态进行裁剪。
// 混合状态时在暴露/隐藏边之间二分逼近真实裁剪边界。
void appendClippedTriangle(const EnvelopeSample& first, const EnvelopeSample& second, const EnvelopeSample& third,
                           const RevolvedSection& firstSection, const RevolvedSection& secondSection,
                           const std::vector<RevolvedSection>& sections, const std::vector<MyMath::CoordinateSystem>& poses,
                           std::size_t motionSegmentIndex, EnvelopeBranch branch, EnvelopeMesh& mesh)
{
    const EnvelopeSample* samples[3] = { &first, &second, &third };
    int exposedIndices[3];
    int hiddenIndices[3];
    int exposedCount = 0;
    int hiddenCount = 0;

    for (int index = 0; index < 3; ++index)
    {
        if (samples[index]->exposed) exposedIndices[exposedCount++] = index;
        else hiddenIndices[hiddenCount++] = index;
    }

    if (exposedCount == 0) return;

    if (exposedCount == 3)
    {
        appendMeshTriangle(mesh, first.candidate, second.candidate, third.candidate);
        return;
    }

    if (exposedCount == 1)
    {
        const EnvelopeSample& exposed = *samples[exposedIndices[0]];
        const EnvelopeSample boundary1 = findTrimBoundary(exposed, *samples[hiddenIndices[0]], firstSection, secondSection,
                                                          sections, poses, motionSegmentIndex, branch);
        const EnvelopeSample boundary2 = findTrimBoundary(exposed, *samples[hiddenIndices[1]], firstSection, secondSection,
                                                          sections, poses, motionSegmentIndex, branch);

        appendMeshTriangle(mesh, exposed.candidate, boundary1.candidate, boundary2.candidate);
        return;
    }

    const EnvelopeSample& exposed1 = *samples[exposedIndices[0]];
    const EnvelopeSample& exposed2 = *samples[exposedIndices[1]];
    const EnvelopeSample& hidden = *samples[hiddenIndices[0]];
    const EnvelopeSample boundary1 = findTrimBoundary(exposed1, hidden, firstSection, secondSection,
                                                      sections, poses, motionSegmentIndex, branch);
    const EnvelopeSample boundary2 = findTrimBoundary(exposed2, hidden, firstSection, secondSection,
                                                      sections, poses, motionSegmentIndex, branch);

    appendMeshTriangle(mesh, exposed1.candidate, exposed2.candidate, boundary2.candidate);
    appendMeshTriangle(mesh, exposed1.candidate, boundary2.candidate, boundary1.candidate);
}

// 对指定母线段和指定运动段的一条包络解析分支进行规则采样，并将暴露区域三角化加入mesh。
// u表示母线参数，v表示当前运动段内的归一化运动参数，二者范围均为[0,1]。
void appendEnvelopeBranchMesh(const RevolvedSection& firstSection, const RevolvedSection& secondSection,
                              const std::vector<RevolvedSection>& sections, const std::vector<MyMath::CoordinateSystem>& poses,
                              std::size_t motionSegmentIndex, EnvelopeBranch branch, EnvelopeMesh& mesh)
{
    // 参数域采样点数量。SubdivisionCount表示区间数量，因此采样点数量需要加1。
    const int uCount = ProfileSubdivisionCount + 1;
    const int vCount = MotionSubdivisionCount + 1;

    // samples按照[vIndex][uIndex]的顺序展开存储，保存每个参数点对应的候选包络点及暴露状态。
    std::vector<EnvelopeSample> samples(static_cast<std::size_t>(uCount * vCount));

    // 在(u,v)参数域上建立规则采样网格。
    for (int vIndex = 0; vIndex < vCount; ++vIndex)
    {
        // v表示当前相邻两个砂轮点位之间的运动参数。
        const double v = static_cast<double>(vIndex) / static_cast<double>(MotionSubdivisionCount);

        for (int uIndex = 0; uIndex < uCount; ++uIndex)
        {
            // u表示当前回转母线段firstSection→secondSection上的线性参数。
            const double u = static_cast<double>(uIndex) / static_cast<double>(ProfileSubdivisionCount);

            // 计算当前(u,v)对应的指定Plus/Minus包络候选点，并判断其是否属于最终暴露包络。
            samples[static_cast<std::size_t>(vIndex * uCount + uIndex)] =
                buildEnvelopeSample(firstSection, secondSection, u, sections, poses, motionSegmentIndex, v, branch);
        }
    }

    // 遍历参数域中的每一个四边形网格单元。
    for (int vIndex = 0; vIndex < MotionSubdivisionCount; ++vIndex)
    {
        for (int uIndex = 0; uIndex < ProfileSubdivisionCount; ++uIndex)
        {
            // 当前参数单元四个角：
            //
            // p01 -------- p11
            //  |            |
            //  |            |
            // p00 -------- p10
            //
            // u沿水平方向增加，v沿竖直方向增加。
            const EnvelopeSample& p00 = samples[static_cast<std::size_t>(vIndex * uCount + uIndex)];
            const EnvelopeSample& p10 = samples[static_cast<std::size_t>(vIndex * uCount + uIndex + 1)];
            const EnvelopeSample& p01 = samples[static_cast<std::size_t>((vIndex + 1) * uCount + uIndex)];
            const EnvelopeSample& p11 = samples[static_cast<std::size_t>((vIndex + 1) * uCount + uIndex + 1)];

            // 将一个参数四边形拆成两个三角形。
            // appendClippedTriangle内部根据三个顶点的暴露状态进行裁剪；
            // 若三角形跨越最终包络边界，则进一步搜索暴露/隐藏之间的裁剪位置。
            appendClippedTriangle(p00, p10, p11, firstSection, secondSection, sections, poses, motionSegmentIndex, branch, mesh);
            appendClippedTriangle(p00, p11, p01, firstSection, secondSection, sections, poses, motionSegmentIndex, branch, mesh);
        }
    }
}

std::vector<MyMath::CoordinateSystem> createTestPoses()
{
    std::vector<MyMath::CoordinateSystem> poses;
    poses.reserve(PoseCount);

    for (int index = 0; index < PoseCount; ++index)
    {
        const double t = static_cast<double>(index) / static_cast<double>(PoseCount - 1);
        const double x = -55.0 + 110.0 * t;
        const double y = 10.0 * std::sin(MyMath::TwoPi * t);
        const double z = 6.0 * std::sin(MyMath::Pi * t);

        const MyMath::Quaternion rotateX = MyMath::Quaternion::fromAxisAngle(MyMath::Vector3::unitX(), 0.18 * std::sin(MyMath::TwoPi * t));
        const MyMath::Quaternion rotateY = MyMath::Quaternion::fromAxisAngle(MyMath::Vector3::unitY(), 0.35 * std::sin(MyMath::Pi * t));
        const MyMath::Quaternion rotateZ = MyMath::Quaternion::fromAxisAngle(MyMath::Vector3::unitZ(), 0.25 * std::sin(MyMath::TwoPi * t));
        const MyMath::Quaternion orientation = rotateZ * rotateY * rotateX;

        poses.push_back(MyMath::CoordinateSystem::fromQuaternion(MyMath::Vector3(x, y, z), orientation));
    }

    return poses;
}

// 将全部99段普通特征包络进行全轨迹暴露筛选，生成一个统一三角网格。
EnvelopeMesh buildFinalEnvelopeMesh(const std::vector<RevolvedSection>& sections, const std::vector<MyMath::CoordinateSystem>& poses)
{
    EnvelopeMesh mesh;

    for (std::size_t motionSegmentIndex = 0; motionSegmentIndex + 1 < poses.size(); ++motionSegmentIndex)
    {
        for (std::size_t sectionIndex = 0; sectionIndex + 1 < sections.size(); ++sectionIndex)
        {
            appendEnvelopeBranchMesh(sections[sectionIndex], sections[sectionIndex + 1], sections, poses,
                                     motionSegmentIndex, EnvelopeBranch::Plus, mesh);
            appendEnvelopeBranchMesh(sections[sectionIndex], sections[sectionIndex + 1], sections, poses,
                                     motionSegmentIndex, EnvelopeBranch::Minus, mesh);
        }
    }

    return mesh;
}

// 直接使用MyOpenGL显示统一包络网格，不再为每个三角形创建独立B-Rep Face/Edge。
bool addEnvelopeMesh(MyBRep::Display::BRepViewerWidget& window, const EnvelopeMesh& mesh)
{
    if (mesh.vertices.empty() || mesh.indices.empty() || !mesh.bounds.isValid()) return false;

    BufferGeometry* geometry = new BufferGeometry("FinalEnvelopeGeometry", BufferUsage::Static, RenderType::Triangles);

    std::vector<GeometryVertexAttribute> attributes;
    GeometryVertexAttribute position;
    position.location = GeometryAttribute::Position;
    position.componentCount = 3;
    position.valueOffset = 0;
    attributes.push_back(position);

    GeometryVertexAttribute normal;
    normal.location = GeometryAttribute::Normal;
    normal.componentCount = 3;
    normal.valueOffset = 3;
    attributes.push_back(normal);

    geometry->setVertexLayout(6, attributes);
    geometry->setVertexData(mesh.vertices);
    geometry->setIndexData(mesh.indices);

    const ResourceId geometryId = window.resourceManager().adopt(geometry);
    if (geometryId == InvalidResourceId)
    {
        delete geometry;
        return false;
    }

    Material* material = window.materialManager().createMaterial("FinalEnvelopeMaterial");
    if (material == 0)
    {
        window.resourceManager().remove(geometryId);
        return false;
    }

    if (!material->setSurfaceMode(SurfaceMode::Color) || !material->setColor(QVector4D(0.18f, 0.68f, 0.78f, 1.0f)))
    {
        window.materialManager().remove(material->id());
        window.resourceManager().remove(geometryId);
        return false;
    }

    material->setLightingEnabled(true);

    RenderItem* item = window.itemManager().createItem("FinalEnvelope");
    if (item == 0)
    {
        window.materialManager().remove(material->id());
        window.resourceManager().remove(geometryId);
        return false;
    }

    RenderPart* part = window.itemManager().createPart();
    if (part == 0)
    {
        window.itemManager().remove(item->id());
        window.materialManager().remove(material->id());
        window.resourceManager().remove(geometryId);
        return false;
    }
    item->addPart(part);
    part->setGeometry(geometry);
    part->setMaterial(material);
    part->setLocalBounds(mesh.bounds);
    window.update();
    return true;
}

}

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    MyBRep::Display::BRepViewerWidget window;
    window.resize(1100, 760);

    const std::vector<RevolvedSection> wheelSections = createGrindingWheelSections();
    const std::vector<MyMath::CoordinateSystem> poses = createTestPoses();
    const MyBRep::Solid wheelLocal = createRevolvedSolid(wheelSections, 64);

    QElapsedTimer buildTimer;
    buildTimer.start();
    const EnvelopeMesh envelopeMesh = buildFinalEnvelopeMesh(wheelSections, poses);
    // qDebug() << "Final envelope mesh:"
    //          << "Triangles =" << envelopeMesh.triangleCount()
    //          << "Vertices =" << envelopeMesh.vertices.size() / 6
    //          << "Build ms =" << buildTimer.elapsed();

    if (!addEnvelopeMesh(window, envelopeMesh)) qWarning() << "Unable to display final envelope mesh.";

    MyBRep::Display::BRepDisplayStyle wheelStyle;
    wheelStyle.surfaceColor = QVector4D(0.88f, 0.58f, 0.20f, 1.0f);
    wheelStyle.wireColor = QVector4D(0.16f, 0.10f, 0.04f, 1.0f);

    MyBRep::Solid firstWheel(wheelLocal.topology(), poses.front().toMatrix());
    MyBRep::Display::BRepDisplayId wheelDisplayId = window.addSolid(firstWheel, "GrindingWheel", wheelStyle);

    window.show();

    QTimer::singleShot(50, [&window]() { window.fitItemsToView(); });

    QTimer timer;
    int frameIndex = 0;

    //动画定时器
    QObject::connect(&timer, &QTimer::timeout, [&]()
    {

        frameIndex = (frameIndex + 1) % PoseCount;
        firstWheel.setLocalToWorld(poses[frameIndex].toMatrix());
        window.refreshPlacement(firstWheel);

    });

    timer.start(AnimationIntervalMs);
    return app.exec();
}