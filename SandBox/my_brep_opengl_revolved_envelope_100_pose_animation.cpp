#include <cmath>
#include <vector>

#include <QApplication>
#include <QDebug>
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

const int PointTrajectorySamplesPerSegment = 5;      // 每两个相邻姿态之间的轨迹采样区间数量。
const double TrackedPointZ = 0.0;                     // 被追踪点在砂轮局部坐标系中的Z位置。
const double TrackedPointAngle = MyMath::HalfPi;                 // 被追踪点绕砂轮局部Z轴的角度，单位rad。
const int CharacteristicMotionSegment = 0;          // 用于静态特征线验证的运动段。
const double CharacteristicMotionParameter = 0.5;    // 当前运动段内部参数。
const int CharacteristicSamplesPerProfile = 32;      // 每条砂轮母线段的特征线采样区间数。


std::size_t segmentIndex = 0;
const int envelopeMotionSamples = 16;
const int samplesPerSegment = 16;
//插值坐标系生成
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

// 返回from到to的最短局部旋转增量向量。
MyMath::Vector3 rotationParameterIncrement(const MyMath::CoordinateSystem& from, const MyMath::CoordinateSystem& to)
{
    MyMath::Quaternion fromOrientation;
    MyMath::Quaternion toOrientation;
    from.orientation(fromOrientation);
    to.orientation(toOrientation);

    const MyMath::Quaternion relativeOrientation = fromOrientation.inverted()*toOrientation;

    MyMath::Vector3 axis;
    double angle = 0.0;
    relativeOrientation.toAxisAngle(axis, angle);
    return axis*angle;
}

// 创建测试姿态。
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
double normalizeAngle(double angle)
{
    double result = std::fmod(angle, MyMath::TwoPi);
    if (result < 0.0) result += MyMath::TwoPi;
    return result;
}

struct RevolvedSection
{
    RevolvedSection(double radiusValue, double zValue) : radius(radiusValue), z(zValue) {}

    double radius;
    double z;
};
}

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

class RevolveBody
{
public:
    RevolveBody(const std::vector<RevolvedSection>& sections)
        : m_sections(sections), m_translationDerivativeWorld(MyMath::Vector3::zero()), m_w(MyMath::Vector3::zero()) { sort(); }

    RevolveBody(const RevolveBody& other) = default;
    RevolveBody& operator=(const RevolveBody& other) = default;
    ~RevolveBody() = default;
    double minZ() const { return m_sections.front().z; }
    double maxZ() const { return m_sections.back().z; }
    double getRadius(double z) const
    {
        if (z <= m_sections.front().z) return m_sections.front().radius;
        if (z >= m_sections.back().z) return m_sections.back().radius;

        for (std::size_t i = 0; i + 1 < m_sections.size(); ++i)
        {
            const RevolvedSection& first = m_sections[i];
            const RevolvedSection& second = m_sections[i + 1];
            if (z < first.z || z > second.z) continue;

            const double ratio = (z - first.z) / (second.z - first.z);
            return first.radius + (second.radius - first.radius) * ratio;
        }

        return m_sections.back().radius;
    }

    // 返回砂轮局部实体符号值：负值表示内部，0表示边界，正值表示外部。
    double solidValueLocal(const MyMath::Vector3& localPoint) const
    {
        const double minZ = m_sections.front().z;
        const double maxZ = m_sections.back().z;
        const double z = (std::max)(minZ, (std::min)(maxZ, localPoint.z()));
        const double radius = getRadius(z);
        const double radialDistance = std::sqrt(localPoint.x()*localPoint.x() + localPoint.y()*localPoint.y());

        const double radialValue = radialDistance - radius;
        const double bottomValue = minZ - localPoint.z();
        const double topValue = localPoint.z() - maxZ;
        return (std::max)(radialValue, (std::max)(bottomValue, topValue));
    }

    MyMath::Vector3 getPointLocal(double z, double angle) const
    {
        const double radius = getRadius(z);
        return MyMath::Vector3(radius*std::cos(angle), radius*std::sin(angle), z);
    }

    MyMath::Vector3 getPointGlobal(double z, double angle) const
    {
        return m_coordinateSystem.toGlobal(getPointLocal(z, angle));
    }

    void moveTo(const MyMath::CoordinateSystem& coordinateSystem) { m_coordinateSystem = coordinateSystem; }

    // 直接设置当前已插值刀路点位及其运动参数导数，用于包络计算。
    void setMotionState(const MyMath::CoordinateSystem& coordinateSystem,
                        const MyMath::Vector3& translationDerivativeWorld,
                        const MyMath::Vector3& rotationDerivativeLocal)
    {
        m_coordinateSystem = coordinateSystem;
        m_translationDerivativeWorld = translationDerivativeWorld;
        m_w = rotationDerivativeLocal;
    }

    // 仅用于砂轮动画显示，在两个已知点位之间补显示帧。
    void moveBetween(const MyMath::CoordinateSystem& start, const MyMath::CoordinateSystem& end, double t)
    {
        m_translationDerivativeWorld = end.origin() - start.origin();
        m_w = rotationParameterIncrement(start, end);
        m_coordinateSystem = interpolateCoordinateSystem(start, end, t);
    }

    // u=R^T*p'。
    MyMath::Vector3 u() const
    {
        return MyMath::Vector3(
            MyMath::Vector3::dot(m_coordinateSystem.xAxis(), m_translationDerivativeWorld),
            MyMath::Vector3::dot(m_coordinateSystem.yAxis(), m_translationDerivativeWorld),
            MyMath::Vector3::dot(m_coordinateSystem.zAxis(), m_translationDerivativeWorld));
    }

    // W(q,t)=u+w×q，返回局部点q关于运动参数t的局部轨迹切向量。
    MyMath::Vector3 W(double z, double angle) const
    {
        const MyMath::Vector3 q = getPointLocal(z, angle);
        return u() + MyMath::Vector3::cross(m_w, q);
    }

    // 返回砂轮表面在(z,angle)处的局部单位法向量。
    MyMath::Vector3 n(double z, double angle) const
    {
        const MyMath::Vector2 normal = normalAtZ(z);
        return MyMath::Vector3(normal.y()*std::cos(angle), normal.y()*std::sin(angle), normal.x());
    }

    // 求当前z位置满足n·W=0的圆周角。
    bool calculateAngles(double z, double& plusAngle, double& minusAngle) const
    {
        const double epsilon = 1.0e-10;
        const double radius = getRadius(z);
        const MyMath::Vector3 localU = u();
        const MyMath::Vector2 normal = normalAtZ(z);

        const double k = normal.y()*z - normal.x()*radius;
        const double A = normal.y()*localU.x() + k*m_w.y();
        const double B = normal.y()*localU.y() - k*m_w.x();
        const double C = normal.x()*localU.z();
        const double D = std::sqrt(A*A + B*B);

        if (D <= epsilon) return false;

        double value = -C/D;
        if (value < -1.0 - epsilon || value > 1.0 + epsilon) return false;
        value = (std::max)(-1.0, (std::min)(1.0, value));

        const double phi = std::atan2(B, A);
        const double alpha = std::acos(value);

        plusAngle = normalizeAngle(phi + alpha);
        minusAngle = normalizeAngle(phi - alpha);
        return true;
    }

    // 求当前z位置的Plus和Minus两个包络候选特征点。
    bool characteristicPoints(double z,
                            MyMath::Vector3& plusPoint,
                            MyMath::Vector3& minusPoint,
                            double& plusConditionError,
                            double& minusConditionError) const
    {
        double plusAngle = 0.0;
        double minusAngle = 0.0;
        if (!calculateAngles(z, plusAngle, minusAngle)) return false;

        const MyMath::Vector3 plusNormal = n(z, plusAngle);
        const MyMath::Vector3 plusW = W(z, plusAngle);
        const double plusScale = plusNormal.length()*plusW.length();

        const MyMath::Vector3 minusNormal = n(z, minusAngle);
        const MyMath::Vector3 minusW = W(z, minusAngle);
        const double minusScale = minusNormal.length()*minusW.length();

        plusConditionError = plusScale > 0.0 ? std::fabs(MyMath::Vector3::dot(plusNormal, plusW))/plusScale : 0.0;
        minusConditionError = minusScale > 0.0 ? std::fabs(MyMath::Vector3::dot(minusNormal, minusW))/minusScale : 0.0;

        plusPoint = getPointGlobal(z, plusAngle);
        minusPoint = getPointGlobal(z, minusAngle);
        return true;
    }

    bool collectCharacteristicPoints(std::vector<MyMath::Vector3>& plusPoints,
                                    std::vector<MyMath::Vector3>& minusPoints,
                                    double& maxConditionError) const
    {
        plusPoints.clear();
        minusPoints.clear();
        maxConditionError = 0.0;

        const double step = 0.1;
        const double minZ = m_sections.front().z;
        const double maxZ = m_sections.back().z;
        const int intervalCount = static_cast<int>(std::ceil((maxZ - minZ)/step));

        plusPoints.reserve(static_cast<std::size_t>(intervalCount + 1));
        minusPoints.reserve(static_cast<std::size_t>(intervalCount + 1));

        for (int i = 0; i <= intervalCount; ++i)
        {
            const double z = i == intervalCount ? maxZ : minZ + static_cast<double>(i)*step;

            MyMath::Vector3 plusPoint;
            MyMath::Vector3 minusPoint;
            double plusConditionError = 0.0;
            double minusConditionError = 0.0;

            if (!characteristicPoints(z, plusPoint, minusPoint, plusConditionError, minusConditionError))
            {
                plusPoints.clear();
                minusPoints.clear();
                return false;
            }

            plusPoints.push_back(plusPoint);
            minusPoints.push_back(minusPoint);
            maxConditionError = (std::max)(maxConditionError, plusConditionError);
            maxConditionError = (std::max)(maxConditionError, minusConditionError);
        }

        return true;
    }        
        
    
    
    
    const MyMath::CoordinateSystem& coordinateSystem() const { return m_coordinateSystem; }
    std::size_t sectionCount() const { return m_sections.size(); }

private:


    // 返回一条母线段在(z,r)平面中的单位外法向量，x为轴向分量，y为径向分量。
    MyMath::Vector2 segmentNormal(std::size_t sectionIndex) const
    {
        const RevolvedSection& first = m_sections[sectionIndex];
        const RevolvedSection& second = m_sections[sectionIndex + 1];

        const double dz = second.z - first.z;
        const double dr = second.radius - first.radius;
        const double length = std::sqrt(dz*dz + dr*dr);

        return MyMath::Vector2(-dr / length, dz / length);
    }

    // 返回母线在给定z位置的二维单位外法向；拐角使用前后法向的角平分方向。
    MyMath::Vector2 normalAtZ(double z) const
    {
        const double epsilon = 1.0e-10;

        if (z <= m_sections.front().z + epsilon) return segmentNormal(0);
        if (z >= m_sections.back().z - epsilon) return segmentNormal(m_sections.size() - 2);

        for (std::size_t i = 1; i + 1 < m_sections.size(); ++i)
        {
            if (std::fabs(z - m_sections[i].z) > epsilon) continue;

            const MyMath::Vector2 first = segmentNormal(i - 1);
            const MyMath::Vector2 second = segmentNormal(i);

            const double x = first.x() + second.x();
            const double y = first.y() + second.y();
            const double length = std::sqrt(x*x + y*y);

            return MyMath::Vector2(x / length, y / length);
        }

        for (std::size_t i = 0; i + 1 < m_sections.size(); ++i)
        {
            if (z < m_sections[i].z || z > m_sections[i + 1].z) continue;
            return segmentNormal(i);
        }

        return MyMath::Vector2();
    }
    void sort()
    {
        std::sort(m_sections.begin(), m_sections.end(), [](const RevolvedSection& a, const RevolvedSection& b) { return a.z < b.z; });
    }

private:
    std::vector<RevolvedSection> m_sections;
    MyMath::CoordinateSystem m_coordinateSystem;

    MyMath::Vector3 m_translationDerivativeWorld; // p'=p2-p1，世界坐标表达。
    MyMath::Vector3 m_w;                          // R^T*R'*q=w×q。
};

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
bool motionDerivativeAtPose(const std::vector<MyMath::CoordinateSystem>& poses,
                            std::size_t poseIndex,
                            MyMath::Vector3& translationDerivativeWorld,
                            MyMath::Vector3& rotationDerivativeLocal)
{
    if (poses.size() < 2 || poseIndex >= poses.size()) return false;

    if (poseIndex == 0)
    {
        translationDerivativeWorld = poses[1].origin() - poses[0].origin();
        rotationDerivativeLocal = rotationParameterIncrement(poses[0], poses[1]);
        return true;
    }

    if (poseIndex + 1 == poses.size())
    {
        translationDerivativeWorld = poses[poseIndex].origin() - poses[poseIndex - 1].origin();
        rotationDerivativeLocal = rotationParameterIncrement(poses[poseIndex], poses[poseIndex - 1])*(-1.0);
        return true;
    }

    translationDerivativeWorld = (poses[poseIndex + 1].origin() - poses[poseIndex - 1].origin())*0.5;

    const MyMath::Vector3 forwardRotation = rotationParameterIncrement(poses[poseIndex], poses[poseIndex + 1]);
    const MyMath::Vector3 backwardRotation = rotationParameterIncrement(poses[poseIndex], poses[poseIndex - 1]);
    rotationDerivativeLocal = (forwardRotation - backwardRotation)*0.5;
    return true;
}

struct EnvelopeSample
{
    EnvelopeSample() : valid(false), plusPoint(), minusPoint() {}

    bool valid;
    MyMath::Vector3 plusPoint;
    MyMath::Vector3 minusPoint;
};

struct EnvelopeSampleGrid
{
    EnvelopeSampleGrid() : uCount(0), vCount(0), minZ(0.0), maxZ(0.0), zStep(0.0), invalidSampleCount(0) {}

    std::size_t index(std::size_t u, std::size_t v) const { return v*uCount + u; }
    const EnvelopeSample& sample(std::size_t u, std::size_t v) const { return samples[index(u, v)]; }
    EnvelopeSample& sample(std::size_t u, std::size_t v) { return samples[index(u, v)]; }

    std::vector<EnvelopeSample> samples;
    std::size_t uCount;
    std::size_t vCount;
    double minZ;
    double maxZ;
    double zStep;
    std::size_t invalidSampleCount;
};

// poses中的每个点位均视为已经完成刀路插值后的真实离散点位。
// 特征方程在部分(z,pose)位置可能没有实数解，这属于候选包络的有效域边界，不作为整张网格失败处理。
bool collectEnvelopeSamples(RevolveBody body,
                            const std::vector<MyMath::CoordinateSystem>& poses,
                            double zStep,
                            EnvelopeSampleGrid& grid)
{
    grid = EnvelopeSampleGrid();
    if (poses.size() < 2 || zStep <= 0.0) return false;

    grid.minZ = body.minZ();
    grid.maxZ = body.maxZ();
    grid.zStep = zStep;

    const int intervalCount = static_cast<int>(std::ceil((grid.maxZ - grid.minZ)/zStep));
    grid.uCount = static_cast<std::size_t>(intervalCount + 1);
    grid.vCount = poses.size();
    grid.samples.resize(grid.uCount*grid.vCount);

    std::size_t validSampleCount = 0;

    for (std::size_t poseIndex = 0; poseIndex < poses.size(); ++poseIndex)
    {
        MyMath::Vector3 translationDerivativeWorld;
        MyMath::Vector3 rotationDerivativeLocal;

        if (!motionDerivativeAtPose(poses, poseIndex, translationDerivativeWorld, rotationDerivativeLocal)) return false;
        body.setMotionState(poses[poseIndex], translationDerivativeWorld, rotationDerivativeLocal);

        for (std::size_t u = 0; u < grid.uCount; ++u)
        {
            const double z = u + 1 == grid.uCount ? grid.maxZ : grid.minZ + static_cast<double>(u)*zStep;
            EnvelopeSample& sample = grid.sample(u, poseIndex);

            double plusConditionError = 0.0;
            double minusConditionError = 0.0;

            sample.valid = body.characteristicPoints(
                z, sample.plusPoint, sample.minusPoint, plusConditionError, minusConditionError);

            if (sample.valid)
            {
                ++validSampleCount;
                continue;
            }

            ++grid.invalidSampleCount;
            qDebug() << "Characteristic unavailable:"
                     << "pose =" << poseIndex
                     << "z =" << z;
        }
    }

    return grid.uCount >= 2 && grid.vCount >= 2 && validSampleCount > 0;
}

struct EnvelopeRenderMesh
{
    std::vector<GLfloat> vertices;
    std::vector<GLuint> indices;
    AxisAlignedBoundingBox bounds;

    std::size_t vertexCount() const { return vertices.size()/3; }
    std::size_t triangleCount() const { return indices.size()/3; }
};

struct EnvelopeDisplay
{
    EnvelopeDisplay() : geometry(0), part(0) {}

    EnvelopeRenderMesh mesh;
    BufferGeometry* geometry;
    RenderPart* part;
};

bool buildEnvelopeMesh(const EnvelopeSampleGrid& grid, bool plusBranch, EnvelopeRenderMesh& mesh)
{
    mesh = EnvelopeRenderMesh();
    if (grid.uCount < 2 || grid.vCount < 2 || grid.samples.size() != grid.uCount*grid.vCount) return false;

    const GLuint invalidVertex = static_cast<GLuint>(-1);
    std::vector<GLuint> vertexIndices(grid.samples.size(), invalidVertex);

    for (std::size_t v = 0; v < grid.vCount; ++v)
    {
        for (std::size_t u = 0; u < grid.uCount; ++u)
        {
            const EnvelopeSample& sample = grid.sample(u, v);
            if (!sample.valid) continue;

            const MyMath::Vector3& point = plusBranch ? sample.plusPoint : sample.minusPoint;
            const GLuint vertexIndex = static_cast<GLuint>(mesh.vertexCount());
            vertexIndices[grid.index(u, v)] = vertexIndex;

            mesh.vertices.push_back(static_cast<GLfloat>(point.x()));
            mesh.vertices.push_back(static_cast<GLfloat>(point.y()));
            mesh.vertices.push_back(static_cast<GLfloat>(point.z()));
            mesh.bounds.expandToInclude(QVector3D(static_cast<float>(point.x()), static_cast<float>(point.y()), static_cast<float>(point.z())));
        }
    }

    for (std::size_t v = 0; v + 1 < grid.vCount; ++v)
    {
        for (std::size_t u = 0; u + 1 < grid.uCount; ++u)
        {
            const GLuint p00 = vertexIndices[grid.index(u, v)];
            const GLuint p10 = vertexIndices[grid.index(u + 1, v)];
            const GLuint p01 = vertexIndices[grid.index(u, v + 1)];
            const GLuint p11 = vertexIndices[grid.index(u + 1, v + 1)];

            // 当前单元四个角必须都属于候选包络有效域，才生成三角形；不跨越无解区域强行补面。
            if (p00 == invalidVertex || p10 == invalidVertex || p01 == invalidVertex || p11 == invalidVertex) continue;

            mesh.indices.push_back(p00);
            mesh.indices.push_back(p10);
            mesh.indices.push_back(p11);

            mesh.indices.push_back(p00);
            mesh.indices.push_back(p11);
            mesh.indices.push_back(p01);
        }
    }

    return !mesh.vertices.empty() && !mesh.indices.empty() && mesh.bounds.isValid();
}


// 固定一个u位置，将整条刀路上每个pose对应的Plus/Minus分支点连接成完整封边曲面。
// uIndex=0对应MinZ，uIndex=uCount-1对应MaxZ。
bool buildBranchConnectorMesh(const EnvelopeSampleGrid& grid, std::size_t uIndex, EnvelopeRenderMesh& mesh)
{
    mesh = EnvelopeRenderMesh();
    if (grid.uCount < 1 || grid.vCount < 2 || uIndex >= grid.uCount) return false;

    const GLuint invalidVertex = static_cast<GLuint>(-1);
    std::vector<GLuint> plusIndices(grid.vCount, invalidVertex);
    std::vector<GLuint> minusIndices(grid.vCount, invalidVertex);

    mesh.vertices.reserve(grid.vCount*2*3);
    mesh.indices.reserve((grid.vCount - 1)*6);

    for (std::size_t v = 0; v < grid.vCount; ++v)
    {
        const EnvelopeSample& sample = grid.sample(uIndex, v);
        if (!sample.valid) continue;

        plusIndices[v] = static_cast<GLuint>(mesh.vertexCount());

        mesh.vertices.push_back(static_cast<GLfloat>(sample.plusPoint.x()));
        mesh.vertices.push_back(static_cast<GLfloat>(sample.plusPoint.y()));
        mesh.vertices.push_back(static_cast<GLfloat>(sample.plusPoint.z()));
        mesh.bounds.expandToInclude(QVector3D(static_cast<float>(sample.plusPoint.x()),
                                              static_cast<float>(sample.plusPoint.y()),
                                              static_cast<float>(sample.plusPoint.z())));

        minusIndices[v] = static_cast<GLuint>(mesh.vertexCount());

        mesh.vertices.push_back(static_cast<GLfloat>(sample.minusPoint.x()));
        mesh.vertices.push_back(static_cast<GLfloat>(sample.minusPoint.y()));
        mesh.vertices.push_back(static_cast<GLfloat>(sample.minusPoint.z()));
        mesh.bounds.expandToInclude(QVector3D(static_cast<float>(sample.minusPoint.x()),
                                              static_cast<float>(sample.minusPoint.y()),
                                              static_cast<float>(sample.minusPoint.z())));
    }

    for (std::size_t v = 0; v + 1 < grid.vCount; ++v)
    {
        const GLuint plus0 = plusIndices[v];
        const GLuint minus0 = minusIndices[v];
        const GLuint plus1 = plusIndices[v + 1];
        const GLuint minus1 = minusIndices[v + 1];

        if (plus0 == invalidVertex || minus0 == invalidVertex ||
            plus1 == invalidVertex || minus1 == invalidVertex) continue;

        mesh.indices.push_back(plus0);
        mesh.indices.push_back(plus1);
        mesh.indices.push_back(minus1);

        mesh.indices.push_back(plus0);
        mesh.indices.push_back(minus1);
        mesh.indices.push_back(minus0);
    }

    return !mesh.vertices.empty() && !mesh.indices.empty() && mesh.bounds.isValid();
}


bool createEnvelopeDisplay(MyBRep::Display::BRepViewerWidget& window,
                           const QString& name,
                           const QVector4D& color,
                           BufferUsage usage,
                           EnvelopeDisplay& display)
{
    if (display.mesh.vertices.empty() || display.mesh.indices.empty() || !display.mesh.bounds.isValid()) return false;

    BufferGeometry* geometry = new BufferGeometry(name + "_Geometry", usage, RenderType::Triangles);

    std::vector<GeometryVertexAttribute> attributes;
    GeometryVertexAttribute position;
    position.location = GeometryAttribute::Position;
    position.componentCount = 3;
    position.valueOffset = 0;
    attributes.push_back(position);

    geometry->setVertexLayout(3, attributes);
    geometry->setVertexData(display.mesh.vertices);
    geometry->setIndexData(display.mesh.indices);

    const ResourceId geometryId = window.resourceManager().adopt(geometry);
    if (geometryId == InvalidResourceId)
    {
        delete geometry;
        return false;
    }

    Material* material = window.materialManager().createMaterial(name + "_Material");
    if (material == 0)
    {
        window.resourceManager().remove(geometryId);
        return false;
    }

    if (!material->setSurfaceMode(SurfaceMode::Color) ||
        !material->setColor(color) ||
        !material->setBlendMode(color.w() < 1.0f ? BlendMode::Alpha : BlendMode::Opaque))
    {
        window.materialManager().remove(material->id());
        window.resourceManager().remove(geometryId);
        return false;
    }

    material->setLightingEnabled(false);

    RenderItem* item = window.itemManager().createItem(name);
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

    if (!item->addPart(part))
    {
        window.itemManager().removePart(part->id());
        window.itemManager().remove(item->id());
        window.materialManager().remove(material->id());
        window.resourceManager().remove(geometryId);
        return false;
    }

    part->setGeometry(geometry);
    part->setMaterial(material);
    part->setLocalBounds(display.mesh.bounds);

    display.geometry = geometry;
    display.part = part;
    window.update();
    return true;
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
    window.resize(1000, 700);

    const std::vector<RevolvedSection> wheelSections = createGrindingWheelSections();
    const std::vector<MyMath::CoordinateSystem> poses = createTestPoses();
    if (poses.size() < 2) return -1;

    // 包络计算直接使用已经完成刀路插值的poses，不再在相邻poses之间进行二次插值。
    RevolveBody envelopeBody(wheelSections);
    EnvelopeSampleGrid envelopeGrid;

    if (!collectEnvelopeSamples(envelopeBody, poses, 1.0, envelopeGrid))
    {
        qDebug() << "Envelope sample construction failed.";
        return -1;
    }

    EnvelopeDisplay plusEnvelope;
    EnvelopeDisplay minusEnvelope;

    if (!buildEnvelopeMesh(envelopeGrid, true, plusEnvelope.mesh) ||
        !buildEnvelopeMesh(envelopeGrid, false, minusEnvelope.mesh))
    {
        qDebug() << "Envelope mesh construction failed.";
        return -1;
    }

    if (!createEnvelopeDisplay(window, "PlusEnvelope", QVector4D(1.0f, 0.30f, 0.22f, 0.55f), BufferUsage::Static, plusEnvelope))
    {
        qDebug() << "Plus envelope MyOpenGL display creation failed.";
        return -1;
    }

    if (!createEnvelopeDisplay(window, "MinusEnvelope", QVector4D(0.18f, 0.36f, 1.0f, 0.55f), BufferUsage::Static, minusEnvelope))
    {
        qDebug() << "Minus envelope MyOpenGL display creation failed.";
        return -1;
    }

    // 使用MinZ和MaxZ处的Plus/Minus分支点分别构造两张完整黄色封边曲面。
    EnvelopeDisplay minZConnector;
    EnvelopeDisplay maxZConnector;

    if (!buildBranchConnectorMesh(envelopeGrid, 0, minZConnector.mesh))
    {
        qDebug() << "MinZ connector mesh construction failed.";
        return -1;
    }

    if (!buildBranchConnectorMesh(envelopeGrid, envelopeGrid.uCount - 1, maxZConnector.mesh))
    {
        qDebug() << "MaxZ connector mesh construction failed.";
        return -1;
    }

    if (!createEnvelopeDisplay(window,
                               "MinZConnector",
                               QVector4D(1.0f, 0.85f, 0.05f, 0.75f),
                               BufferUsage::Static,
                               minZConnector))
    {
        qDebug() << "MinZ connector MyOpenGL display creation failed.";
        return -1;
    }

    if (!createEnvelopeDisplay(window,
                               "MaxZConnector",
                               QVector4D(1.0f, 0.85f, 0.05f, 0.75f),
                               BufferUsage::Static,
                               maxZConnector))
    {
        qDebug() << "MaxZ connector MyOpenGL display creation failed.";
        return -1;
    }

    qDebug() << "Envelope:"
             << "poses =" << poses.size()
             << "uCount =" << envelopeGrid.uCount
             << "vCount =" << envelopeGrid.vCount
             << "invalid samples =" << envelopeGrid.invalidSampleCount
             << "Plus triangles =" << plusEnvelope.mesh.triangleCount()
             << "Minus triangles =" << minusEnvelope.mesh.triangleCount()
             << "MinZ connector triangles =" << minZConnector.mesh.triangleCount()
             << "MaxZ connector triangles =" << maxZConnector.mesh.triangleCount();

    // 砂轮显示仍使用原来的MyBRep方式；相邻poses之间的插值只用于动画补帧，不参与包络计算。
    RevolveBody wheelBody(wheelSections);
    wheelBody.moveBetween(poses[0], poses[1], 0.0);

    MyBRep::Solid grindingWheel = createRevolvedSolid(wheelSections, 64);
    grindingWheel.setLocalToWorld(wheelBody.coordinateSystem().toMatrix());
    window.addSolid(grindingWheel);

    // 当前特征线仅用于显示；若当前两条刀路采样行包含无解位置，则本帧不强行跨越无解区域连接。
    MyBRep::Display::BRepDisplayStyle plusStyle;
    plusStyle.wireColor = QVector4D(1.0f, 0.0f, 0.0f, 1.0f);
    plusStyle.wireWidth = 3.0f;

    MyBRep::Display::BRepDisplayStyle minusStyle;
    minusStyle.wireColor = QVector4D(0.0f, 0.2f, 1.0f, 1.0f);
    minusStyle.wireWidth = 3.0f;

    MyBRep::Display::BRepDisplayId plusDisplayId = MyBRep::Display::InvalidBRepDisplayId;
    MyBRep::Display::BRepDisplayId minusDisplayId = MyBRep::Display::InvalidBRepDisplayId;

    std::size_t poseIndex = 0;

    auto updateCharacteristicDisplay = [&]()
    {
        if (plusDisplayId != MyBRep::Display::InvalidBRepDisplayId)
        {
            window.removeDisplay(plusDisplayId);
            plusDisplayId = MyBRep::Display::InvalidBRepDisplayId;
        }

        if (minusDisplayId != MyBRep::Display::InvalidBRepDisplayId)
        {
            window.removeDisplay(minusDisplayId);
            minusDisplayId = MyBRep::Display::InvalidBRepDisplayId;
        }

        std::vector<MyMath::Vector3> plusPoints;
        std::vector<MyMath::Vector3> minusPoints;
        plusPoints.reserve(envelopeGrid.uCount);
        minusPoints.reserve(envelopeGrid.uCount);

        for (std::size_t u = 0; u < envelopeGrid.uCount; ++u)
        {
            const EnvelopeSample& sample = envelopeGrid.sample(u, poseIndex);
            if (!sample.valid) return;

            plusPoints.push_back(sample.plusPoint);
            minusPoints.push_back(sample.minusPoint);
        }

        if (plusPoints.size() >= 2)
        {
            const MyBRep::Edge edge = MyBRep::Modeling::makeBSpline(plusPoints);
            plusDisplayId = window.addWireframe(edge, "CharacteristicPlus", plusStyle);
        }

        if (minusPoints.size() >= 2)
        {
            const MyBRep::Edge edge = MyBRep::Modeling::makeBSpline(minusPoints);
            minusDisplayId = window.addWireframe(edge, "CharacteristicMinus", minusStyle);
        }
    };

    auto updateFrame = [&]()
    {
        wheelBody.moveTo(poses[poseIndex]);
        grindingWheel.setLocalToWorld(wheelBody.coordinateSystem().toMatrix());
        window.refreshPlacement(grindingWheel);
        updateCharacteristicDisplay();
    };

    updateFrame();

    window.show();
    QTimer::singleShot(50, [&window]() { window.fitItemsToView(); });

    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [&]()
    {
        ++poseIndex;

        if (poseIndex >= poses.size())
        {
            timer.stop();
            qDebug() << "All toolpath poses completed.";
            return;
        }

        updateFrame();
    });

    timer.start(AnimationIntervalMs);
    return app.exec();
}