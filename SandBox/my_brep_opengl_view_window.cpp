#include <algorithm>
#include <cmath>
#include <vector>

#include <QApplication>
#include <QDebug>
#include <QTimer>
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
#include "MyBRep/Geometry/Surface/Geometry_BSplineSurface.h"
#include "MyBRepOpenGL/Viewer/BRepViewerWidget.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Line2D.h"
#include "MyBRep/Topology/Topology_Builder.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"
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
const int AnimationIntervalMs = 200;                   // 砂轮动画帧间隔，单位ms。

const int PointTrajectorySamplesPerSegment = 16;      // 每两个相邻姿态之间的轨迹采样区间数量。
const double TrackedPointZ = 0.0;                     // 被追踪点在砂轮局部坐标系中的Z位置。
const double TrackedPointAngle = MyMath::HalfPi;                 // 被追踪点绕砂轮局部Z轴的角度，单位rad。
const int CharacteristicMotionSegment = 0;          // 用于静态特征线验证的运动段。
const double CharacteristicMotionParameter = 0.5;    // 当前运动段内部参数。
const int CharacteristicSamplesPerProfile = 32;      // 每条砂轮母线段的特征线采样区间数。


// std::size_t segmentIndex = 0;
// const int envelopeMotionSamples = 16;
// const int samplesPerSegment = 16;
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

    // 设置S1到S2的运动段，并将砂轮放到运动参数t对应的位置。
    void moveBetween(const MyMath::CoordinateSystem& start, const MyMath::CoordinateSystem& end, double t)
    {
        m_translationDerivativeWorld = end.origin() - start.origin();
        m_w = rotationParameterDerivative(start, end);
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

        const double step = 1.0;
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
    // 返回SLERP运动对应的局部旋转参数导数w，使R^T*R'*q=w×q。
    static MyMath::Vector3 rotationParameterDerivative(const MyMath::CoordinateSystem& start, const MyMath::CoordinateSystem& end)
    {
        MyMath::Quaternion startOrientation;
        MyMath::Quaternion endOrientation;
        start.orientation(startOrientation);
        end.orientation(endOrientation);

        const MyMath::Quaternion relativeOrientation = startOrientation.inverted()*endOrientation;

        MyMath::Vector3 axis;
        double angle = 0.0;
        relativeOrientation.toAxisAngle(axis, angle);
        return axis*angle;
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
std::vector<double> createLinearKnots(double first, double last, std::size_t controlPointCount)
{
    std::vector<double> knots;
    knots.reserve(controlPointCount + 2);

    knots.push_back(first);
    knots.push_back(first);

    for (std::size_t i = 1; i + 1 < controlPointCount; ++i)
    {
        const double t = static_cast<double>(i) / static_cast<double>(controlPointCount - 1);
        knots.push_back(first + (last - first)*t);
    }

    knots.push_back(last);
    knots.push_back(last);
    return knots;
}
bool collectEnvelopeControlPoints(RevolveBody body,
                                  const MyMath::CoordinateSystem& start,
                                  const MyMath::CoordinateSystem& end,
                                  int motionSampleCount,
                                  std::vector<MyMath::Vector3>& plusControlPoints,
                                  std::vector<MyMath::Vector3>& minusControlPoints,
                                  std::size_t& uCount,
                                  std::size_t& vCount)
{
    plusControlPoints.clear();
    minusControlPoints.clear();
    uCount = 0;
    vCount = 0;

    for (int i = 0; i <= motionSampleCount; ++i)
    {
        const double t = static_cast<double>(i)/static_cast<double>(motionSampleCount);
        body.moveBetween(start, end, t);

        std::vector<MyMath::Vector3> plusRow;
        std::vector<MyMath::Vector3> minusRow;
        double maxConditionError = 0.0;

        if (!body.collectCharacteristicPoints(plusRow, minusRow, maxConditionError)) return false;
        if (plusRow.size() != minusRow.size()) return false;

        if (i == 0) uCount = plusRow.size();
        else if (plusRow.size() != uCount) return false;

        plusControlPoints.insert(plusControlPoints.end(), plusRow.begin(), plusRow.end());
        minusControlPoints.insert(minusControlPoints.end(), minusRow.begin(), minusRow.end());
    }

    vCount = static_cast<std::size_t>(motionSampleCount + 1);
    return uCount >= 2 && vCount >= 2;
}



bool createEnvelopeSurfaces(const RevolveBody& body,
                            const MyMath::CoordinateSystem& start,
                            const MyMath::CoordinateSystem& end,
                            int motionSampleCount,
                            MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>& plusSurface,
                            MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>& minusSurface)
{
    plusSurface = MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>();
    minusSurface = MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>();

    std::vector<MyMath::Vector3> plusControlPoints;
    std::vector<MyMath::Vector3> minusControlPoints;
    std::size_t uCount = 0;
    std::size_t vCount = 0;

    if (!collectEnvelopeControlPoints(body, start, end, motionSampleCount, plusControlPoints, minusControlPoints, uCount, vCount)) return false;

    const std::vector<double> uKnots = createLinearKnots(body.minZ(), body.maxZ(), uCount);
    const std::vector<double> vKnots = createLinearKnots(0.0, 1.0, vCount);

    plusSurface = MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>(
        new MyBRep::Geometry_BSplineSurface(1, 1, uCount, vCount, plusControlPoints, uKnots, vKnots));

    minusSurface = MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>(
        new MyBRep::Geometry_BSplineSurface(1, 1, uCount, vCount, minusControlPoints, uKnots, vKnots));

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

    const int envelopeMotionSamples = 16;
    const int samplesPerSegment = 16;
    std::size_t segmentIndex = 0;

    RevolveBody wheelBody(wheelSections);

    // 为完整有限B-Spline曲面建立参数域边界Wire和对应P-Curve。
    auto createEnvelopeFace = [&](const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>& surface) -> MyBRep::Face
    {
        if (!surface || surface->kind() != MyBRep::SurfaceKind::BSpline) return MyBRep::Face();

        const MyBRep::Geometry_BSplineSurface& spline = static_cast<const MyBRep::Geometry_BSplineSurface&>(*surface);
        const std::size_t uCount = spline.uControlPointCount();
        const std::size_t vCount = spline.vControlPointCount();

        const double u0 = spline.uDomainStart();
        const double u1 = spline.uDomainEnd();
        const double v0 = spline.vDomainStart();
        const double v1 = spline.vDomainEnd();

        const MyMath::Vector2 uv00(u0, v0);
        const MyMath::Vector2 uv10(u1, v0);
        const MyMath::Vector2 uv11(u1, v1);
        const MyMath::Vector2 uv01(u0, v1);

        const MyBRep::Topology_Vertex vertex00(surface->pointAt(u0, v0));
        const MyBRep::Topology_Vertex vertex10(surface->pointAt(u1, v0));
        const MyBRep::Topology_Vertex vertex11(surface->pointAt(u1, v1));
        const MyBRep::Topology_Vertex vertex01(surface->pointAt(u0, v1));

        std::vector<MyMath::Vector3> bottomPoints;
        std::vector<MyMath::Vector3> topPoints;
        std::vector<MyMath::Vector3> leftPoints;
        std::vector<MyMath::Vector3> rightPoints;

        bottomPoints.reserve(uCount);
        topPoints.reserve(uCount);
        leftPoints.reserve(vCount);
        rightPoints.reserve(vCount);

        for (std::size_t u = 0; u < uCount; ++u)
        {
            bottomPoints.push_back(spline.controlPoint(u, 0));
            topPoints.push_back(spline.controlPoint(u, vCount - 1));
        }

        for (std::size_t v = 0; v < vCount; ++v)
        {
            leftPoints.push_back(spline.controlPoint(0, v));
            rightPoints.push_back(spline.controlPoint(uCount - 1, v));
        }

        MyBRep::Topology_Edge bottom = MyBRep::Modeling::createBSpline(vertex00, vertex10, bottomPoints, spline.uDegree(), spline.uKnots(), SolidTolerance);
        MyBRep::Topology_Edge right = MyBRep::Modeling::createBSpline(vertex10, vertex11, rightPoints, spline.vDegree(), spline.vKnots(), SolidTolerance);
        MyBRep::Topology_Edge top = MyBRep::Modeling::createBSpline(vertex01, vertex11, topPoints, spline.uDegree(), spline.uKnots(), SolidTolerance);
        MyBRep::Topology_Edge left = MyBRep::Modeling::createBSpline(vertex00, vertex01, leftPoints, spline.vDegree(), spline.vKnots(), SolidTolerance);

        auto addCurveOnSurface = [&](MyBRep::Topology_Edge& edge, const MyMath::Vector2& firstUV, const MyMath::Vector2& lastUV)
        {
            const MyMath::Vector2 direction = lastUV - firstUV;
            const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> curve(new MyBRep::Geometry_Line2D(firstUV, direction));
            MyBRep::Topology_Builder::addCurveOnSurface(edge, surface, curve, 0.0, direction.length(), SolidTolerance);
        };

        addCurveOnSurface(bottom, uv00, uv10);
        addCurveOnSurface(right, uv10, uv11);
        addCurveOnSurface(top, uv01, uv11);
        addCurveOnSurface(left, uv00, uv01);

        std::vector<MyBRep::Topology_Edge> edges;
        edges.reserve(4);
        edges.push_back(bottom);
        edges.push_back(right);
        edges.push_back(top.reversed());
        edges.push_back(left.reversed());

        const MyBRep::Topology_Wire wire(edges);
        return MyBRep::Modeling::makeFace(surface, std::vector<MyBRep::Topology_Wire>(1, wire));
    };

    MyBRep::Display::BRepDisplayStyle plusSurfaceStyle;
    plusSurfaceStyle.surfaceColor = QVector4D(1.0f, 0.55f, 0.50f, 0.45f);
    plusSurfaceStyle.wireColor = QVector4D(0.65f, 0.05f, 0.03f, 1.0f);
    plusSurfaceStyle.wireWidth = 1.0f;

    MyBRep::Display::BRepDisplayStyle minusSurfaceStyle;
    minusSurfaceStyle.surfaceColor = QVector4D(0.45f, 0.60f, 1.0f, 0.45f);
    minusSurfaceStyle.wireColor = QVector4D(0.04f, 0.12f, 0.65f, 1.0f);
    minusSurfaceStyle.wireWidth = 1.0f;

    // 构造当前运动段候选包络面并永久保留在Viewer中。
    auto addEnvelopeSurfaces = [&]() -> bool
    {
        MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> plusSurface;
        MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> minusSurface;

        if (!createEnvelopeSurfaces(wheelBody, poses[segmentIndex], poses[segmentIndex + 1], envelopeMotionSamples, plusSurface, minusSurface))
            return false;

        const MyBRep::Face plusFace = createEnvelopeFace(plusSurface);
        const MyBRep::Face minusFace = createEnvelopeFace(minusSurface);
        if (!plusFace.isValid() || !minusFace.isValid()) return false;

        const QString plusName = QString("PlusEnvelopeSurface_%1").arg(static_cast<qulonglong>(segmentIndex));
        const QString minusName = QString("MinusEnvelopeSurface_%1").arg(static_cast<qulonglong>(segmentIndex));

        const MyBRep::Display::BRepDisplayId plusId = window.addFace(plusFace, plusName, plusSurfaceStyle);
        const MyBRep::Display::BRepDisplayId minusId = window.addFace(minusFace, minusName, minusSurfaceStyle);

        if (plusId == MyBRep::Display::InvalidBRepDisplayId || minusId == MyBRep::Display::InvalidBRepDisplayId) return false;

        qDebug() << "Envelope segment" << segmentIndex << "Plus =" << plusId << "Minus =" << minusId;
        return true;
    };

    // 创建砂轮。
    wheelBody.moveBetween(poses[0], poses[1], 0.0);

    MyBRep::Solid grindingWheel = createRevolvedSolid(wheelSections, 64);
    grindingWheel.setLocalToWorld(wheelBody.coordinateSystem().toMatrix());
    window.addSolid(grindingWheel);

    // 当前参数位置的两条特征线。
    MyBRep::Display::BRepDisplayStyle plusStyle;
    plusStyle.wireColor = QVector4D(1.0f, 0.0f, 0.0f, 1.0f);
    plusStyle.wireWidth = 3.0f;

    MyBRep::Display::BRepDisplayStyle minusStyle;
    minusStyle.wireColor = QVector4D(0.0f, 0.2f, 1.0f, 1.0f);
    minusStyle.wireWidth = 3.0f;

    MyBRep::Display::BRepDisplayId plusDisplayId = MyBRep::Display::InvalidBRepDisplayId;
    MyBRep::Display::BRepDisplayId minusDisplayId = MyBRep::Display::InvalidBRepDisplayId;

    // 更新当前运动参数位置。
    auto updateFrame = [&](double t)
    {
        wheelBody.moveBetween(poses[segmentIndex], poses[segmentIndex + 1], t);

        grindingWheel.setLocalToWorld(wheelBody.coordinateSystem().toMatrix());
        window.refreshPlacement(grindingWheel);

        // 特征线只保留当前帧。
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
        double maxConditionError = 0.0;

        if (!wheelBody.collectCharacteristicPoints(plusPoints, minusPoints, maxConditionError)) return;

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

    // 第一运动段候选面。
    if (!addEnvelopeSurfaces())
    {
        qDebug() << "Envelope surface construction failed at segment" << segmentIndex;
        return -1;
    }

    updateFrame(0.0);

    window.show();
    QTimer::singleShot(50, [&window]() { window.fitItemsToView(); });

    int sampleIndex = 0;

    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [&]()
    {
        ++sampleIndex;

        if (sampleIndex > samplesPerSegment)
        {
            sampleIndex = 0;
            ++segmentIndex;

            if (segmentIndex + 1 >= poses.size())
            {
                timer.stop();
                qDebug() << "All motion segments completed.";
                qDebug() << "Envelope surfaces kept for" << poses.size() - 1 << "segments.";
                return;
            }

            // 新运动段的候选包络面加入Viewer，旧面不删除。
            if (!addEnvelopeSurfaces())
            {
                timer.stop();
                qDebug() << "Envelope surface construction failed at segment" << segmentIndex;
                return;
            }

            qDebug() << "Motion segment" << segmentIndex << "/" << poses.size() - 2;
        }

        const double t = static_cast<double>(sampleIndex)/static_cast<double>(samplesPerSegment);
        updateFrame(t);
    });

    timer.start(AnimationIntervalMs);
    return app.exec();
}



// int main(int argc, char* argv[])
// {
//     QApplication app(argc, argv);
//     MyBRep::Display::BRepViewerWidget window;
//     window.resize(1000, 700);

//     const std::vector<RevolvedSection> wheelSections = createGrindingWheelSections();
//     const std::vector<MyMath::CoordinateSystem> poses = createTestPoses();

//     const std::size_t segmentIndex = CharacteristicMotionSegment;
//     const MyMath::CoordinateSystem& startPose = poses[segmentIndex];
//     const MyMath::CoordinateSystem& endPose = poses[segmentIndex + 99];
//     const int envelopeMotionSamples = 16;

//     RevolveBody wheelBody(wheelSections);

//     // 为完整有限B-Spline曲面建立参数域边界Wire和对应P-Curve。
//     auto createEnvelopeFace = [&](const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>& surface) -> MyBRep::Face
//     {
//         if (!surface || surface->kind() != MyBRep::SurfaceKind::BSpline) return MyBRep::Face();

//         const MyBRep::Geometry_BSplineSurface& spline = static_cast<const MyBRep::Geometry_BSplineSurface&>(*surface);
//         const std::size_t uCount = spline.uControlPointCount();
//         const std::size_t vCount = spline.vControlPointCount();

//         const double u0 = spline.uDomainStart();
//         const double u1 = spline.uDomainEnd();
//         const double v0 = spline.vDomainStart();
//         const double v1 = spline.vDomainEnd();

//         const MyMath::Vector2 uv00(u0, v0);
//         const MyMath::Vector2 uv10(u1, v0);
//         const MyMath::Vector2 uv11(u1, v1);
//         const MyMath::Vector2 uv01(u0, v1);

//         const MyBRep::Topology_Vertex vertex00(surface->pointAt(u0, v0));
//         const MyBRep::Topology_Vertex vertex10(surface->pointAt(u1, v0));
//         const MyBRep::Topology_Vertex vertex11(surface->pointAt(u1, v1));
//         const MyBRep::Topology_Vertex vertex01(surface->pointAt(u0, v1));

//         std::vector<MyMath::Vector3> bottomPoints;
//         std::vector<MyMath::Vector3> topPoints;
//         std::vector<MyMath::Vector3> leftPoints;
//         std::vector<MyMath::Vector3> rightPoints;

//         bottomPoints.reserve(uCount);
//         topPoints.reserve(uCount);
//         leftPoints.reserve(vCount);
//         rightPoints.reserve(vCount);

//         for (std::size_t u = 0; u < uCount; ++u)
//         {
//             bottomPoints.push_back(spline.controlPoint(u, 0));
//             topPoints.push_back(spline.controlPoint(u, vCount - 1));
//         }

//         for (std::size_t v = 0; v < vCount; ++v)
//         {
//             leftPoints.push_back(spline.controlPoint(0, v));
//             rightPoints.push_back(spline.controlPoint(uCount - 1, v));
//         }

//         MyBRep::Topology_Edge bottom = MyBRep::Modeling::createBSpline(vertex00, vertex10, bottomPoints, spline.uDegree(), spline.uKnots(), SolidTolerance);
//         MyBRep::Topology_Edge right = MyBRep::Modeling::createBSpline(vertex10, vertex11, rightPoints, spline.vDegree(), spline.vKnots(), SolidTolerance);
//         MyBRep::Topology_Edge top = MyBRep::Modeling::createBSpline(vertex01, vertex11, topPoints, spline.uDegree(), spline.uKnots(), SolidTolerance);
//         MyBRep::Topology_Edge left = MyBRep::Modeling::createBSpline(vertex00, vertex01, leftPoints, spline.vDegree(), spline.vKnots(), SolidTolerance);

//         auto addCurveOnSurface = [&](MyBRep::Topology_Edge& edge, const MyMath::Vector2& firstUV, const MyMath::Vector2& lastUV)
//         {
//             const MyMath::Vector2 direction = lastUV - firstUV;
//             const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> curve(new MyBRep::Geometry_Line2D(firstUV, direction));
//             MyBRep::Topology_Builder::addCurveOnSurface(edge, surface, curve, 0.0, direction.length(), SolidTolerance);
//         };

//         addCurveOnSurface(bottom, uv00, uv10);
//         addCurveOnSurface(right, uv10, uv11);
//         addCurveOnSurface(top, uv01, uv11);
//         addCurveOnSurface(left, uv00, uv01);

//         std::vector<MyBRep::Topology_Edge> edges;
//         edges.reserve(4);
//         edges.push_back(bottom);
//         edges.push_back(right);
//         edges.push_back(top.reversed());
//         edges.push_back(left.reversed());

//         const MyBRep::Topology_Wire wire(edges);
//         return MyBRep::Modeling::makeFace(surface, std::vector<MyBRep::Topology_Wire>(1, wire));
//     };
    
    
    
//     // 构造当前运动段的两张候选包络面。
//     MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> plusSurface;
//     MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> minusSurface;




//     if (!createEnvelopeSurfaces(wheelBody, startPose, endPose, envelopeMotionSamples, plusSurface, minusSurface))
//     {
//         qDebug() << "Envelope surface construction failed.";
//         return -1;
//     }

//     const MyBRep::Face plusFace = createEnvelopeFace(plusSurface);
//     const MyBRep::Face minusFace = createEnvelopeFace(minusSurface);

//     if (!plusFace.isValid() || !minusFace.isValid())
//     {
//         qDebug() << "Envelope Face construction failed.";
//         return -1;
//     }

//     MyBRep::Display::BRepDisplayStyle plusSurfaceStyle;
//     plusSurfaceStyle.surfaceColor = QVector4D(1.0f, 0.55f, 0.50f, 0.45f);
//     plusSurfaceStyle.wireColor = QVector4D(0.65f, 0.05f, 0.03f, 1.0f);
//     plusSurfaceStyle.wireWidth = 1.0f;

//     MyBRep::Display::BRepDisplayStyle minusSurfaceStyle;
//     minusSurfaceStyle.surfaceColor = QVector4D(0.45f, 0.60f, 1.0f, 0.45f);
//     minusSurfaceStyle.wireColor = QVector4D(0.04f, 0.12f, 0.65f, 1.0f);
//     minusSurfaceStyle.wireWidth = 1.0f;

//     const MyBRep::Display::BRepDisplayId plusSurfaceId =
//         window.addFace(plusFace, "PlusEnvelopeSurface", plusSurfaceStyle);

//     const MyBRep::Display::BRepDisplayId minusSurfaceId =
//         window.addFace(minusFace, "MinusEnvelopeSurface", minusSurfaceStyle);

//     qDebug() << "Plus envelope display =" << plusSurfaceId;
//     qDebug() << "Minus envelope display =" << minusSurfaceId;

//     // 创建运动砂轮。
//     wheelBody.moveBetween(startPose, endPose, 0.0);

//     MyBRep::Solid grindingWheel = createRevolvedSolid(wheelSections, 64);
//     grindingWheel.setLocalToWorld(wheelBody.coordinateSystem().toMatrix());
//     window.addSolid(grindingWheel);

//     // 当前参数位置的两条特征线。
//     MyBRep::Display::BRepDisplayStyle plusStyle;
//     plusStyle.wireColor = QVector4D(1.0f, 0.0f, 0.0f, 1.0f);
//     plusStyle.wireWidth = 3.0f;

//     MyBRep::Display::BRepDisplayStyle minusStyle;
//     minusStyle.wireColor = QVector4D(0.0f, 0.2f, 1.0f, 1.0f);
//     minusStyle.wireWidth = 3.0f;

//     MyBRep::Display::BRepDisplayId plusDisplayId = MyBRep::Display::InvalidBRepDisplayId;
//     MyBRep::Display::BRepDisplayId minusDisplayId = MyBRep::Display::InvalidBRepDisplayId;



//     auto updateFrame = [&](double t)
//     {
//         wheelBody.moveBetween(startPose, endPose, t);

//         grindingWheel.setLocalToWorld(wheelBody.coordinateSystem().toMatrix());
//         window.refreshPlacement(grindingWheel);

//         if (plusDisplayId != MyBRep::Display::InvalidBRepDisplayId)
//         {
//             window.removeDisplay(plusDisplayId);
//             plusDisplayId = MyBRep::Display::InvalidBRepDisplayId;
//         }

//         if (minusDisplayId != MyBRep::Display::InvalidBRepDisplayId)
//         {
//             window.removeDisplay(minusDisplayId);
//             minusDisplayId = MyBRep::Display::InvalidBRepDisplayId;
//         }

//         std::vector<MyMath::Vector3> plusPoints;
//         std::vector<MyMath::Vector3> minusPoints;
//         double maxConditionError = 0.0;

//         if (!wheelBody.collectCharacteristicPoints(plusPoints, minusPoints, maxConditionError)) return;

//         if (plusPoints.size() >= 2)
//         {
//             const MyBRep::Edge edge = MyBRep::Modeling::makeBSpline(plusPoints);
//             plusDisplayId = window.addWireframe(edge, "CharacteristicPlus", plusStyle);
//         }

//         if (minusPoints.size() >= 2)
//         {
//             const MyBRep::Edge edge = MyBRep::Modeling::makeBSpline(minusPoints);
//             minusDisplayId = window.addWireframe(edge, "CharacteristicMinus", minusStyle);
//         }
//     };
    
//     updateFrame(0.0);

//     window.show();
//     QTimer::singleShot(50, [&window]() { window.fitItemsToView(); });

//     const int samplesPerSegment = 16;
//     int sampleIndex = 0;

//     QTimer timer;
//     QObject::connect(&timer, &QTimer::timeout, [&]()
//     {
//         ++sampleIndex;
//         if (sampleIndex > samplesPerSegment) sampleIndex = 0;

//         const double t = static_cast<double>(sampleIndex) / static_cast<double>(samplesPerSegment);
//         updateFrame(t);
//     });

//     timer.start(AnimationIntervalMs);
//     return app.exec();
// }




// int main(int argc, char* argv[])
// {
//     QApplication app(argc, argv);
//     MyBRep::Display::BRepViewerWidget window;
//     window.resize(1000, 700);

//     const std::vector<RevolvedSection> wheelSections = createGrindingWheelSections();
//     const std::vector<MyMath::CoordinateSystem> poses = createTestPoses();

//     std::size_t segmentIndex = 0;
//     const int envelopeMotionSamples = 16;
//     const int samplesPerSegment = 16;

//     RevolveBody wheelBody(wheelSections);

//     // 为完整有限B-Spline曲面建立参数域边界Wire和对应P-Curve。
//     auto createEnvelopeFace = [&](const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface>& surface) -> MyBRep::Face
//     {
//         if (!surface || surface->kind() != MyBRep::SurfaceKind::BSpline) return MyBRep::Face();

//         const MyBRep::Geometry_BSplineSurface& spline = static_cast<const MyBRep::Geometry_BSplineSurface&>(*surface);
//         const std::size_t uCount = spline.uControlPointCount();
//         const std::size_t vCount = spline.vControlPointCount();

//         const double u0 = spline.uDomainStart();
//         const double u1 = spline.uDomainEnd();
//         const double v0 = spline.vDomainStart();
//         const double v1 = spline.vDomainEnd();

//         const MyMath::Vector2 uv00(u0, v0);
//         const MyMath::Vector2 uv10(u1, v0);
//         const MyMath::Vector2 uv11(u1, v1);
//         const MyMath::Vector2 uv01(u0, v1);

//         const MyBRep::Topology_Vertex vertex00(surface->pointAt(u0, v0));
//         const MyBRep::Topology_Vertex vertex10(surface->pointAt(u1, v0));
//         const MyBRep::Topology_Vertex vertex11(surface->pointAt(u1, v1));
//         const MyBRep::Topology_Vertex vertex01(surface->pointAt(u0, v1));

//         std::vector<MyMath::Vector3> bottomPoints;
//         std::vector<MyMath::Vector3> topPoints;
//         std::vector<MyMath::Vector3> leftPoints;
//         std::vector<MyMath::Vector3> rightPoints;

//         bottomPoints.reserve(uCount);
//         topPoints.reserve(uCount);
//         leftPoints.reserve(vCount);
//         rightPoints.reserve(vCount);

//         for (std::size_t u = 0; u < uCount; ++u)
//         {
//             bottomPoints.push_back(spline.controlPoint(u, 0));
//             topPoints.push_back(spline.controlPoint(u, vCount - 1));
//         }

//         for (std::size_t v = 0; v < vCount; ++v)
//         {
//             leftPoints.push_back(spline.controlPoint(0, v));
//             rightPoints.push_back(spline.controlPoint(uCount - 1, v));
//         }

//         MyBRep::Topology_Edge bottom = MyBRep::Modeling::createBSpline(vertex00, vertex10, bottomPoints, spline.uDegree(), spline.uKnots(), SolidTolerance);
//         MyBRep::Topology_Edge right = MyBRep::Modeling::createBSpline(vertex10, vertex11, rightPoints, spline.vDegree(), spline.vKnots(), SolidTolerance);
//         MyBRep::Topology_Edge top = MyBRep::Modeling::createBSpline(vertex01, vertex11, topPoints, spline.uDegree(), spline.uKnots(), SolidTolerance);
//         MyBRep::Topology_Edge left = MyBRep::Modeling::createBSpline(vertex00, vertex01, leftPoints, spline.vDegree(), spline.vKnots(), SolidTolerance);

//         auto addCurveOnSurface = [&](MyBRep::Topology_Edge& edge, const MyMath::Vector2& firstUV, const MyMath::Vector2& lastUV)
//         {
//             const MyMath::Vector2 direction = lastUV - firstUV;
//             const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> curve(new MyBRep::Geometry_Line2D(firstUV, direction));
//             MyBRep::Topology_Builder::addCurveOnSurface(edge, surface, curve, 0.0, direction.length(), SolidTolerance);
//         };

//         addCurveOnSurface(bottom, uv00, uv10);
//         addCurveOnSurface(right, uv10, uv11);
//         addCurveOnSurface(top, uv01, uv11);
//         addCurveOnSurface(left, uv00, uv01);

//         std::vector<MyBRep::Topology_Edge> edges;
//         edges.reserve(4);
//         edges.push_back(bottom);
//         edges.push_back(right);
//         edges.push_back(top.reversed());
//         edges.push_back(left.reversed());

//         const MyBRep::Topology_Wire wire(edges);
//         return MyBRep::Modeling::makeFace(surface, std::vector<MyBRep::Topology_Wire>(1, wire));
//     };

//     // Plus候选包络面显示样式。
//     MyBRep::Display::BRepDisplayStyle plusSurfaceStyle;
//     plusSurfaceStyle.surfaceColor = QVector4D(1.0f, 0.55f, 0.50f, 0.45f);
//     plusSurfaceStyle.wireColor = QVector4D(0.65f, 0.05f, 0.03f, 1.0f);
//     plusSurfaceStyle.wireWidth = 1.0f;

//     // Minus候选包络面显示样式。
//     MyBRep::Display::BRepDisplayStyle minusSurfaceStyle;
//     minusSurfaceStyle.surfaceColor = QVector4D(0.45f, 0.60f, 1.0f, 0.45f);
//     minusSurfaceStyle.wireColor = QVector4D(0.04f, 0.12f, 0.65f, 1.0f);
//     minusSurfaceStyle.wireWidth = 1.0f;

//     MyBRep::Display::BRepDisplayId plusSurfaceDisplayId = MyBRep::Display::InvalidBRepDisplayId;
//     MyBRep::Display::BRepDisplayId minusSurfaceDisplayId = MyBRep::Display::InvalidBRepDisplayId;

//     // 构造并显示当前运动段的两张候选包络面。
//     auto rebuildEnvelopeSurfaces = [&]() -> bool
//     {
//         if (plusSurfaceDisplayId != MyBRep::Display::InvalidBRepDisplayId)
//         {
//             window.removeDisplay(plusSurfaceDisplayId);
//             plusSurfaceDisplayId = MyBRep::Display::InvalidBRepDisplayId;
//         }

//         if (minusSurfaceDisplayId != MyBRep::Display::InvalidBRepDisplayId)
//         {
//             window.removeDisplay(minusSurfaceDisplayId);
//             minusSurfaceDisplayId = MyBRep::Display::InvalidBRepDisplayId;
//         }

//         MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> plusSurface;
//         MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Surface> minusSurface;

//         if (!createEnvelopeSurfaces(wheelBody, poses[segmentIndex], poses[segmentIndex + 1], envelopeMotionSamples, plusSurface, minusSurface))
//             return false;

//         const MyBRep::Face plusFace = createEnvelopeFace(plusSurface);
//         const MyBRep::Face minusFace = createEnvelopeFace(minusSurface);
//         if (!plusFace.isValid() || !minusFace.isValid()) return false;

//         plusSurfaceDisplayId = window.addFace(plusFace, "PlusEnvelopeSurface", plusSurfaceStyle);
//         minusSurfaceDisplayId = window.addFace(minusFace, "MinusEnvelopeSurface", minusSurfaceStyle);

//         return plusSurfaceDisplayId != MyBRep::Display::InvalidBRepDisplayId &&
//                minusSurfaceDisplayId != MyBRep::Display::InvalidBRepDisplayId;
//     };

//     // 创建运动砂轮。
//     wheelBody.moveBetween(poses[0], poses[1], 0.0);

//     MyBRep::Solid grindingWheel = createRevolvedSolid(wheelSections, 64);
//     grindingWheel.setLocalToWorld(wheelBody.coordinateSystem().toMatrix());
//     window.addSolid(grindingWheel);

//     // 当前参数位置的Plus特征线。
//     MyBRep::Display::BRepDisplayStyle plusStyle;
//     plusStyle.wireColor = QVector4D(1.0f, 0.0f, 0.0f, 1.0f);
//     plusStyle.wireWidth = 3.0f;

//     // 当前参数位置的Minus特征线。
//     MyBRep::Display::BRepDisplayStyle minusStyle;
//     minusStyle.wireColor = QVector4D(0.0f, 0.2f, 1.0f, 1.0f);
//     minusStyle.wireWidth = 3.0f;

//     MyBRep::Display::BRepDisplayId plusDisplayId = MyBRep::Display::InvalidBRepDisplayId;
//     MyBRep::Display::BRepDisplayId minusDisplayId = MyBRep::Display::InvalidBRepDisplayId;

//     // 更新当前运动段参数位置的砂轮和两条特征线。
//     auto updateFrame = [&](double t)
//     {
//         wheelBody.moveBetween(poses[segmentIndex], poses[segmentIndex + 1], t);

//         grindingWheel.setLocalToWorld(wheelBody.coordinateSystem().toMatrix());
//         window.refreshPlacement(grindingWheel);

//         if (plusDisplayId != MyBRep::Display::InvalidBRepDisplayId)
//         {
//             window.removeDisplay(plusDisplayId);
//             plusDisplayId = MyBRep::Display::InvalidBRepDisplayId;
//         }

//         if (minusDisplayId != MyBRep::Display::InvalidBRepDisplayId)
//         {
//             window.removeDisplay(minusDisplayId);
//             minusDisplayId = MyBRep::Display::InvalidBRepDisplayId;
//         }

//         std::vector<MyMath::Vector3> plusPoints;
//         std::vector<MyMath::Vector3> minusPoints;
//         double maxConditionError = 0.0;

//         if (!wheelBody.collectCharacteristicPoints(plusPoints, minusPoints, maxConditionError)) return;

//         if (plusPoints.size() >= 2)
//         {
//             const MyBRep::Edge edge = MyBRep::Modeling::makeBSpline(plusPoints);
//             plusDisplayId = window.addWireframe(edge, "CharacteristicPlus", plusStyle);
//         }

//         if (minusPoints.size() >= 2)
//         {
//             const MyBRep::Edge edge = MyBRep::Modeling::makeBSpline(minusPoints);
//             minusDisplayId = window.addWireframe(edge, "CharacteristicMinus", minusStyle);
//         }
//     };

//     // 初始化第一运动段。
//     if (!rebuildEnvelopeSurfaces())
//     {
//         qDebug() << "Envelope surface construction failed at segment" << segmentIndex;
//         return -1;
//     }

//     updateFrame(0.0);

//     qDebug() << "Motion segment" << segmentIndex << "/" << poses.size() - 2;
//     qDebug() << "Plus envelope display =" << plusSurfaceDisplayId;
//     qDebug() << "Minus envelope display =" << minusSurfaceDisplayId;

//     window.show();
//     QTimer::singleShot(50, [&window]() { window.fitItemsToView(); });

//     int sampleIndex = 0;

//     QTimer timer;
//     QObject::connect(&timer, &QTimer::timeout, [&]()
//     {
//         ++sampleIndex;

//         if (sampleIndex > samplesPerSegment)
//         {
//             sampleIndex = 0;
//             ++segmentIndex;

//             if (segmentIndex + 1 >= poses.size())
//             {
//                 timer.stop();
//                 qDebug() << "All motion segments completed.";
//                 return;
//             }

//             if (!rebuildEnvelopeSurfaces())
//             {
//                 timer.stop();
//                 qDebug() << "Envelope surface construction failed at segment" << segmentIndex;
//                 return;
//             }

//             qDebug() << "Motion segment" << segmentIndex << "/" << poses.size() - 2;
//         }

//         const double t = static_cast<double>(sampleIndex)/static_cast<double>(samplesPerSegment);
//         updateFrame(t);
//     });

//     timer.start(AnimationIntervalMs);
//     return app.exec();
// }

// int main(int argc, char* argv[])
// {
//     QApplication app(argc, argv);
//     MyBRep::Display::BRepViewerWidget window;
//     window.resize(800, 600);

//     // MyBRep::Solid tool = createTool();

//     // window.addSolid(tool);
    
//     const std::vector<RevolvedSection> wheelSections = createGrindingWheelSections();
//     const std::vector<MyMath::CoordinateSystem> poses = createTestPoses();
//     RevolveBody wheelBody(wheelSections);
//     wheelBody.moveTo(poses.front());
//     MyBRep::Display::BRepDisplayStyle trajectoryStyle;
//     trajectoryStyle.wireColor = QVector4D(1.0f, 0.15f, 0.10f, 1.0f);                      // 使用明显的红色显示轨迹。
//     trajectoryStyle.wireWidth = 1.0f;                                                       // 增加轨迹线宽便于观察。
//     for(double z = wheelBody.m_sections.front().z; z <= wheelBody.m_sections.back().z; z += 0.2)
//     {
//         for(double angle = 0; angle <= MyMath::TwoPi; angle += MyMath::TwoPi / 60)
//         {

//         const std::vector<MyMath::Vector3> trajectoryPoints =buildPointTrajectory(wheelBody, poses, z, angle);          // 计算固定砂轮点的完整运动轨迹。
//         const MyBRep::Edge trajectory = MyBRep::Modeling::makeBSpline(trajectoryPoints);                                 // 将轨迹采样点连接成三维折线。
//         window.addWireframe(trajectory, "TrackedPointTrajectory", trajectoryStyle);
//         }
//         // const std::vector<MyMath::Vector3> trajectoryPoints =buildPointTrajectory(wheelBody, poses, z, TrackedPointAngle);          // 计算固定砂轮点的完整运动轨迹。
//         // const MyBRep::Edge trajectory = MyBRep::Modeling::makeBSpline(trajectoryPoints);                                 // 将轨迹采样点连接成三维折线。
//         // window.addWireframe(trajectory, "TrackedPointTrajectory", trajectoryStyle);
//     }
//     // for(double angle = 0; angle <= MyMath::TwoPi; angle += MyMath::TwoPi / 10)
//     // {
//     //     const std::vector<MyMath::Vector3> trajectoryPoints =buildPointTrajectory(wheelBody, poses, TrackedPointZ, angle);          // 计算固定砂轮点的完整运动轨迹。
//     //     const MyBRep::Edge trajectory = MyBRep::Modeling::makeBSpline(trajectoryPoints);                                 // 将轨迹采样点连接成三维折线。

//     //     window.addWireframe(trajectory, "TrackedPointTrajectory", trajectoryStyle);
//     // }
//     MyBRep::Solid grindingWheel = createRevolvedSolid(wheelSections, 64);
//     window.addSolid(grindingWheel);
    
//     window.show();
//     QTimer::singleShot(50, [&window]() { window.fitItemsToView(); });

//     QTimer timer;
//     int frameIndex = 0;

//     //动画定时器
//     QObject::connect(&timer, &QTimer::timeout, [&]()
//     {

//         frameIndex = (frameIndex + 1) % PoseCount;
//         grindingWheel.setLocalToWorld(poses[frameIndex].toMatrix());
//         window.refreshPlacement(grindingWheel);

//     });

//     timer.start(AnimationIntervalMs);
//     return app.exec();
// }
