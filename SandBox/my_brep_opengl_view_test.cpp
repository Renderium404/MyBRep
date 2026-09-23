#include <algorithm>
#include <cmath>
#include <vector>

#include <QApplication>
#include <QDebug>
#include <QString>
#include <QTimer>
#include <QVector3D>
#include <QVector4D>

#include "MyMath/CoordinateSystem.h"
#include "MyMath/MathUtils.h"
#include "MyMath/Quaternion.h"
#include "MyMath/Vector2.h"
#include "MyMath/Vector3.h"

#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Construction/Geometry_Revolved.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Curve2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Line2D.h"
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

const double SolidTolerance = 1.0e-8;                 // 测试回转体与B-Rep显示构造统一容差。
const int PoseCount = 100;                            // 已完成插值的刀路点位数量。
const int ProfileSamplesPerSegment = 32;              // 每条非退化母线段的包络采样区间数量。
const int ProfileDisplaySubdivisionCount = 8;         // 非直线母线仅用于砂轮显示时的离散区间数量。
const int EdgeArcSubdivisionCount = 12;               // sharp circular edge候选包络圆弧的离散区间数量。
const int MergeBisectionCount = 48;                   // H=0分支合并边界二分求根次数。
const double MergeDiscriminantTolerance = 1.0e-12;    // 分支合并判别式零值判断容差。
const int AnimationIntervalMs = 50;                   // 砂轮动画相邻刀路点显示间隔，单位ms。

typedef MyBRep::Geometry_Revolved::ProfileSegment ProfileSegment;

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

// 创建测试姿态；poses中的每个元素均视为已经完成刀路插值后的实际离散点位。
std::vector<MyMath::CoordinateSystem> createTestPoses()
{
    std::vector<MyMath::CoordinateSystem> poses;
    poses.reserve(PoseCount);

    for (int index = 0; index < PoseCount; ++index)
    {
        const double t = static_cast<double>(index)/static_cast<double>(PoseCount - 1);
        const double x = -55.0 + 110.0*t;
        const double y = 10.0*std::sin(MyMath::TwoPi*t);
        const double z = 6.0*std::sin(MyMath::Pi*t);

        const MyMath::Quaternion rotateX =
            MyMath::Quaternion::fromAxisAngle(MyMath::Vector3::unitX(), 0.18*std::sin(MyMath::TwoPi*t));
        const MyMath::Quaternion rotateY =
            MyMath::Quaternion::fromAxisAngle(MyMath::Vector3::unitY(), 0.35*std::sin(MyMath::Pi*t));
        const MyMath::Quaternion rotateZ =
            MyMath::Quaternion::fromAxisAngle(MyMath::Vector3::unitZ(), 0.25*std::sin(MyMath::TwoPi*t));
        const MyMath::Quaternion orientation = rotateZ*rotateY*rotateX;

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

double positiveAngleDifference(double from, double to)
{
    double difference = normalizeAngle(to) - normalizeAngle(from);
    if (difference <= 0.0) difference += MyMath::TwoPi;
    return difference;
}

// 创建一个有限有向二维直线母线段。
ProfileSegment createLineProfileSegment(const MyMath::Vector2& start, const MyMath::Vector2& end)
{
    const MyMath::Vector2 direction = end - start;
    const double length = direction.length();
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> curve(
        new MyBRep::Geometry_Line2D(start, direction));
    return ProfileSegment(curve, 0.0, length);
}

// 创建当前测试砂轮的完整闭合二维母线。
// X为带符号旋转半径，Y为砂轮局部Z。
MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Revolved> createGrindingWheelGeometry()
{
    const MyMath::Vector2 p0(0.0, -8.0);
    const MyMath::Vector2 p1(34.0, -8.0);
    const MyMath::Vector2 p2(40.0, -5.0);
    const MyMath::Vector2 p3(40.0, -2.5);
    const MyMath::Vector2 p4(40.0, 2.5);
    const MyMath::Vector2 p5(38.0, 5.0);
    const MyMath::Vector2 p6(34.0, 8.0);
    const MyMath::Vector2 p7(0.0, 8.0);

    std::vector<ProfileSegment> segments;
    segments.reserve(8);
    segments.push_back(createLineProfileSegment(p0, p1)); // MinZ截面母线。
    segments.push_back(createLineProfileSegment(p1, p2)); // 左侧工作面过渡。
    segments.push_back(createLineProfileSegment(p2, p3)); // 主外圆左段。
    segments.push_back(createLineProfileSegment(p3, p4)); // 主外圆中段。
    segments.push_back(createLineProfileSegment(p4, p5)); // 右侧工作面过渡。
    segments.push_back(createLineProfileSegment(p5, p6)); // 右侧内缩面。
    segments.push_back(createLineProfileSegment(p6, p7)); // MaxZ截面母线。
    segments.push_back(createLineProfileSegment(p7, p0)); // 回转轴闭合段。

    return MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Revolved>(
        new MyBRep::Geometry_Revolved(segments, SolidTolerance));
}

// 判断母线段是否完全位于回转轴上。
// 这类段绕Z轴旋转后退化，不建立独立包络曲面。
bool isAxisProfileSegment(const MyBRep::Geometry_Revolved& geometry, const ProfileSegment& segment)
{
    const MyMath::Vector2 start = segment.curve->pointAt(segment.firstParameter);
    const MyMath::Vector2 end = segment.curve->pointAt(segment.lastParameter);
    const double tolerance = geometry.profileTolerance();

    return std::fabs(start.x()) <= tolerance && std::fabs(end.x()) <= tolerance;
}

// 仅供砂轮B-Rep显示使用：识别水平端面母线。
// 包络计算不会过滤它们。
bool isPlanarEndProfileSegment(const ProfileSegment& segment)
{
    if (segment.curve->kind() != MyBRep::CurveKind::Line) return false;

    const double parameter = (segment.firstParameter + segment.lastParameter)*0.5;
    const MyMath::Vector2 derivative = segment.curve->firstDerivativeAt(parameter);
    return std::fabs(derivative.y()) <= MyMath::Vector2::DefaultEpsilon;
}

// 返回使用固定母线参数网格处理的ProfileSegment。
// 回转轴段退化；水平端面另由H=0自适应Branch Merge网格处理。
std::vector<std::size_t> smoothEnvelopeProfileSegmentIndices(const MyBRep::Geometry_Revolved& geometry)
{
    std::vector<std::size_t> indices;
    indices.reserve(geometry.profileSegmentCount());

    for (std::size_t index = 0; index < geometry.profileSegmentCount(); ++index)
    {
        const ProfileSegment& segment = geometry.profileSegment(index);
        if (isAxisProfileSegment(geometry, segment) || isPlanarEndProfileSegment(segment)) continue;
        indices.push_back(index);
    }

    return indices;
}

// 返回需要建立H=0 Branch Merge自适应网格的水平端面ProfileSegment。
std::vector<std::size_t> planarEndProfileSegmentIndices(const MyBRep::Geometry_Revolved& geometry)
{
    std::vector<std::size_t> indices;
    indices.reserve(2);

    for (std::size_t index = 0; index < geometry.profileSegmentCount(); ++index)
    {
        const ProfileSegment& segment = geometry.profileSegment(index);
        if (isAxisProfileSegment(geometry, segment) || !isPlanarEndProfileSegment(segment)) continue;
        indices.push_back(index);
    }

    return indices;
}

// 返回有限有向母线段在规范化参数u方向对应的物理(r,z)切向量。
MyMath::Vector2 physicalProfileTangent(const MyBRep::Geometry_Revolved& geometry,
                                       const ProfileSegment& segment,
                                       double parameter)
{
    const double parameterSpan = segment.lastParameter - segment.firstParameter;
    const MyMath::Vector2 sourceDerivative = segment.curve->firstDerivativeAt(parameter)*parameterSpan;
    return MyMath::Vector2(geometry.radialSign()*sourceDerivative.x(), sourceDerivative.y());
}

// 返回当前母线点的二维单位外法向，约定x=n_z、y=n_r。
// 法向只由当前ProfileSegment自身的有向切向确定，不在母线拐角处做法向平均。
MyMath::Vector2 profileNormal(const MyBRep::Geometry_Revolved& geometry,
                              const ProfileSegment& segment,
                              double parameter)
{
    const MyMath::Vector2 tangent = physicalProfileTangent(geometry, segment, parameter);
    const double length = tangent.length();
    if (length <= MyMath::Vector2::DefaultEpsilon) return MyMath::Vector2::zero();

    // x_signed -> r的映射在radialSign<0时会翻转二维区域方向。
    const double physicalSignedArea = geometry.profileSignedArea()*geometry.radialSign();
    const double orientationSign = physicalSignedArea >= 0.0 ? 1.0 : -1.0;

    const double normalRadial = orientationSign*tangent.y()/length;
    const double normalAxial = -orientationSign*tangent.x()/length;
    return MyMath::Vector2(normalAxial, normalRadial);
}

struct EnvelopeProfileSample
{
    EnvelopeProfileSample() : parameter(0.0), radius(0.0), z(0.0), normal(MyMath::Vector2::zero()) {}

    double parameter;
    double radius;
    double z;
    MyMath::Vector2 normal; // x=n_z，y=n_r。
};

bool buildProfileSampleAtParameter(const MyBRep::Geometry_Revolved& geometry,
                                   const ProfileSegment& segment,
                                   double parameter,
                                   EnvelopeProfileSample& sample)
{
    if (!segment.curve || !segment.curve->isParameterInDomain(parameter)) return false;

    const MyMath::Vector2 profilePoint = segment.curve->pointAt(parameter);
    const MyMath::Vector2 normal = profileNormal(geometry, segment, parameter);

    if (!profilePoint.isFinite() || !normal.isFinite() ||
        normal.length() <= MyMath::Vector2::DefaultEpsilon)
    {
        return false;
    }

    const double radius = geometry.radialSign()*profilePoint.x();
    if (radius < -geometry.profileTolerance()) return false;

    sample.parameter = parameter;
    sample.radius = std::fabs(radius) <= geometry.profileTolerance() ? 0.0 : radius;
    sample.z = profilePoint.y();
    sample.normal = normal;
    return true;
}

// 对一条普通ProfileSegment独立建立固定参数母线采样。
bool buildProfileSegmentSamples(const MyBRep::Geometry_Revolved& geometry,
                                std::size_t segmentIndex,
                                int samplesPerSegment,
                                std::vector<EnvelopeProfileSample>& samples)
{
    samples.clear();
    if (segmentIndex >= geometry.profileSegmentCount() || samplesPerSegment <= 0) return false;

    const ProfileSegment& segment = geometry.profileSegment(segmentIndex);
    if (isAxisProfileSegment(geometry, segment)) return false;

    samples.reserve(static_cast<std::size_t>(samplesPerSegment + 1));

    for (int sampleIndex = 0; sampleIndex <= samplesPerSegment; ++sampleIndex)
    {
        const double u = static_cast<double>(sampleIndex)/static_cast<double>(samplesPerSegment);
        const double parameter = segment.firstParameter + (segment.lastParameter - segment.firstParameter)*u;
        EnvelopeProfileSample sample;
        if (!buildProfileSampleAtParameter(geometry, segment, parameter, sample)) return false;
        samples.push_back(sample);
    }

    return samples.size() >= 2;
}

// 表示完整二维母线一个非退化sharp顶点；绕Z轴旋转后形成sharp circular edge。
struct EnvelopeProfileVertex
{
    EnvelopeProfileVertex()
        : firstSegmentIndex(0), secondSegmentIndex(0), radius(0.0), z(0.0),
          firstNormal(MyMath::Vector2::zero()), secondNormal(MyMath::Vector2::zero())
    {
    }

    std::size_t firstSegmentIndex;
    std::size_t secondSegmentIndex;
    double radius;
    double z;
    MyMath::Vector2 firstNormal;  // x=n_z，y=n_r。
    MyMath::Vector2 secondNormal; // x=n_z，y=n_r。
};

// 收集需要独立建立圆边包络的母线sharp顶点。
// 半径为0的轴上顶点退化成点，不在当前CircularEdge阶段建立曲面；相邻法向一致的光滑连接也跳过。
bool buildEnvelopeProfileVertices(const MyBRep::Geometry_Revolved& geometry,
                                  std::vector<EnvelopeProfileVertex>& vertices)
{
    vertices.clear();

    const std::size_t segmentCount = geometry.profileSegmentCount();
    if (segmentCount < 2) return false;

    for (std::size_t firstIndex = 0; firstIndex < segmentCount; ++firstIndex)
    {
        const std::size_t secondIndex = (firstIndex + 1)%segmentCount;
        const ProfileSegment& first = geometry.profileSegment(firstIndex);
        const ProfileSegment& second = geometry.profileSegment(secondIndex);

        if (isAxisProfileSegment(geometry, first) || isAxisProfileSegment(geometry, second)) continue;

        const MyMath::Vector2 firstEnd = first.curve->pointAt(first.lastParameter);
        const MyMath::Vector2 secondStart = second.curve->pointAt(second.firstParameter);
        if (!firstEnd.isEqualTo(secondStart, geometry.profileTolerance())) return false;

        const double radius = geometry.radialSign()*firstEnd.x();
        if (radius <= geometry.profileTolerance()) continue;

        const MyMath::Vector2 firstNormal = profileNormal(geometry, first, first.lastParameter);
        const MyMath::Vector2 secondNormal = profileNormal(geometry, second, second.firstParameter);
        if (firstNormal.length() <= MyMath::Vector2::DefaultEpsilon ||
            secondNormal.length() <= MyMath::Vector2::DefaultEpsilon) return false;

        const double normalCross = std::fabs(MyMath::Vector2::cross(firstNormal, secondNormal));
        const double normalDot = MyMath::Vector2::dot(firstNormal, secondNormal);
        if (normalCross <= MyMath::Vector2::DefaultEpsilon && normalDot > 0.0) continue;

        EnvelopeProfileVertex vertex;
        vertex.firstSegmentIndex = firstIndex;
        vertex.secondSegmentIndex = secondIndex;
        vertex.radius = radius;
        vertex.z = firstEnd.y();
        vertex.firstNormal = firstNormal;
        vertex.secondNormal = secondNormal;
        vertices.push_back(vertex);
    }

    return !vertices.empty();
}

// 仅用于砂轮动画显示：返回外侧母线，不包括上下端面和回转轴闭合段。
std::vector<std::size_t> displayOuterProfileSegmentIndices(const MyBRep::Geometry_Revolved& geometry)
{
    std::vector<std::size_t> indices;
    indices.reserve(geometry.profileSegmentCount());

    for (std::size_t index = 0; index < geometry.profileSegmentCount(); ++index)
    {
        const ProfileSegment& segment = geometry.profileSegment(index);
        if (isAxisProfileSegment(geometry, segment) || isPlanarEndProfileSegment(segment)) continue;
        indices.push_back(index);
    }

    return indices;
}

// 仅用于砂轮动画显示：从Geometry_Revolved外侧母线生成离散(r,z)点。
bool buildDisplayOuterProfile(const MyBRep::Geometry_Revolved& geometry, std::vector<MyMath::Vector2>& points)
{
    points.clear();

    const std::vector<std::size_t> indices = displayOuterProfileSegmentIndices(geometry);
    if (indices.empty()) return false;

    for (std::size_t segmentPosition = 0; segmentPosition < indices.size(); ++segmentPosition)
    {
        const ProfileSegment& segment = geometry.profileSegment(indices[segmentPosition]);
        const int subdivisions = segment.curve->kind() == MyBRep::CurveKind::Line ? 1 : ProfileDisplaySubdivisionCount;

        for (int sampleIndex = 0; sampleIndex <= subdivisions; ++sampleIndex)
        {
            if (segmentPosition > 0 && sampleIndex == 0) continue;

            const double u = static_cast<double>(sampleIndex)/static_cast<double>(subdivisions);
            const double parameter = segment.firstParameter + (segment.lastParameter - segment.firstParameter)*u;
            const MyMath::Vector2 point = segment.curve->pointAt(parameter);
            points.push_back(MyMath::Vector2(geometry.radialSign()*point.x(), point.y()));
        }
    }

    if (points.size() < 2) return false;
    if (points.front().y() > points.back().y()) std::reverse(points.begin(), points.end());
    return true;
}

// 仅用于砂轮动画显示：将外侧母线离散点旋转成闭合B-Rep Solid。
MyBRep::Solid createRevolvedDisplaySolid(const MyBRep::Geometry_Revolved& geometry, int sideCount)
{
    std::vector<MyMath::Vector2> profile;
    if (!buildDisplayOuterProfile(geometry, profile) || sideCount < 3) return MyBRep::Solid();

    const std::size_t sectionCount = profile.size();
    std::vector<std::vector<MyBRep::Topology_Vertex> > vertices(sectionCount);

    for (std::size_t sectionIndex = 0; sectionIndex < sectionCount; ++sectionIndex)
    {
        vertices[sectionIndex].reserve(sideCount);

        for (int sideIndex = 0; sideIndex < sideCount; ++sideIndex)
        {
            const double angle = MyMath::TwoPi*static_cast<double>(sideIndex)/static_cast<double>(sideCount);
            const double radius = profile[sectionIndex].x();
            const double z = profile[sectionIndex].y();

            vertices[sectionIndex].push_back(
                MyBRep::Topology_Vertex(MyMath::Vector3(radius*std::cos(angle), radius*std::sin(angle), z)));
        }
    }

    std::vector<std::vector<MyBRep::Topology_Edge> > ringEdges(sectionCount);

    for (std::size_t sectionIndex = 0; sectionIndex < sectionCount; ++sectionIndex)
    {
        ringEdges[sectionIndex].reserve(sideCount);

        for (int sideIndex = 0; sideIndex < sideCount; ++sideIndex)
        {
            const int nextIndex = (sideIndex + 1)%sideCount;
            ringEdges[sectionIndex].push_back(
                MyBRep::Modeling::createLine(vertices[sectionIndex][sideIndex], vertices[sectionIndex][nextIndex]));
        }
    }

    std::vector<std::vector<MyBRep::Topology_Edge> > longitudinalEdges(sectionCount - 1);

    for (std::size_t sectionIndex = 0; sectionIndex + 1 < sectionCount; ++sectionIndex)
    {
        longitudinalEdges[sectionIndex].reserve(sideCount);

        for (int sideIndex = 0; sideIndex < sideCount; ++sideIndex)
        {
            longitudinalEdges[sectionIndex].push_back(
                MyBRep::Modeling::createLine(vertices[sectionIndex][sideIndex], vertices[sectionIndex + 1][sideIndex]));
        }
    }

    std::vector<MyBRep::Topology_Face> faces;
    faces.reserve((sectionCount - 1)*static_cast<std::size_t>(sideCount) + 2);

    for (std::size_t sectionIndex = 0; sectionIndex + 1 < sectionCount; ++sectionIndex)
    {
        for (int sideIndex = 0; sideIndex < sideCount; ++sideIndex)
        {
            const int nextIndex = (sideIndex + 1)%sideCount;

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

            faces.push_back(MyBRep::Modeling::createPlanarFace(
                coordinateSystem, MyBRep::Modeling::createWire(edges), SolidTolerance));
        }
    }

    // MinZ实体端面。
    {
        std::vector<MyBRep::Topology_Edge> edges;
        edges.reserve(sideCount);
        for (int sideIndex = sideCount - 1; sideIndex >= 0; --sideIndex) edges.push_back(ringEdges.front()[sideIndex].reversed());

        const MyMath::CoordinateSystem coordinateSystem = MyMath::CoordinateSystem::fromAxes(
            MyMath::Vector3(0.0, 0.0, profile.front().y()),
            MyMath::Vector3::unitX(),
            MyMath::Vector3(0.0, -1.0, 0.0),
            MyMath::Vector3(0.0, 0.0, -1.0));

        faces.push_back(MyBRep::Modeling::createPlanarFace(
            coordinateSystem, MyBRep::Modeling::createWire(edges), SolidTolerance));
    }

    // MaxZ实体端面。
    {
        std::vector<MyBRep::Topology_Edge> edges;
        edges.reserve(sideCount);
        for (int sideIndex = 0; sideIndex < sideCount; ++sideIndex) edges.push_back(ringEdges.back()[sideIndex]);

        const MyMath::CoordinateSystem coordinateSystem = MyMath::CoordinateSystem::fromAxes(
            MyMath::Vector3(0.0, 0.0, profile.back().y()),
            MyMath::Vector3::unitX(),
            MyMath::Vector3::unitY(),
            MyMath::Vector3::unitZ());

        faces.push_back(MyBRep::Modeling::createPlanarFace(
            coordinateSystem, MyBRep::Modeling::createWire(edges), SolidTolerance));
    }

    return MyBRep::Modeling::makeSolid(MyBRep::Modeling::createShell(faces));
}

class RevolveBody
{
public:
    explicit RevolveBody(const MyBRep::Geometry_Revolved& geometry)
        : m_geometry(&geometry),
          m_coordinateSystem(),
          m_translationDerivativeWorld(MyMath::Vector3::zero()),
          m_w(MyMath::Vector3::zero())
    {
    }

    RevolveBody(const RevolveBody& other) = default;
    RevolveBody& operator=(const RevolveBody& other) = default;
    ~RevolveBody() = default;

    const MyBRep::Geometry_Revolved& geometry() const { return *m_geometry; }

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

    // u=R^T*p'。
    MyMath::Vector3 u() const
    {
        return MyMath::Vector3(
            MyMath::Vector3::dot(m_coordinateSystem.xAxis(), m_translationDerivativeWorld),
            MyMath::Vector3::dot(m_coordinateSystem.yAxis(), m_translationDerivativeWorld),
            MyMath::Vector3::dot(m_coordinateSystem.zAxis(), m_translationDerivativeWorld));
    }

    MyMath::Vector3 getPointLocal(const EnvelopeProfileSample& profile, double angle) const
    {
        return MyMath::Vector3(profile.radius*std::cos(angle), profile.radius*std::sin(angle), profile.z);
    }

    MyMath::Vector3 getPointGlobal(const EnvelopeProfileSample& profile, double angle) const
    {
        return m_coordinateSystem.toGlobal(getPointLocal(profile, angle));
    }

    // W(q)=u+w×q。
    MyMath::Vector3 W(const EnvelopeProfileSample& profile, double angle) const
    {
        return u() + MyMath::Vector3::cross(m_w, getPointLocal(profile, angle));
    }

    // profile.normal约定x=n_z、y=n_r。
    MyMath::Vector3 n(const EnvelopeProfileSample& profile, double angle) const
    {
        return MyMath::Vector3(
            profile.normal.y()*std::cos(angle),
            profile.normal.y()*std::sin(angle),
            profile.normal.x());
    }

    double characteristicValue(const EnvelopeProfileSample& profile, double angle) const
    {
        return MyMath::Vector3::dot(n(profile, angle), W(profile, angle));
    }

    void characteristicEquation(const EnvelopeProfileSample& profile, double& A, double& B, double& C) const
    {
        const MyMath::Vector3 localU = u();
        const double normalAxial = profile.normal.x();
        const double normalRadial = profile.normal.y();
        const double k = normalRadial*profile.z - normalAxial*profile.radius;

        A = normalRadial*localU.x() + k*m_w.y();
        B = normalRadial*localU.y() - k*m_w.x();
        C = normalAxial*localU.z();
    }

    // H=A^2+B^2-C^2；H>0有两个实根，H=0两分支合并，H<0无实根。
    double characteristicDiscriminant(const EnvelopeProfileSample& profile) const
    {
        double A = 0.0;
        double B = 0.0;
        double C = 0.0;
        characteristicEquation(profile, A, B, C);
        return A*A + B*B - C*C;
    }

    // 求当前母线采样点满足n·W=0的两个圆周角。
    bool calculateAngles(const EnvelopeProfileSample& profile, double& plusAngle, double& minusAngle) const
    {
        const double epsilon = 1.0e-10;
        double A = 0.0;
        double B = 0.0;
        double C = 0.0;
        characteristicEquation(profile, A, B, C);
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

    // 求当前母线采样点的Plus和Minus候选包络点。
    bool characteristicPoints(const EnvelopeProfileSample& profile,
                              MyMath::Vector3& plusPoint,
                              MyMath::Vector3& minusPoint,
                              double& plusConditionError,
                              double& minusConditionError) const
    {
        double plusAngle = 0.0;
        double minusAngle = 0.0;
        if (!calculateAngles(profile, plusAngle, minusAngle)) return false;

        const MyMath::Vector3 plusNormal = n(profile, plusAngle);
        const MyMath::Vector3 plusW = W(profile, plusAngle);
        const double plusScale = plusNormal.length()*plusW.length();

        const MyMath::Vector3 minusNormal = n(profile, minusAngle);
        const MyMath::Vector3 minusW = W(profile, minusAngle);
        const double minusScale = minusNormal.length()*minusW.length();

        plusConditionError =
            plusScale > 0.0 ? std::fabs(MyMath::Vector3::dot(plusNormal, plusW))/plusScale : 0.0;
        minusConditionError =
            minusScale > 0.0 ? std::fabs(MyMath::Vector3::dot(minusNormal, minusW))/minusScale : 0.0;

        plusPoint = getPointGlobal(profile, plusAngle);
        minusPoint = getPointGlobal(profile, minusAngle);
        return true;
    }

    const MyMath::CoordinateSystem& coordinateSystem() const { return m_coordinateSystem; }

private:
    const MyBRep::Geometry_Revolved* m_geometry;
    MyMath::CoordinateSystem m_coordinateSystem;
    MyMath::Vector3 m_translationDerivativeWorld;
    MyMath::Vector3 m_w;
};

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

    const MyMath::Vector3 forwardRotation =
        rotationParameterIncrement(poses[poseIndex], poses[poseIndex + 1]);
    const MyMath::Vector3 backwardRotation =
        rotationParameterIncrement(poses[poseIndex], poses[poseIndex - 1]);

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

// 每个ProfileSegment拥有独立的(u,pose)采样网格。
// 不跨母线拐角共享法向，也不把不同母线段强行拼成一张参数面。
struct EnvelopeSegmentGrid
{
    EnvelopeSegmentGrid()
        : profileSegmentIndex(0), uCount(0), vCount(0), invalidSampleCount(0), maxConditionError(0.0)
    {
    }

    std::size_t index(std::size_t u, std::size_t v) const { return v*uCount + u; }
    const EnvelopeSample& sample(std::size_t u, std::size_t v) const { return samples[index(u, v)]; }
    EnvelopeSample& sample(std::size_t u, std::size_t v) { return samples[index(u, v)]; }

    std::size_t profileSegmentIndex;
    std::vector<EnvelopeProfileSample> profileSamples;
    std::vector<EnvelopeSample> samples;
    std::size_t uCount;
    std::size_t vCount;
    std::size_t invalidSampleCount;
    double maxConditionError;
};

// 为普通非水平ProfileSegment建立固定参数网格。
// 水平端面ProfileSegment由后续H=0 Branch Merge自适应网格负责。
bool collectEnvelopeSegmentGrids(RevolveBody body,
                                 const std::vector<MyMath::CoordinateSystem>& poses,
                                 int profileSamplesPerSegment,
                                 std::vector<EnvelopeSegmentGrid>& grids)
{
    grids.clear();
    if (poses.size() < 2 || profileSamplesPerSegment <= 0) return false;

    const std::vector<std::size_t> segmentIndices = smoothEnvelopeProfileSegmentIndices(body.geometry());
    if (segmentIndices.empty()) return false;

    grids.reserve(segmentIndices.size());

    for (std::size_t index = 0; index < segmentIndices.size(); ++index)
    {
        EnvelopeSegmentGrid grid;
        grid.profileSegmentIndex = segmentIndices[index];

        if (!buildProfileSegmentSamples(
                body.geometry(), grid.profileSegmentIndex, profileSamplesPerSegment, grid.profileSamples))
        {
            return false;
        }

        grid.uCount = grid.profileSamples.size();
        grid.vCount = poses.size();
        grid.samples.resize(grid.uCount*grid.vCount);
        grids.push_back(grid);
    }

    std::size_t validSampleCount = 0;

    for (std::size_t poseIndex = 0; poseIndex < poses.size(); ++poseIndex)
    {
        MyMath::Vector3 translationDerivativeWorld;
        MyMath::Vector3 rotationDerivativeLocal;

        if (!motionDerivativeAtPose(poses, poseIndex, translationDerivativeWorld, rotationDerivativeLocal)) return false;
        body.setMotionState(poses[poseIndex], translationDerivativeWorld, rotationDerivativeLocal);

        for (std::size_t gridIndex = 0; gridIndex < grids.size(); ++gridIndex)
        {
            EnvelopeSegmentGrid& grid = grids[gridIndex];

            for (std::size_t u = 0; u < grid.uCount; ++u)
            {
                const EnvelopeProfileSample& profile = grid.profileSamples[u];
                EnvelopeSample& sample = grid.sample(u, poseIndex);

                double plusConditionError = 0.0;
                double minusConditionError = 0.0;

                sample.valid = body.characteristicPoints(
                    profile, sample.plusPoint, sample.minusPoint, plusConditionError, minusConditionError);

                if (sample.valid)
                {
                    ++validSampleCount;
                    grid.maxConditionError = (std::max)(grid.maxConditionError, plusConditionError);
                    grid.maxConditionError = (std::max)(grid.maxConditionError, minusConditionError);
                    continue;
                }

                ++grid.invalidSampleCount;

                qDebug() << "Characteristic unavailable:"
                         << "pose =" << poseIndex
                         << "profileSegment =" << grid.profileSegmentIndex
                         << "parameter =" << profile.parameter
                         << "radius =" << profile.radius
                         << "z =" << profile.z;
            }
        }
    }

    return validSampleCount > 0;
}

// 水平端面在每个pose上的有效域一般从H=0合并半径开始，一直到外圆sharp edge。
// u=0强制为Plus/Minus共同merge点，u=1为端面外圆边界，因此两分支不会在有效域内侧撕裂。
struct EnvelopePlanarEndGrid
{
    EnvelopePlanarEndGrid()
        : profileSegmentIndex(0), uCount(0), vCount(0), validRowCount(0),
          invalidRowCount(0), maxConditionError(0.0)
    {
    }

    std::size_t index(std::size_t u, std::size_t v) const { return v*uCount + u; }
    const EnvelopeSample& sample(std::size_t u, std::size_t v) const { return samples[index(u, v)]; }
    EnvelopeSample& sample(std::size_t u, std::size_t v) { return samples[index(u, v)]; }

    std::size_t profileSegmentIndex;
    std::size_t uCount;
    std::size_t vCount;
    std::vector<EnvelopeSample> samples;
    std::vector<unsigned char> validRows;
    std::vector<double> mergeParameters;
    std::size_t validRowCount;
    std::size_t invalidRowCount;
    double maxConditionError;
};

// 返回水平端面母线的轴端参数和外圆端参数。
bool planarEndParameterRange(const MyBRep::Geometry_Revolved& geometry,
                             const ProfileSegment& segment,
                             double& axisParameter,
                             double& outerParameter)
{
    EnvelopeProfileSample first;
    EnvelopeProfileSample last;

    if (!buildProfileSampleAtParameter(geometry, segment, segment.firstParameter, first) ||
        !buildProfileSampleAtParameter(geometry, segment, segment.lastParameter, last))
    {
        return false;
    }

    if (first.radius <= last.radius)
    {
        axisParameter = segment.firstParameter;
        outerParameter = segment.lastParameter;
    }
    else
    {
        axisParameter = segment.lastParameter;
        outerParameter = segment.firstParameter;
    }

    return true;
}

// 在一条水平端面母线上寻找H=0分支合并参数。
// 返回的mergeParameter取H>=0一侧极限，因此后续calculateAngles不会因为舍入误差落入无解侧。
bool solvePlanarEndMergeParameter(const RevolveBody& body,
                                  const MyBRep::Geometry_Revolved& geometry,
                                  const ProfileSegment& segment,
                                  double axisParameter,
                                  double outerParameter,
                                  double& mergeParameter,
                                  bool& mergeAtAxis)
{
    mergeAtAxis = false;

    EnvelopeProfileSample axisProfile;
    EnvelopeProfileSample outerProfile;

    if (!buildProfileSampleAtParameter(geometry, segment, axisParameter, axisProfile) ||
        !buildProfileSampleAtParameter(geometry, segment, outerParameter, outerProfile))
    {
        return false;
    }

    const double axisH = body.characteristicDiscriminant(axisProfile);
    const double outerH = body.characteristicDiscriminant(outerProfile);

    // 整个端面在当前pose都没有实特征根。
    if (outerH < -MergeDiscriminantTolerance) return false;

    // H在轴点已经到达0；这里theta退化，但空间点唯一，直接把轴心作为Plus/Minus公共merge点。
    if (axisH >= -MergeDiscriminantTolerance)
    {
        mergeParameter = axisParameter;
        mergeAtAxis = true;
        return true;
    }

    double invalidParameter = axisParameter;
    double validParameter = outerParameter;

    EnvelopeProfileSample middleProfile;

    for (int iteration = 0; iteration < MergeBisectionCount; ++iteration)
    {
        const double middleParameter = (invalidParameter + validParameter)*0.5;

        if (!buildProfileSampleAtParameter(geometry, segment, middleParameter, middleProfile)) return false;

        const double middleH = body.characteristicDiscriminant(middleProfile);

        if (middleH >= 0.0)
        {
            validParameter = middleParameter;
        }
        else
        {
            invalidParameter = middleParameter;
        }
    }

    mergeParameter = validParameter;
    return true;
}

// 构造全部水平端面的自适应Plus/Minus网格。
// 每个pose独立求mergeParameter，然后在[merge,outer]上重新均匀采样。
bool collectEnvelopePlanarEndGrids(RevolveBody body,
                                   const std::vector<MyMath::CoordinateSystem>& poses,
                                   int profileSamplesPerSegment,
                                   std::vector<EnvelopePlanarEndGrid>& grids)
{
    grids.clear();
    if (poses.size() < 2 || profileSamplesPerSegment <= 0) return false;

    const std::vector<std::size_t> segmentIndices = planarEndProfileSegmentIndices(body.geometry());
    if (segmentIndices.empty()) return true;

    grids.reserve(segmentIndices.size());

    for (std::size_t index = 0; index < segmentIndices.size(); ++index)
    {
        EnvelopePlanarEndGrid grid;
        grid.profileSegmentIndex = segmentIndices[index];
        grid.uCount = static_cast<std::size_t>(profileSamplesPerSegment + 1);
        grid.vCount = poses.size();
        grid.samples.resize(grid.uCount*grid.vCount);
        grid.validRows.assign(grid.vCount, 0);
        grid.mergeParameters.assign(grid.vCount, 0.0);
        grids.push_back(grid);
    }

    for (std::size_t poseIndex = 0; poseIndex < poses.size(); ++poseIndex)
    {
        MyMath::Vector3 translationDerivativeWorld;
        MyMath::Vector3 rotationDerivativeLocal;

        if (!motionDerivativeAtPose(poses, poseIndex, translationDerivativeWorld, rotationDerivativeLocal)) return false;
        body.setMotionState(poses[poseIndex], translationDerivativeWorld, rotationDerivativeLocal);

        for (std::size_t gridIndex = 0; gridIndex < grids.size(); ++gridIndex)
        {
            EnvelopePlanarEndGrid& grid = grids[gridIndex];
            const ProfileSegment& segment = body.geometry().profileSegment(grid.profileSegmentIndex);

            double axisParameter = 0.0;
            double outerParameter = 0.0;

            if (!planarEndParameterRange(
                    body.geometry(), segment, axisParameter, outerParameter))
            {
                return false;
            }

            double mergeParameter = 0.0;
            bool mergeAtAxis = false;

            if (!solvePlanarEndMergeParameter(
                    body, body.geometry(), segment,
                    axisParameter, outerParameter,
                    mergeParameter, mergeAtAxis))
            {
                ++grid.invalidRowCount;
                continue;
            }

            grid.mergeParameters[poseIndex] = mergeParameter;

            EnvelopeProfileSample mergeProfile;
            if (!buildProfileSampleAtParameter(
                    body.geometry(), segment, mergeParameter, mergeProfile))
            {
                return false;
            }

            // u=0：Plus和Minus必须使用完全相同的空间点。
            EnvelopeSample& mergeSample = grid.sample(0, poseIndex);

            if (mergeAtAxis)
            {
                const MyMath::Vector3 mergePoint =
                    poses[poseIndex].toGlobal(MyMath::Vector3(0.0, 0.0, mergeProfile.z));

                mergeSample.valid = true;
                mergeSample.plusPoint = mergePoint;
                mergeSample.minusPoint = mergePoint;
            }
            else
            {
                MyMath::Vector3 plusPoint;
                MyMath::Vector3 minusPoint;
                double plusConditionError = 0.0;
                double minusConditionError = 0.0;

                if (!body.characteristicPoints(
                        mergeProfile,
                        plusPoint,
                        minusPoint,
                        plusConditionError,
                        minusConditionError))
                {
                    ++grid.invalidRowCount;
                    continue;
                }

                // H=0时两根理论上重合；取两点中值消除二分与三角函数舍入差异，强制形成共同拓扑边界。
                const MyMath::Vector3 mergePoint = (plusPoint + minusPoint)*0.5;

                mergeSample.valid = true;
                mergeSample.plusPoint = mergePoint;
                mergeSample.minusPoint = mergePoint;

                grid.maxConditionError = (std::max)(grid.maxConditionError, plusConditionError);
                grid.maxConditionError = (std::max)(grid.maxConditionError, minusConditionError);
            }

            bool rowValid = true;

            for (std::size_t u = 1; u < grid.uCount; ++u)
            {
                const double lambda =
                    static_cast<double>(u)/static_cast<double>(grid.uCount - 1);

                const double parameter =
                    mergeParameter + (outerParameter - mergeParameter)*lambda;

                EnvelopeProfileSample profile;
                if (!buildProfileSampleAtParameter(
                        body.geometry(), segment, parameter, profile))
                {
                    rowValid = false;
                    break;
                }

                EnvelopeSample& sample = grid.sample(u, poseIndex);
                double plusConditionError = 0.0;
                double minusConditionError = 0.0;

                sample.valid = body.characteristicPoints(
                    profile,
                    sample.plusPoint,
                    sample.minusPoint,
                    plusConditionError,
                    minusConditionError);

                if (!sample.valid)
                {
                    rowValid = false;
                    break;
                }

                grid.maxConditionError = (std::max)(grid.maxConditionError, plusConditionError);
                grid.maxConditionError = (std::max)(grid.maxConditionError, minusConditionError);
            }

            if (!rowValid)
            {
                for (std::size_t u = 0; u < grid.uCount; ++u) grid.sample(u, poseIndex).valid = false;
                ++grid.invalidRowCount;
                continue;
            }

            grid.validRows[poseIndex] = 1;
            ++grid.validRowCount;
        }
    }

    return true;
}

enum EdgeRootTag
{
    EdgeRootFirstPlus = 0,
    EdgeRootFirstMinus = 1,
    EdgeRootSecondPlus = 2,
    EdgeRootSecondMinus = 3
};

struct EdgeRoot
{
    EdgeRoot() : angle(0.0), tag(EdgeRootFirstPlus) {}
    EdgeRoot(double angleValue, EdgeRootTag tagValue) : angle(normalizeAngle(angleValue)), tag(tagValue) {}

    double angle;
    EdgeRootTag tag;
};

struct EdgeArc
{
    EdgeArc() : key(-1), startAngle(0.0), sweep(0.0) {}

    int key;
    double startAngle;
    double sweep;
};

// 一个sharp circular edge在某一pose上可能产生最多两个有效法向锥圆弧。
// 圆弧区间由g1(theta)*g2(theta)<=0判定，其中g1/g2分别来自相邻两个面的外法向。
bool solveEdgeArcs(const RevolveBody& body,
                   const EnvelopeProfileVertex& vertex,
                   std::vector<EdgeArc>& arcs)
{
    arcs.clear();

    EnvelopeProfileSample firstProfile;
    firstProfile.radius = vertex.radius;
    firstProfile.z = vertex.z;
    firstProfile.normal = vertex.firstNormal;

    EnvelopeProfileSample secondProfile = firstProfile;
    secondProfile.normal = vertex.secondNormal;

    double firstPlus = 0.0;
    double firstMinus = 0.0;
    double secondPlus = 0.0;
    double secondMinus = 0.0;

    if (!body.calculateAngles(firstProfile, firstPlus, firstMinus) ||
        !body.calculateAngles(secondProfile, secondPlus, secondMinus))
    {
        return false;
    }

    std::vector<EdgeRoot> roots;
    roots.reserve(4);
    roots.push_back(EdgeRoot(firstPlus, EdgeRootFirstPlus));
    roots.push_back(EdgeRoot(firstMinus, EdgeRootFirstMinus));
    roots.push_back(EdgeRoot(secondPlus, EdgeRootSecondPlus));
    roots.push_back(EdgeRoot(secondMinus, EdgeRootSecondMinus));

    std::sort(roots.begin(), roots.end(),
        [](const EdgeRoot& first, const EdgeRoot& second)
        {
            if (first.angle != second.angle) return first.angle < second.angle;
            return static_cast<int>(first.tag) < static_cast<int>(second.tag);
        });

    const double angleTolerance = 1.0e-9;

    for (std::size_t index = 0; index < roots.size(); ++index)
    {
        const EdgeRoot& start = roots[index];
        const EdgeRoot& end = roots[(index + 1)%roots.size()];
        const double sweep = positiveAngleDifference(start.angle, end.angle);

        if (sweep <= angleTolerance || sweep >= MyMath::TwoPi - angleTolerance) continue;

        const double middleAngle = normalizeAngle(start.angle + sweep*0.5);
        const double firstValue = body.characteristicValue(firstProfile, middleAngle);
        const double secondValue = body.characteristicValue(secondProfile, middleAngle);

        if (firstValue*secondValue > 0.0) continue;

        EdgeArc arc;
        arc.key = static_cast<int>(start.tag)*4 + static_cast<int>(end.tag);
        arc.startAngle = start.angle;
        arc.sweep = sweep;
        arcs.push_back(arc);
    }

    return !arcs.empty();
}

// 一个固定sharp顶点、固定根边界组合对应一张候选圆边包络分支。
struct EnvelopeEdgeBranchGrid
{
    EnvelopeEdgeBranchGrid()
        : profileVertexIndex(0), firstSegmentIndex(0), secondSegmentIndex(0), branchKey(-1),
          uCount(0), vCount(0), validRowCount(0)
    {
    }

    std::size_t index(std::size_t u, std::size_t v) const { return v*uCount + u; }
    const MyMath::Vector3& point(std::size_t u, std::size_t v) const { return points[index(u, v)]; }
    MyMath::Vector3& point(std::size_t u, std::size_t v) { return points[index(u, v)]; }

    std::size_t profileVertexIndex;
    std::size_t firstSegmentIndex;
    std::size_t secondSegmentIndex;
    int branchKey;
    std::size_t uCount;
    std::size_t vCount;
    std::vector<MyMath::Vector3> points;
    std::vector<unsigned char> validRows;
    std::size_t validRowCount;
};

// 为所有非退化sharp circular edge建立候选包络分支。
// 每个pose先求法向锥有效圆弧，再按圆弧两端根标签组合映射到稳定branchKey。
bool collectEnvelopeEdgeBranchGrids(RevolveBody body,
                                    const std::vector<MyMath::CoordinateSystem>& poses,
                                    int arcSubdivisionCount,
                                    std::vector<EnvelopeProfileVertex>& profileVertices,
                                    std::vector<EnvelopeEdgeBranchGrid>& grids)
{
    profileVertices.clear();
    grids.clear();

    if (poses.size() < 2 || arcSubdivisionCount <= 0) return false;
    if (!buildEnvelopeProfileVertices(body.geometry(), profileVertices)) return true;

    const std::size_t branchesPerVertex = 16;
    grids.resize(profileVertices.size()*branchesPerVertex);

    for (std::size_t vertexIndex = 0; vertexIndex < profileVertices.size(); ++vertexIndex)
    {
        const EnvelopeProfileVertex& vertex = profileVertices[vertexIndex];

        for (int key = 0; key < static_cast<int>(branchesPerVertex); ++key)
        {
            EnvelopeEdgeBranchGrid& grid = grids[vertexIndex*branchesPerVertex + static_cast<std::size_t>(key)];
            grid.profileVertexIndex = vertexIndex;
            grid.firstSegmentIndex = vertex.firstSegmentIndex;
            grid.secondSegmentIndex = vertex.secondSegmentIndex;
            grid.branchKey = key;
            grid.uCount = static_cast<std::size_t>(arcSubdivisionCount + 1);
            grid.vCount = poses.size();
            grid.points.resize(grid.uCount*grid.vCount);
            grid.validRows.assign(grid.vCount, 0);
        }
    }

    for (std::size_t poseIndex = 0; poseIndex < poses.size(); ++poseIndex)
    {
        MyMath::Vector3 translationDerivativeWorld;
        MyMath::Vector3 rotationDerivativeLocal;

        if (!motionDerivativeAtPose(poses, poseIndex, translationDerivativeWorld, rotationDerivativeLocal)) return false;
        body.setMotionState(poses[poseIndex], translationDerivativeWorld, rotationDerivativeLocal);

        for (std::size_t vertexIndex = 0; vertexIndex < profileVertices.size(); ++vertexIndex)
        {
            const EnvelopeProfileVertex& vertex = profileVertices[vertexIndex];
            std::vector<EdgeArc> arcs;

            if (!solveEdgeArcs(body, vertex, arcs)) continue;

            for (std::size_t arcIndex = 0; arcIndex < arcs.size(); ++arcIndex)
            {
                const EdgeArc& arc = arcs[arcIndex];
                if (arc.key < 0 || arc.key >= static_cast<int>(branchesPerVertex)) continue;

                EnvelopeEdgeBranchGrid& grid =
                    grids[vertexIndex*branchesPerVertex + static_cast<std::size_t>(arc.key)];

                if (grid.validRows[poseIndex]) continue;

                for (std::size_t u = 0; u < grid.uCount; ++u)
                {
                    const double lambda =
                        static_cast<double>(u)/static_cast<double>(grid.uCount - 1);
                    const double angle = arc.startAngle + arc.sweep*lambda;

                    const MyMath::Vector3 localPoint(
                        vertex.radius*std::cos(angle),
                        vertex.radius*std::sin(angle),
                        vertex.z);

                    grid.point(u, poseIndex) = poses[poseIndex].toGlobal(localPoint);
                }

                grid.validRows[poseIndex] = 1;
                ++grid.validRowCount;
            }
        }
    }

    return true;
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

struct EnvelopeSegmentDisplay
{
    EnvelopeSegmentDisplay() : profileSegmentIndex(0), plusCreated(false), minusCreated(false) {}

    std::size_t profileSegmentIndex;
    EnvelopeDisplay plus;
    EnvelopeDisplay minus;
    bool plusCreated;
    bool minusCreated;
};

struct EnvelopePlanarEndDisplay
{
    EnvelopePlanarEndDisplay() : profileSegmentIndex(0), plusCreated(false), minusCreated(false) {}

    std::size_t profileSegmentIndex;
    EnvelopeDisplay plus;
    EnvelopeDisplay minus;
    bool plusCreated;
    bool minusCreated;
};

struct EnvelopeEdgeDisplay
{
    EnvelopeEdgeDisplay()
        : profileVertexIndex(0), firstSegmentIndex(0), secondSegmentIndex(0), branchKey(-1), created(false)
    {
    }

    std::size_t profileVertexIndex;
    std::size_t firstSegmentIndex;
    std::size_t secondSegmentIndex;
    int branchKey;
    EnvelopeDisplay display;
    bool created;
};

// 只在一个ProfileSegment自己的参数域内建三角网格。
bool buildEnvelopeMesh(const EnvelopeSegmentGrid& grid, bool plusBranch, EnvelopeRenderMesh& mesh)
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

            mesh.bounds.expandToInclude(
                QVector3D(static_cast<float>(point.x()),
                          static_cast<float>(point.y()),
                          static_cast<float>(point.z())));
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

            // 当前单元四角都必须属于该候选分支有效域，不跨无解区域强行补三角形。
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

// 水平端面自适应网格已经把u=0放在H=0公共Merge Curve上，直接按规则网格三角化即可。
bool buildPlanarEndEnvelopeMesh(const EnvelopePlanarEndGrid& grid,
                                bool plusBranch,
                                EnvelopeRenderMesh& mesh)
{
    mesh = EnvelopeRenderMesh();

    if (grid.uCount < 2 || grid.vCount < 2 ||
        grid.samples.size() != grid.uCount*grid.vCount ||
        grid.validRows.size() != grid.vCount)
    {
        return false;
    }

    const GLuint invalidVertex = static_cast<GLuint>(-1);
    std::vector<GLuint> vertexIndices(grid.samples.size(), invalidVertex);

    for (std::size_t v = 0; v < grid.vCount; ++v)
    {
        if (!grid.validRows[v]) continue;

        for (std::size_t u = 0; u < grid.uCount; ++u)
        {
            const EnvelopeSample& sample = grid.sample(u, v);
            if (!sample.valid) continue;

            const MyMath::Vector3& point =
                plusBranch ? sample.plusPoint : sample.minusPoint;

            const GLuint vertexIndex =
                static_cast<GLuint>(mesh.vertexCount());

            vertexIndices[grid.index(u, v)] = vertexIndex;

            mesh.vertices.push_back(static_cast<GLfloat>(point.x()));
            mesh.vertices.push_back(static_cast<GLfloat>(point.y()));
            mesh.vertices.push_back(static_cast<GLfloat>(point.z()));

            mesh.bounds.expandToInclude(
                QVector3D(static_cast<float>(point.x()),
                          static_cast<float>(point.y()),
                          static_cast<float>(point.z())));
        }
    }

    for (std::size_t v = 0; v + 1 < grid.vCount; ++v)
    {
        if (!grid.validRows[v] || !grid.validRows[v + 1]) continue;

        for (std::size_t u = 0; u + 1 < grid.uCount; ++u)
        {
            const GLuint p00 = vertexIndices[grid.index(u, v)];
            const GLuint p10 = vertexIndices[grid.index(u + 1, v)];
            const GLuint p01 = vertexIndices[grid.index(u, v + 1)];
            const GLuint p11 = vertexIndices[grid.index(u + 1, v + 1)];

            if (p00 == invalidVertex || p10 == invalidVertex ||
                p01 == invalidVertex || p11 == invalidVertex)
            {
                continue;
            }

            mesh.indices.push_back(p00);
            mesh.indices.push_back(p10);
            mesh.indices.push_back(p11);

            mesh.indices.push_back(p00);
            mesh.indices.push_back(p11);
            mesh.indices.push_back(p01);
        }
    }

    return !mesh.vertices.empty() &&
           !mesh.indices.empty() &&
           mesh.bounds.isValid();
}

// 将一个sharp circular edge候选分支沿圆弧参数和刀路参数直接三角化。
bool buildEdgeEnvelopeMesh(const EnvelopeEdgeBranchGrid& grid, EnvelopeRenderMesh& mesh)
{
    mesh = EnvelopeRenderMesh();
    if (grid.uCount < 2 || grid.vCount < 2 ||
        grid.points.size() != grid.uCount*grid.vCount ||
        grid.validRows.size() != grid.vCount)
    {
        return false;
    }

    const GLuint invalidVertex = static_cast<GLuint>(-1);
    std::vector<GLuint> vertexIndices(grid.points.size(), invalidVertex);

    for (std::size_t v = 0; v < grid.vCount; ++v)
    {
        if (!grid.validRows[v]) continue;

        for (std::size_t u = 0; u < grid.uCount; ++u)
        {
            const MyMath::Vector3& point = grid.point(u, v);
            const GLuint vertexIndex = static_cast<GLuint>(mesh.vertexCount());
            vertexIndices[grid.index(u, v)] = vertexIndex;

            mesh.vertices.push_back(static_cast<GLfloat>(point.x()));
            mesh.vertices.push_back(static_cast<GLfloat>(point.y()));
            mesh.vertices.push_back(static_cast<GLfloat>(point.z()));

            mesh.bounds.expandToInclude(
                QVector3D(static_cast<float>(point.x()),
                          static_cast<float>(point.y()),
                          static_cast<float>(point.z())));
        }
    }

    for (std::size_t v = 0; v + 1 < grid.vCount; ++v)
    {
        if (!grid.validRows[v] || !grid.validRows[v + 1]) continue;

        for (std::size_t u = 0; u + 1 < grid.uCount; ++u)
        {
            const GLuint p00 = vertexIndices[grid.index(u, v)];
            const GLuint p10 = vertexIndices[grid.index(u + 1, v)];
            const GLuint p01 = vertexIndices[grid.index(u, v + 1)];
            const GLuint p11 = vertexIndices[grid.index(u + 1, v + 1)];

            if (p00 == invalidVertex || p10 == invalidVertex ||
                p01 == invalidVertex || p11 == invalidVertex) continue;

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

bool createEnvelopeDisplay(MyBRep::Display::BRepViewerWidget& window,
                           const QString& name,
                           const QVector4D& color,
                           EnvelopeDisplay& display)
{
    if (display.mesh.vertices.empty() || display.mesh.indices.empty() || !display.mesh.bounds.isValid()) return false;

    BufferGeometry* geometry = new BufferGeometry(name + "_Geometry", BufferUsage::Static, RenderType::Triangles);

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

    item->addPart(part);
    part->setGeometry(geometry);
    part->setMaterial(material);
    part->setLocalBounds(display.mesh.bounds);

    display.geometry = geometry;
    display.part = part;
    window.update();
    return true;
}

// 将一条Segment在当前pose中的连续有效采样区间分别显示成特征线，不跨无解位置连接。
void appendCharacteristicRuns(const EnvelopeSegmentGrid& grid,
                              std::size_t poseIndex,
                              bool plusBranch,
                              std::vector<std::vector<MyMath::Vector3> >& runs)
{
    runs.clear();
    if (poseIndex >= grid.vCount) return;

    std::vector<MyMath::Vector3> current;

    for (std::size_t u = 0; u < grid.uCount; ++u)
    {
        const EnvelopeSample& sample = grid.sample(u, poseIndex);

        if (!sample.valid)
        {
            if (current.size() >= 2) runs.push_back(current);
            current.clear();
            continue;
        }

        current.push_back(plusBranch ? sample.plusPoint : sample.minusPoint);
    }

    if (current.size() >= 2) runs.push_back(current);
}

void appendPlanarEndCharacteristicRun(const EnvelopePlanarEndGrid& grid,
                                      std::size_t poseIndex,
                                      bool plusBranch,
                                      std::vector<MyMath::Vector3>& points)
{
    points.clear();
    if (poseIndex >= grid.vCount || !grid.validRows[poseIndex]) return;

    points.reserve(grid.uCount);

    for (std::size_t u = 0; u < grid.uCount; ++u)
    {
        const EnvelopeSample& sample = grid.sample(u, poseIndex);
        if (!sample.valid)
        {
            points.clear();
            return;
        }

        points.push_back(plusBranch ? sample.plusPoint : sample.minusPoint);
    }
}


}

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    MyBRep::Display::BRepViewerWidget window;
    window.resize(1000, 700);

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Revolved> wheelGeometry =
        createGrindingWheelGeometry();

    const std::vector<MyMath::CoordinateSystem> poses = createTestPoses();
    if (!wheelGeometry || poses.size() < 2) return -1;

    // 普通非水平ProfileSegment使用固定参数Plus/Minus网格。
    // MinZ/MaxZ水平端面使用H=0 Branch Merge自适应网格；sharp母线顶点再由CircularEdge包络补齐。
    RevolveBody envelopeBody(*wheelGeometry);

    std::vector<EnvelopeSegmentGrid> envelopeGrids;
    if (!collectEnvelopeSegmentGrids(
            envelopeBody, poses, ProfileSamplesPerSegment, envelopeGrids))
    {
        qDebug() << "Envelope segment grid construction failed.";
        return -1;
    }

    std::vector<EnvelopePlanarEndGrid> planarEndGrids;
    if (!collectEnvelopePlanarEndGrids(
            envelopeBody, poses, ProfileSamplesPerSegment, planarEndGrids))
    {
        qDebug() << "Planar end envelope grid construction failed.";
        return -1;
    }

    std::vector<EnvelopeSegmentDisplay> envelopeDisplays;
    envelopeDisplays.reserve(envelopeGrids.size());

    std::size_t totalPlusTriangles = 0;
    std::size_t totalMinusTriangles = 0;

    for (std::size_t gridIndex = 0; gridIndex < envelopeGrids.size(); ++gridIndex)
    {
        const EnvelopeSegmentGrid& grid = envelopeGrids[gridIndex];

        EnvelopeSegmentDisplay display;
        display.profileSegmentIndex = grid.profileSegmentIndex;

        const bool plusBuilt = buildEnvelopeMesh(grid, true, display.plus.mesh);
        const bool minusBuilt = buildEnvelopeMesh(grid, false, display.minus.mesh);

        const QString plusName = QString("PlusEnvelope_S%1").arg(static_cast<qulonglong>(grid.profileSegmentIndex));
        const QString minusName = QString("MinusEnvelope_S%1").arg(static_cast<qulonglong>(grid.profileSegmentIndex));

        if (plusBuilt)
        {
            display.plusCreated = createEnvelopeDisplay(
                window, plusName, QVector4D(1.0f, 0.30f, 0.22f, 0.55f), display.plus);

            if (!display.plusCreated)
            {
                qDebug() << "Plus envelope MyOpenGL display creation failed:"
                         << "profileSegment =" << grid.profileSegmentIndex;
                return -1;
            }

            totalPlusTriangles += display.plus.mesh.triangleCount();
        }

        if (minusBuilt)
        {
            display.minusCreated = createEnvelopeDisplay(
                window, minusName, QVector4D(0.18f, 0.36f, 1.0f, 0.55f), display.minus);

            if (!display.minusCreated)
            {
                qDebug() << "Minus envelope MyOpenGL display creation failed:"
                         << "profileSegment =" << grid.profileSegmentIndex;
                return -1;
            }

            totalMinusTriangles += display.minus.mesh.triangleCount();
        }

        qDebug() << "Envelope segment:"
                 << "profileSegment =" << grid.profileSegmentIndex
                 << "uCount =" << grid.uCount
                 << "vCount =" << grid.vCount
                 << "invalid samples =" << grid.invalidSampleCount
                 << "max condition error =" << grid.maxConditionError
                 << "Plus triangles =" << display.plus.mesh.triangleCount()
                 << "Minus triangles =" << display.minus.mesh.triangleCount();

        envelopeDisplays.push_back(display);
    }

    std::vector<EnvelopePlanarEndDisplay> planarEndDisplays;
    planarEndDisplays.reserve(planarEndGrids.size());

    std::size_t totalPlanarPlusTriangles = 0;
    std::size_t totalPlanarMinusTriangles = 0;

    for (std::size_t gridIndex = 0; gridIndex < planarEndGrids.size(); ++gridIndex)
    {
        const EnvelopePlanarEndGrid& grid = planarEndGrids[gridIndex];

        EnvelopePlanarEndDisplay display;
        display.profileSegmentIndex = grid.profileSegmentIndex;

        const bool plusBuilt =
            buildPlanarEndEnvelopeMesh(grid, true, display.plus.mesh);

        const bool minusBuilt =
            buildPlanarEndEnvelopeMesh(grid, false, display.minus.mesh);

        const QString plusName =
            QString("PlanarEndPlusEnvelope_S%1")
                .arg(static_cast<qulonglong>(grid.profileSegmentIndex));

        const QString minusName =
            QString("PlanarEndMinusEnvelope_S%1")
                .arg(static_cast<qulonglong>(grid.profileSegmentIndex));

        if (plusBuilt)
        {
            display.plusCreated = createEnvelopeDisplay(
                window,
                plusName,
                QVector4D(1.0f, 0.30f, 0.22f, 0.55f),
                display.plus);

            if (!display.plusCreated)
            {
                qDebug() << "Planar end Plus display creation failed:"
                         << "profileSegment =" << grid.profileSegmentIndex;
                return -1;
            }

            totalPlanarPlusTriangles += display.plus.mesh.triangleCount();
        }

        if (minusBuilt)
        {
            display.minusCreated = createEnvelopeDisplay(
                window,
                minusName,
                QVector4D(0.18f, 0.36f, 1.0f, 0.55f),
                display.minus);

            if (!display.minusCreated)
            {
                qDebug() << "Planar end Minus display creation failed:"
                         << "profileSegment =" << grid.profileSegmentIndex;
                return -1;
            }

            totalPlanarMinusTriangles += display.minus.mesh.triangleCount();
        }

        qDebug() << "Planar end envelope:"
                 << "profileSegment =" << grid.profileSegmentIndex
                 << "uCount =" << grid.uCount
                 << "vCount =" << grid.vCount
                 << "valid rows =" << grid.validRowCount
                 << "invalid rows =" << grid.invalidRowCount
                 << "max condition error =" << grid.maxConditionError
                 << "Plus triangles =" << display.plus.mesh.triangleCount()
                 << "Minus triangles =" << display.minus.mesh.triangleCount();

        planarEndDisplays.push_back(display);
    }

    // sharp母线顶点绕Z轴形成圆边；其法向不是唯一值，而是相邻两面外法向构成的法向锥。
    // 使用g1(theta)*g2(theta)<=0求法向锥中存在n·W=0的圆弧，再沿poses形成真正的圆边包络面。
    std::vector<EnvelopeProfileVertex> envelopeProfileVertices;
    std::vector<EnvelopeEdgeBranchGrid> edgeBranchGrids;

    if (!collectEnvelopeEdgeBranchGrids(
            envelopeBody, poses, EdgeArcSubdivisionCount, envelopeProfileVertices, edgeBranchGrids))
    {
        qDebug() << "Circular edge envelope grid construction failed.";
        return -1;
    }

    std::vector<EnvelopeEdgeDisplay> edgeDisplays;
    std::size_t totalEdgeTriangles = 0;

    for (std::size_t gridIndex = 0; gridIndex < edgeBranchGrids.size(); ++gridIndex)
    {
        const EnvelopeEdgeBranchGrid& grid = edgeBranchGrids[gridIndex];

        EnvelopeEdgeDisplay edgeDisplay;
        edgeDisplay.profileVertexIndex = grid.profileVertexIndex;
        edgeDisplay.firstSegmentIndex = grid.firstSegmentIndex;
        edgeDisplay.secondSegmentIndex = grid.secondSegmentIndex;
        edgeDisplay.branchKey = grid.branchKey;

        if (!buildEdgeEnvelopeMesh(grid, edgeDisplay.display.mesh)) continue;

        const QString name =
            QString("CircularEdgeEnvelope_V%1_B%2")
                .arg(static_cast<qulonglong>(grid.profileVertexIndex))
                .arg(grid.branchKey);

        edgeDisplay.created = createEnvelopeDisplay(
            window, name, QVector4D(1.0f, 0.82f, 0.05f, 0.72f), edgeDisplay.display);

        if (!edgeDisplay.created)
        {
            qDebug() << "Circular edge envelope MyOpenGL display creation failed:"
                     << "profileVertex =" << grid.profileVertexIndex
                     << "firstSegment =" << grid.firstSegmentIndex
                     << "secondSegment =" << grid.secondSegmentIndex
                     << "branchKey =" << grid.branchKey;
            return -1;
        }

        totalEdgeTriangles += edgeDisplay.display.mesh.triangleCount();

        qDebug() << "Circular edge envelope:"
                 << "profileVertex =" << grid.profileVertexIndex
                 << "firstSegment =" << grid.firstSegmentIndex
                 << "secondSegment =" << grid.secondSegmentIndex
                 << "branchKey =" << grid.branchKey
                 << "valid rows =" << grid.validRowCount
                 << "triangles =" << edgeDisplay.display.mesh.triangleCount();

        edgeDisplays.push_back(edgeDisplay);
    }

    qDebug() << "Envelope total:"
             << "source profile segments =" << wheelGeometry->profileSegmentCount()
             << "smooth segment grids =" << envelopeGrids.size()
             << "planar end grids =" << planarEndGrids.size()
             << "sharp profile vertices =" << envelopeProfileVertices.size()
             << "edge envelope branches =" << edgeDisplays.size()
             << "poses =" << poses.size()
             << "Smooth Plus triangles =" << totalPlusTriangles
             << "Smooth Minus triangles =" << totalMinusTriangles
             << "Planar Plus triangles =" << totalPlanarPlusTriangles
             << "Planar Minus triangles =" << totalPlanarMinusTriangles
             << "Edge triangles =" << totalEdgeTriangles;

    // 砂轮本体仍使用MyBRep Solid显示；显示网格同样从Geometry_Revolved母线读取，不再维护RevolvedSection。
    RevolveBody wheelBody(*wheelGeometry);
    wheelBody.moveTo(poses[0]);

    MyBRep::Solid grindingWheel = createRevolvedDisplaySolid(*wheelGeometry, 64);
    grindingWheel.setLocalToWorld(wheelBody.coordinateSystem().toMatrix());
    window.addSolid(grindingWheel);

    MyBRep::Display::BRepDisplayStyle plusStyle;
    plusStyle.wireColor = QVector4D(1.0f, 0.0f, 0.0f, 1.0f);
    plusStyle.wireWidth = 3.0f;

    MyBRep::Display::BRepDisplayStyle minusStyle;
    minusStyle.wireColor = QVector4D(0.0f, 0.2f, 1.0f, 1.0f);
    minusStyle.wireWidth = 3.0f;

    std::vector<MyBRep::Display::BRepDisplayId> characteristicDisplayIds;
    std::size_t poseIndex = 0;

    auto clearCharacteristicDisplays = [&]()
    {
        for (std::size_t index = 0; index < characteristicDisplayIds.size(); ++index)
        {
            if (characteristicDisplayIds[index] == MyBRep::Display::InvalidBRepDisplayId) continue;
            window.removeDisplay(characteristicDisplayIds[index]);
        }

        characteristicDisplayIds.clear();
    };

    auto updateCharacteristicDisplay = [&]()
    {
        clearCharacteristicDisplays();

        for (std::size_t gridIndex = 0; gridIndex < envelopeGrids.size(); ++gridIndex)
        {
            const EnvelopeSegmentGrid& grid = envelopeGrids[gridIndex];

            std::vector<std::vector<MyMath::Vector3> > plusRuns;
            std::vector<std::vector<MyMath::Vector3> > minusRuns;

            appendCharacteristicRuns(grid, poseIndex, true, plusRuns);
            appendCharacteristicRuns(grid, poseIndex, false, minusRuns);

            for (std::size_t runIndex = 0; runIndex < plusRuns.size(); ++runIndex)
            {
                const MyBRep::Edge edge = MyBRep::Modeling::makeBSpline(plusRuns[runIndex]);
                const QString name =
                    QString("CharacteristicPlus_S%1_R%2")
                        .arg(static_cast<qulonglong>(grid.profileSegmentIndex))
                        .arg(static_cast<qulonglong>(runIndex));

                characteristicDisplayIds.push_back(window.addWireframe(edge, name, plusStyle));
            }

            for (std::size_t runIndex = 0; runIndex < minusRuns.size(); ++runIndex)
            {
                const MyBRep::Edge edge = MyBRep::Modeling::makeBSpline(minusRuns[runIndex]);
                const QString name =
                    QString("CharacteristicMinus_S%1_R%2")
                        .arg(static_cast<qulonglong>(grid.profileSegmentIndex))
                        .arg(static_cast<qulonglong>(runIndex));

                characteristicDisplayIds.push_back(window.addWireframe(edge, name, minusStyle));
            }
        }

        for (std::size_t gridIndex = 0; gridIndex < planarEndGrids.size(); ++gridIndex)
        {
            const EnvelopePlanarEndGrid& grid = planarEndGrids[gridIndex];

            std::vector<MyMath::Vector3> plusPoints;
            std::vector<MyMath::Vector3> minusPoints;

            appendPlanarEndCharacteristicRun(
                grid, poseIndex, true, plusPoints);

            appendPlanarEndCharacteristicRun(
                grid, poseIndex, false, minusPoints);

            if (plusPoints.size() >= 2)
            {
                const MyBRep::Edge edge =
                    MyBRep::Modeling::makeBSpline(plusPoints);

                const QString name =
                    QString("PlanarEndCharacteristicPlus_S%1")
                        .arg(static_cast<qulonglong>(grid.profileSegmentIndex));

                characteristicDisplayIds.push_back(
                    window.addWireframe(edge, name, plusStyle));
            }

            if (minusPoints.size() >= 2)
            {
                const MyBRep::Edge edge =
                    MyBRep::Modeling::makeBSpline(minusPoints);

                const QString name =
                    QString("PlanarEndCharacteristicMinus_S%1")
                        .arg(static_cast<qulonglong>(grid.profileSegmentIndex));

                characteristicDisplayIds.push_back(
                    window.addWireframe(edge, name, minusStyle));
            }
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
