#include <cmath>
#include <cstddef>
#include <iostream>
#include <string>
#include <vector>

#include "MyMath/Vector2.h"
#include "MyMath/Vector3.h"
#include "MyBRep/Foundation/RefPtr.h"
#include "MyBRep/Geometry/Curve/Geometry_Curve.h"
#include "MyBRep/Geometry/Curve/Geometry_Line.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Curve2D.h"
#include "MyBRep/Geometry/Curve2D/Geometry_Line2D.h"
#include "MyBRep/Modeling/Edge/EdgeModeling.h"
#include "MyBRep/Modeling/Face/FaceModeling.h"
#include "MyBRep/Modeling/Wire/WireModeling.h"
#include "MyBRep/Operation/Split/EdgeSplit.h"
#include "MyBRep/Operation/Split/FaceSplit.h"
#include "MyBRep/Operation/Split/WireSplit.h"
#include "MyBRep/Topology/Topology_Builder.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Face/Topology_Face.h"
#include "MyBRep/Topology/Vertex/Topology_Vertex.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

namespace
{

const double TestTolerance = 1.0e-10; // 专项测试统一使用的几何与参数比较容差。

class TestContext
{
public:
    TestContext()
        : m_passed(0), m_failed(0)
    {
    }

    void expect(bool condition, const std::string& name)
    {
        if (condition)
        {
            ++m_passed;
            std::cout << "[PASS] " << name << std::endl;
        }
        else
        {
            ++m_failed;
            std::cout << "[FAIL] " << name << std::endl;
        }
    }

    int passed() const
    {
        return m_passed;
    }

    int failed() const
    {
        return m_failed;
    }

private:
    int m_passed;
    int m_failed;
};

// 判断两个标量是否在专项测试容差内相等。
bool nearEqual(double first, double second)
{
    return std::fabs(first - second) <= TestTolerance;
}

// 判断三维点是否在专项测试容差内相等。
bool pointEqual(const MyMath::Vector3& first, const MyMath::Vector3& second)
{
    return first.isEqualTo(second, TestTolerance);
}

// 判断二维点是否在专项测试容差内相等。
bool pointEqual(const MyMath::Vector2& first, const MyMath::Vector2& second)
{
    return std::fabs(first.x() - second.x()) <= TestTolerance && std::fabs(first.y() - second.y()) <= TestTolerance;
}

// 判断Wire全部相邻Edge-use是否使用同一Topology_Vertex身份连接。
bool hasExactVertexConnectivity(const MyBRep::Topology_Wire& wire)
{
    if (!wire.isValid() || wire.edgeCount() == 0)
    {
        return false;
    }

    for (std::size_t index = 0; index + 1 < wire.edgeCount(); ++index)
    {
        if (!wire.edge(index).endVertex().isSame(wire.edge(index + 1).startVertex()))
        {
            return false;
        }
    }

    return !wire.isClosed() || wire.edge(wire.edgeCount() - 1).endVertex().isSame(wire.edge(0).startVertex());
}

// 返回指定Topology_Vertex身份作为Wire Edge-use起点出现的次数。
std::size_t startVertexOccurrenceCount(const MyBRep::Topology_Wire& wire, const MyBRep::Topology_Vertex& vertex)
{
    std::size_t count = 0;

    for (std::size_t index = 0; index < wire.edgeCount(); ++index)
    {
        if (wire.edge(index).startVertex().isSame(vertex))
        {
            ++count;
        }
    }

    return count;
}

// 判断Wire是否包含指定Topology_TEdge身份。
bool containsEdgeIdentity(const MyBRep::Topology_Wire& wire, const MyBRep::Topology_Edge& edge)
{
    for (std::size_t index = 0; index < wire.edgeCount(); ++index)
    {
        if (wire.edge(index).isSame(edge))
        {
            return true;
        }
    }

    return false;
}

// 查找两个Wire之间唯一共享的Topology_TEdge，并返回两侧Edge-use。
bool findUniqueSharedEdge(const MyBRep::Topology_Wire& firstWire, const MyBRep::Topology_Wire& secondWire,
                          MyBRep::Topology_Edge& firstUse, MyBRep::Topology_Edge& secondUse)
{
    std::size_t count = 0;

    for (std::size_t firstIndex = 0; firstIndex < firstWire.edgeCount(); ++firstIndex)
    {
        for (std::size_t secondIndex = 0; secondIndex < secondWire.edgeCount(); ++secondIndex)
        {
            if (!firstWire.edge(firstIndex).isSame(secondWire.edge(secondIndex)))
            {
                continue;
            }

            firstUse = firstWire.edge(firstIndex);
            secondUse = secondWire.edge(secondIndex);
            ++count;
        }
    }

    return count == 1;
}

// 使用两个既有Topology_Vertex身份创建共享端点的有限直线Edge。
MyBRep::Topology_Edge createSharedLineEdge(const MyBRep::Topology_Vertex& startVertex, const MyBRep::Topology_Vertex& endVertex)
{
    const MyMath::Vector3 direction = endVertex.point() - startVertex.point();
    const double length = direction.length();
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> geometry(new MyBRep::Geometry_Line(startVertex.point(), direction));
    return MyBRep::Topology_Edge(startVertex, endVertex, geometry, 0.0, length, TestTolerance);
}

// 为世界XY平面内的直线Edge建立指定Face Surface上的P-Curve。
void attachWorldXYLinePCurve(MyBRep::Topology_Edge& edge, const MyBRep::Topology_Face& face)
{
    const MyMath::Vector3 start = edge.startVertex().point();
    const MyMath::Vector3 end = edge.endVertex().point();
    const MyMath::Vector2 startUV(start.x(), start.y());
    const MyMath::Vector2 directionUV(end.x() - start.x(), end.y() - start.y());
    const double length = directionUV.length();
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> curve(new MyBRep::Geometry_Line2D(startUV, directionUV));

    MyBRep::Topology_Builder::addCurveOnSurface(edge, face.geometryResource(), curve, 0.0, length, TestTolerance);
}

// 测试Forward Edge单点与多点切分。
void testEdgeSplit(TestContext& context)
{
    const MyBRep::Topology_Edge edge = MyBRep::Modeling::createLine(MyMath::Vector3(0.0, 0.0, 0.0), MyMath::Vector3(10.0, 0.0, 0.0));
    const std::vector<MyBRep::Topology_Edge> single = MyBRep::Operation::Split::splitEdge(edge, 0.25);

    context.expect(single.size() == 2, "EdgeSplit single count");
    context.expect(single[0].geometryResource().get() == edge.geometryResource().get() && single[1].geometryResource().get() == edge.geometryResource().get(),
                   "EdgeSplit single shares Geometry_Curve");
    context.expect(single[0].startVertex().isSame(edge.startVertex()) && single[1].endVertex().isSame(edge.endVertex()),
                   "EdgeSplit single preserves source endpoint identities");
    context.expect(single[0].endVertex().isSame(single[1].startVertex()), "EdgeSplit single shares split Vertex identity");
    context.expect(pointEqual(single[0].endVertex().point(), MyMath::Vector3(2.5, 0.0, 0.0)), "EdgeSplit single split point");
    context.expect(nearEqual(single[0].firstParameter(), 0.0) && nearEqual(single[0].lastParameter(), 2.5) &&
                   nearEqual(single[1].firstParameter(), 2.5) && nearEqual(single[1].lastParameter(), 10.0),
                   "EdgeSplit single curve parameter ranges");

    std::vector<double> parameters;
    parameters.push_back(0.2);
    parameters.push_back(0.6);
    parameters.push_back(0.8);

    const std::vector<MyBRep::Topology_Edge> multiple = MyBRep::Operation::Split::splitEdge(edge, parameters);
    context.expect(multiple.size() == 4, "EdgeSplit multiple count");

    bool connected = multiple.front().startVertex().isSame(edge.startVertex()) && multiple.back().endVertex().isSame(edge.endVertex());
    bool sharedGeometry = true;

    for (std::size_t index = 0; index < multiple.size(); ++index)
    {
        sharedGeometry = sharedGeometry && multiple[index].geometryResource().get() == edge.geometryResource().get();

        if (index + 1 < multiple.size())
        {
            connected = connected && multiple[index].endVertex().isSame(multiple[index + 1].startVertex());
        }
    }

    context.expect(connected, "EdgeSplit multiple exact Vertex connectivity");
    context.expect(sharedGeometry, "EdgeSplit multiple shares Geometry_Curve");
    context.expect(pointEqual(multiple[0].endVertex().point(), MyMath::Vector3(2.0, 0.0, 0.0)) &&
                   pointEqual(multiple[1].endVertex().point(), MyMath::Vector3(6.0, 0.0, 0.0)) &&
                   pointEqual(multiple[2].endVertex().point(), MyMath::Vector3(8.0, 0.0, 0.0)),
                   "EdgeSplit multiple split points");
}

// 测试Reversed Edge的输入方向、结果顺序和参数映射。
void testReversedEdgeSplit(TestContext& context)
{
    const MyBRep::Topology_Edge source = MyBRep::Modeling::createLine(MyMath::Vector3(0.0, 0.0, 0.0), MyMath::Vector3(10.0, 0.0, 0.0));
    const MyBRep::Topology_Edge edge = source.reversed();
    const std::vector<MyBRep::Topology_Edge> result = MyBRep::Operation::Split::splitEdge(edge, 0.25);

    context.expect(result.size() == 2, "Reversed EdgeSplit count");
    context.expect(result[0].isReversed() && result[1].isReversed(), "Reversed EdgeSplit preserves child use orientation");
    context.expect(result[0].startVertex().isSame(edge.startVertex()) && result[1].endVertex().isSame(edge.endVertex()),
                   "Reversed EdgeSplit preserves traversal endpoints");
    context.expect(result[0].endVertex().isSame(result[1].startVertex()), "Reversed EdgeSplit shares split Vertex identity");
    context.expect(pointEqual(result[0].endVertex().point(), edge.pointAt(0.25)), "Reversed EdgeSplit normalized split point");
    context.expect(result[0].geometryResource().get() == source.geometryResource().get() && result[1].geometryResource().get() == source.geometryResource().get(),
                   "Reversed EdgeSplit shares Geometry_Curve");
}

// 测试完整圆Edge切分后仍可由共享首尾Vertex重新组成闭合Wire。
void testCircleEdgeSplit(TestContext& context)
{
    const MyBRep::Topology_Wire circle = MyBRep::Modeling::createCircle(5.0);
    const MyBRep::Topology_Edge edge = circle.edge(0);

    std::vector<double> parameters;
    parameters.push_back(0.25);
    parameters.push_back(0.5);
    parameters.push_back(0.75);

    const std::vector<MyBRep::Topology_Edge> edges = MyBRep::Operation::Split::splitEdge(edge, parameters);
    const MyBRep::Topology_Wire rebuilt(edges);

    context.expect(edges.size() == 4, "Circle EdgeSplit count");
    context.expect(rebuilt.isValid() && rebuilt.isClosed(), "Circle EdgeSplit rebuilds closed Wire");
    context.expect(rebuilt.edge(0).startVertex().isSame(edge.startVertex()) &&
                   rebuilt.edge(rebuilt.edgeCount() - 1).endVertex().isSame(edge.endVertex()),
                   "Circle EdgeSplit preserves shared closure Vertex");
    context.expect(hasExactVertexConnectivity(rebuilt), "Circle EdgeSplit exact Wire connectivity");

    bool sharedGeometry = true;

    for (std::size_t index = 0; index < rebuilt.edgeCount(); ++index)
    {
        sharedGeometry = sharedGeometry && rebuilt.edge(index).geometryResource().get() == edge.geometryResource().get();
    }

    context.expect(sharedGeometry, "Circle EdgeSplit shares Geometry_Circle");
}

// 测试带P-Curve的Face边界Edge切分后二维表示资源和参数区间正确继承。
void testPCurvePropagation(TestContext& context)
{
    const MyBRep::Topology_Wire rectangle = MyBRep::Modeling::createRectangle(10.0, 8.0);
    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(rectangle, TestTolerance);
    const MyBRep::Geometry_Surface& surface = face.geometry();
    const MyBRep::Topology_Edge source = face.wire(0).edge(0);

    context.expect(source.hasCurveOnSurface(surface) && source.curveOnSurfaceCount() == 1, "Planar Face boundary has one P-Curve representation");

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> sourceCurve = source.curveOnSurfaceResource(surface);
    const double splitParameter = 0.3;
    const double expectedSplitCurveParameter = source.curveOnSurfaceParameterAt(surface, splitParameter);
    const MyMath::Vector2 expectedSplitUV = source.surfaceParameterAt(surface, splitParameter);
    const std::vector<MyBRep::Topology_Edge> result = MyBRep::Operation::Split::splitEdge(source, splitParameter);

    context.expect(result[0].hasCurveOnSurface(surface) && result[1].hasCurveOnSurface(surface), "EdgeSplit propagates P-Curve representation");
    context.expect(result[0].curveOnSurfaceResource(surface).get() == sourceCurve.get() && result[1].curveOnSurfaceResource(surface).get() == sourceCurve.get(),
                   "EdgeSplit shares Geometry_Curve2D resource");
    context.expect(nearEqual(result[0].curveOnSurfaceFirstParameter(surface), source.curveOnSurfaceFirstParameter(surface)) &&
                   nearEqual(result[0].curveOnSurfaceLastParameter(surface), expectedSplitCurveParameter) &&
                   nearEqual(result[1].curveOnSurfaceFirstParameter(surface), expectedSplitCurveParameter) &&
                   nearEqual(result[1].curveOnSurfaceLastParameter(surface), source.curveOnSurfaceLastParameter(surface)),
                   "EdgeSplit trims P-Curve parameter ranges");
    context.expect(pointEqual(result[0].surfaceParameterAt(surface, 1.0), expectedSplitUV) &&
                   pointEqual(result[1].surfaceParameterAt(surface, 0.0), expectedSplitUV),
                   "EdgeSplit P-Curve split UV continuity");

    const MyBRep::Topology_Edge reversed = source.reversed();
    const std::vector<MyBRep::Topology_Edge> reversedResult = MyBRep::Operation::Split::splitEdge(reversed, 0.25);
    const double reversedSplitParameter = reversed.curveOnSurfaceParameterAt(surface, 0.25);

    context.expect(reversedResult[0].isReversed() && reversedResult[1].isReversed(), "Reversed P-Curve EdgeSplit orientation");
    context.expect(nearEqual(reversedResult[0].curveOnSurfaceFirstParameter(surface), reversed.curveOnSurfaceFirstParameter(surface)) &&
                   nearEqual(reversedResult[0].curveOnSurfaceLastParameter(surface), reversedSplitParameter) &&
                   nearEqual(reversedResult[1].curveOnSurfaceFirstParameter(surface), reversedSplitParameter) &&
                   nearEqual(reversedResult[1].curveOnSurfaceLastParameter(surface), reversed.curveOnSurfaceLastParameter(surface)),
                   "Reversed EdgeSplit P-Curve parameter direction");
}

// 测试Wire Edge-use替换、边界内部位置细分和Reversed Wire。
void testWireSplit(TestContext& context)
{
    const MyBRep::Topology_Wire wire = MyBRep::Modeling::createRectangle(10.0, 8.0);
    const MyBRep::Topology_Wire splitWire = MyBRep::Operation::Split::splitWireEdge(wire, 1, 0.5);

    context.expect(splitWire.isValid() && splitWire.isClosed(), "WireSplit preserves closed state");
    context.expect(splitWire.edgeCount() == 5, "WireSplit increases Edge-use count");
    context.expect(hasExactVertexConnectivity(splitWire), "WireSplit exact Vertex connectivity");
    context.expect(splitWire.edge(0).isSame(wire.edge(0)) && splitWire.edge(3).isSame(wire.edge(2)) && splitWire.edge(4).isSame(wire.edge(3)),
                   "WireSplit preserves unaffected Edge identities");
    context.expect(splitWire.edge(1).geometryResource().get() == wire.edge(1).geometryResource().get() &&
                   splitWire.edge(2).geometryResource().get() == wire.edge(1).geometryResource().get(),
                   "WireSplit replacement Edges share source Geometry_Curve");

    const MyBRep::Operation::Split::WireBoundarySplit boundary =
        MyBRep::Operation::Split::splitClosedWireBoundary(wire, MyBRep::Operation::Split::WireSplitLocation(0, 0.25),
                                                         MyBRep::Operation::Split::WireSplitLocation(0, 0.75));

    context.expect(boundary.wire.isClosed() && boundary.wire.edgeCount() == 6, "Wire boundary split supports two locations on same Edge-use");
    context.expect(pointEqual(boundary.firstVertex.point(), MyMath::Vector3(-2.5, -4.0, 0.0)) &&
                   pointEqual(boundary.secondVertex.point(), MyMath::Vector3(2.5, -4.0, 0.0)),
                   "Wire boundary split resolves internal Vertex positions");
    context.expect(startVertexOccurrenceCount(boundary.wire, boundary.firstVertex) == 1 &&
                   startVertexOccurrenceCount(boundary.wire, boundary.secondVertex) == 1,
                   "Wire boundary split inserts unique shared Vertex identities");
    context.expect(hasExactVertexConnectivity(boundary.wire), "Wire boundary split exact connectivity");

    const MyBRep::Topology_Wire reversed = wire.reversed();
    const MyBRep::Topology_Wire reversedSplit = MyBRep::Operation::Split::splitWireEdge(reversed, 0, 0.4);
    context.expect(reversedSplit.isClosed() && reversedSplit.edge(0).startVertex().isSame(reversed.edge(0).startVertex()) &&
                   reversedSplit.edge(reversedSplit.edgeCount() - 1).endVertex().isSame(reversed.edge(reversed.edgeCount() - 1).endVertex()),
                   "Reversed WireSplit preserves current traversal direction");
    context.expect(reversedSplit.edgeCount() == 5 && hasExactVertexConnectivity(reversedSplit), "Reversed WireSplit result topology");
}

// 测试使用两个既有边界Vertex的splitting Edge把单Wire Face分成两个Face。
void testFaceSplitAtExistingVertices(TestContext& context)
{
    const MyBRep::Topology_Wire rectangle = MyBRep::Modeling::createRectangle(10.0, 8.0);
    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(rectangle, TestTolerance);
    const MyBRep::Topology_Wire boundary = face.wire(0);
    MyBRep::Topology_Edge splittingEdge = createSharedLineEdge(boundary.edge(0).startVertex(), boundary.edge(1).endVertex());

    attachWorldXYLinePCurve(splittingEdge, face);

    const std::vector<MyBRep::Topology_Face> result = MyBRep::Operation::Split::splitFaceByEdge(face, splittingEdge);
    context.expect(result.size() == 2, "FaceSplit existing Vertex result count");
    context.expect(result[0].geometryResource().get() == face.geometryResource().get() &&
                   result[1].geometryResource().get() == face.geometryResource().get(),
                   "FaceSplit existing Vertex shares Geometry_Surface");
    context.expect(result[0].wireCount() == 1 && result[1].wireCount() == 1 &&
                   result[0].wire(0).isClosed() && result[1].wire(0).isClosed(),
                   "FaceSplit existing Vertex produces two closed trimming Wires");
    context.expect(result[0].wire(0).edgeCount() == 3 && result[1].wire(0).edgeCount() == 3,
                   "FaceSplit existing Vertex produces two triangular regions");
    context.expect(hasExactVertexConnectivity(result[0].wire(0)) && hasExactVertexConnectivity(result[1].wire(0)),
                   "FaceSplit existing Vertex exact Wire connectivity");
    context.expect(containsEdgeIdentity(result[0].wire(0), splittingEdge) && containsEdgeIdentity(result[1].wire(0), splittingEdge),
                   "FaceSplit existing Vertex shares splitting TEdge identity");

    MyBRep::Topology_Edge firstUse;
    MyBRep::Topology_Edge secondUse;
    const bool uniqueShared = findUniqueSharedEdge(result[0].wire(0), result[1].wire(0), firstUse, secondUse);

    context.expect(uniqueShared, "FaceSplit existing Vertex has one shared internal Edge");
    context.expect(uniqueShared && firstUse.isForward() != secondUse.isForward(), "FaceSplit existing Vertex internal Edge uses opposite orientations");
}

// 测试splitting Edge端点落在boundary Edge内部时自动细分边界并重建共享拓扑端点。
void testFaceSplitAtBoundaryLocations(TestContext& context)
{
    const MyBRep::Topology_Wire rectangle = MyBRep::Modeling::createRectangle(10.0, 8.0);
    const MyBRep::Topology_Face face = MyBRep::Modeling::createPlanarFace(rectangle, TestTolerance);
    MyBRep::Topology_Edge splittingEdge = MyBRep::Modeling::createLine(MyMath::Vector3(-5.0, 0.0, 0.0), MyMath::Vector3(5.0, 0.0, 0.0));

    attachWorldXYLinePCurve(splittingEdge, face);

    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve> sourceGeometry = splittingEdge.geometryResource();
    const MyBRep::Foundation::RefPtr<const MyBRep::Geometry_Curve2D> sourcePCurve = splittingEdge.curveOnSurfaceResource(face.geometry());

    const std::vector<MyBRep::Topology_Face> result = MyBRep::Operation::Split::splitFaceByEdge(
        face, MyBRep::Operation::Split::WireSplitLocation(3, 0.5), MyBRep::Operation::Split::WireSplitLocation(1, 0.5), splittingEdge, TestTolerance);

    context.expect(result.size() == 2, "FaceSplit boundary location result count");
    context.expect(result[0].geometryResource().get() == face.geometryResource().get() &&
                   result[1].geometryResource().get() == face.geometryResource().get(),
                   "FaceSplit boundary location shares Geometry_Surface");
    context.expect(result[0].wire(0).edgeCount() == 4 && result[1].wire(0).edgeCount() == 4,
                   "FaceSplit boundary location produces two four-edge regions");
    context.expect(result[0].wire(0).isClosed() && result[1].wire(0).isClosed() &&
                   hasExactVertexConnectivity(result[0].wire(0)) && hasExactVertexConnectivity(result[1].wire(0)),
                   "FaceSplit boundary location produces valid closed Wires");

    MyBRep::Topology_Edge firstUse;
    MyBRep::Topology_Edge secondUse;
    const bool uniqueShared = findUniqueSharedEdge(result[0].wire(0), result[1].wire(0), firstUse, secondUse);

    context.expect(uniqueShared, "FaceSplit boundary location has one shared internal Edge");
    context.expect(uniqueShared && firstUse.isForward() != secondUse.isForward(), "FaceSplit boundary location internal Edge uses opposite orientations");
    context.expect(uniqueShared && !firstUse.isSame(splittingEdge), "FaceSplit boundary location rebuilds splitting TEdge identity");
    context.expect(uniqueShared && firstUse.geometryResource().get() == sourceGeometry.get() &&
                   firstUse.curveOnSurfaceResource(face.geometry()).get() == sourcePCurve.get(),
                   "FaceSplit boundary location shares splitting Geometry and P-Curve resources");
    context.expect(uniqueShared && ((pointEqual(firstUse.startVertex().point(), MyMath::Vector3(-5.0, 0.0, 0.0)) &&
                                    pointEqual(firstUse.endVertex().point(), MyMath::Vector3(5.0, 0.0, 0.0))) ||
                                   (pointEqual(firstUse.startVertex().point(), MyMath::Vector3(5.0, 0.0, 0.0)) &&
                                    pointEqual(firstUse.endVertex().point(), MyMath::Vector3(-5.0, 0.0, 0.0)))),
                   "FaceSplit boundary location internal Edge endpoints");

    const MyBRep::Topology_Face reversedFace = face.reversed();
    const MyBRep::Topology_Wire reversedBoundary = reversedFace.wire(0);
    const MyMath::Vector3 reversedStart = reversedBoundary.edge(0).pointAt(0.5);
    const MyMath::Vector3 reversedEnd = reversedBoundary.edge(2).pointAt(0.5);
    MyBRep::Topology_Edge reversedSplittingEdge = MyBRep::Modeling::createLine(reversedStart, reversedEnd);

    attachWorldXYLinePCurve(reversedSplittingEdge, reversedFace);

    const std::vector<MyBRep::Topology_Face> reversedResult = MyBRep::Operation::Split::splitFaceByEdge(
        reversedFace, MyBRep::Operation::Split::WireSplitLocation(0, 0.5), MyBRep::Operation::Split::WireSplitLocation(2, 0.5),
        reversedSplittingEdge, TestTolerance);

    context.expect(reversedResult.size() == 2 && reversedResult[0].isReversed() && reversedResult[1].isReversed(),
                   "Reversed FaceSplit preserves Face use orientation");
    context.expect(reversedResult[0].geometryResource().get() == face.geometryResource().get() &&
                   reversedResult[1].geometryResource().get() == face.geometryResource().get(),
                   "Reversed FaceSplit shares Geometry_Surface");
    context.expect(reversedResult[0].wire(0).isClosed() && reversedResult[1].wire(0).isClosed(),
                   "Reversed FaceSplit produces closed Wires");
}

}

int main()
{
    TestContext context;

    std::cout << "============================================================" << std::endl;
    std::cout << "MyBRep Split Core v1 Test" << std::endl;
    std::cout << "============================================================" << std::endl;

    testEdgeSplit(context);
    testReversedEdgeSplit(context);
    testCircleEdgeSplit(context);
    testPCurvePropagation(context);
    testWireSplit(context);
    testFaceSplitAtExistingVertices(context);
    testFaceSplitAtBoundaryLocations(context);

    std::cout << "============================================================" << std::endl;
    std::cout << "Passed: " << context.passed() << " | Failed: " << context.failed() << std::endl;
    std::cout << "============================================================" << std::endl;

    return context.failed() == 0 ? 0 : 1;
}