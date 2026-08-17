#include "FaceSplit.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "MyBRep/Foundation/Diagnostic.h"
#include "MyBRep/Topology/Topology_Builder.h"

namespace
{

bool isFiniteNonNegative(double value)
{
    const double infinity = (std::numeric_limits<double>::infinity)();
    return value == value && value != infinity && value != -infinity && value >= 0.0;
}

// 将当前Face使用方向下的Wire序列转换为新Topology_TFace需要的Forward存储方向。
std::vector<MyBRep::Topology_Wire> forwardStoredWires(const MyBRep::Topology_Face& sourceFace, const std::vector<MyBRep::Topology_Wire>& currentWires)
{
    if (sourceFace.isForward())
    {
        return currentWires;
    }

    std::vector<MyBRep::Topology_Wire> result;
    result.reserve(currentWires.size());

    for (std::size_t index = 0; index < currentWires.size(); ++index)
    {
        result.push_back(currentWires[index].reversed());
    }

    return result;
}

// 使用当前Face方向下的完整Wire序列重建共享同一Geometry_Surface的新Face，并恢复源Face使用方向。
MyBRep::Topology_Face rebuildFace(const MyBRep::Topology_Face& sourceFace, const std::vector<MyBRep::Topology_Wire>& currentWires)
{
    const std::vector<MyBRep::Topology_Wire> storedWires = forwardStoredWires(sourceFace, currentWires);
    MyBRep::Topology_Face result(sourceFace.geometryResource(), storedWires);
    return sourceFace.isForward() ? result : result.reversed();
}

MyBRep::Topology_Face rebuildSingleWireFace(const MyBRep::Topology_Face& sourceFace, const MyBRep::Topology_Wire& currentWire)
{
    std::vector<MyBRep::Topology_Wire> wires;
    wires.push_back(currentWire);
    return rebuildFace(sourceFace, wires);
}

// 使用boundary细分产生的共享Vertex替换独立splitting Edge的端点，3D Curve及全部Curve-on-Surface资源继续共享。
MyBRep::Topology_Edge rebuildSplittingEdge(const MyBRep::Topology_Edge& splittingEdge, const MyBRep::Topology_Vertex& startVertex,
                                           const MyBRep::Topology_Vertex& endVertex, double tolerance)
{
    const MyBRep::Topology_Edge forwardSource = splittingEdge.isForward() ? splittingEdge : splittingEdge.reversed();
    const MyBRep::Topology_Vertex forwardStart = splittingEdge.isForward() ? startVertex : endVertex;
    const MyBRep::Topology_Vertex forwardEnd = splittingEdge.isForward() ? endVertex : startVertex;
    const MyMath::Vector3 sourceStartPoint = forwardSource.geometry().pointAt(forwardSource.firstParameter());
    const MyMath::Vector3 sourceEndPoint = forwardSource.geometry().pointAt(forwardSource.lastParameter());
    const double startError = (forwardStart.point() - sourceStartPoint).length();
    const double endError = (forwardEnd.point() - sourceEndPoint).length();

    MYBREP_ASSERT_MESSAGE(startError <= tolerance && endError <= tolerance,
                          "FaceSplit splitting Edge geometry endpoints must match the resolved boundary vertices within tolerance.");

    const double endpointTolerance = (std::max)(tolerance, (std::max)(startError, endError));
    MyBRep::Topology_Edge forwardResult(forwardStart, forwardEnd, forwardSource.geometryResource(),
                                        forwardSource.firstParameter(), forwardSource.lastParameter(), endpointTolerance);
    MyBRep::Topology_Builder::copyCurveOnSurfaceRange(forwardSource, 0.0, 1.0, forwardResult);
    return splittingEdge.isForward() ? forwardResult : forwardResult.reversed();
}

}

namespace MyBRep
{
namespace Operation
{
namespace Split
{

Topology_Face replaceFaceWire(const Topology_Face& face, std::size_t wireIndex, const Topology_Wire& replacementWire)
{
    MYBREP_ASSERT_MESSAGE(face.isValid(), "FaceSplit requires a valid Topology_Face.");
    MYBREP_ASSERT_MESSAGE(wireIndex < face.wireCount(), "FaceSplit Wire index is out of range.");
    MYBREP_ASSERT_MESSAGE(replacementWire.isValid() && replacementWire.isClosed(),
                          "FaceSplit replacement Wire must be valid and closed.");

    std::vector<Topology_Wire> wires = face.wires();
    wires[wireIndex] = replacementWire;

    const Topology_Face result = rebuildFace(face, wires);
    MYBREP_ASSERT_MESSAGE(result.isForward() == face.isForward(), "FaceSplit rebuilt Face must preserve the source Face orientation.");
    return result;
}

Topology_Face splitFaceBoundaryEdge(const Topology_Face& face, std::size_t wireIndex, std::size_t edgeIndex, double parameter)
{
    MYBREP_ASSERT_MESSAGE(face.isValid(), "FaceSplit requires a valid Topology_Face.");
    MYBREP_ASSERT_MESSAGE(wireIndex < face.wireCount(), "FaceSplit Wire index is out of range.");

    const Topology_Wire wire = face.wire(wireIndex);
    MYBREP_ASSERT_MESSAGE(edgeIndex < wire.edgeCount(), "FaceSplit Edge index is out of range.");
    return replaceFaceWire(face, wireIndex, splitWireEdge(wire, edgeIndex, parameter));
}

Topology_Face splitFaceBoundaryEdge(const Topology_Face& face, std::size_t wireIndex, std::size_t edgeIndex, const std::vector<double>& parameters)
{
    MYBREP_ASSERT_MESSAGE(face.isValid(), "FaceSplit requires a valid Topology_Face.");
    MYBREP_ASSERT_MESSAGE(wireIndex < face.wireCount(), "FaceSplit Wire index is out of range.");

    const Topology_Wire wire = face.wire(wireIndex);
    MYBREP_ASSERT_MESSAGE(edgeIndex < wire.edgeCount(), "FaceSplit Edge index is out of range.");
    return replaceFaceWire(face, wireIndex, splitWireEdge(wire, edgeIndex, parameters));
}

std::vector<Topology_Face> splitFaceByEdge(const Topology_Face& face, const Topology_Edge& splittingEdge)
{
    MYBREP_ASSERT_MESSAGE(face.isValid(), "FaceSplit region split requires a valid Topology_Face.");
    MYBREP_ASSERT_MESSAGE(face.wireCount() == 1, "FaceSplit region split currently supports only Faces with exactly one trimming Wire.");
    MYBREP_ASSERT_MESSAGE(splittingEdge.isValid(), "FaceSplit region split requires a valid splitting Edge.");
    MYBREP_ASSERT_MESSAGE(splittingEdge.hasCurveOnSurface(face.geometry()),
                          "FaceSplit region splitting Edge requires a Curve-on-Surface representation on the Face Geometry_Surface.");

    const std::vector<Topology_Wire> splitWires = splitClosedWireByEdge(face.wire(0), splittingEdge);
    MYBREP_ASSERT_MESSAGE(splitWires.size() == 2, "FaceSplit region split must produce exactly two closed trimming Wires.");

    std::vector<Topology_Face> results;
    results.reserve(2);
    results.push_back(rebuildSingleWireFace(face, splitWires[0]));
    results.push_back(rebuildSingleWireFace(face, splitWires[1]));

    MYBREP_ASSERT_MESSAGE(results[0].isForward() == face.isForward() && results[1].isForward() == face.isForward(),
                          "FaceSplit region split must preserve the source Face orientation on both result Faces.");
    return results;
}

std::vector<Topology_Face> splitFaceByEdge(const Topology_Face& face, const WireSplitLocation& firstLocation,
                                           const WireSplitLocation& secondLocation, const Topology_Edge& splittingEdge, double tolerance)
{
    MYBREP_ASSERT_MESSAGE(face.isValid(), "FaceSplit boundary-location split requires a valid Topology_Face.");
    MYBREP_ASSERT_MESSAGE(face.wireCount() == 1,
                          "FaceSplit boundary-location split currently supports only Faces with exactly one trimming Wire.");
    MYBREP_ASSERT_MESSAGE(splittingEdge.isValid(), "FaceSplit boundary-location split requires a valid splitting Edge.");
    MYBREP_ASSERT_MESSAGE(splittingEdge.hasCurveOnSurface(face.geometry()),
                          "FaceSplit splitting Edge requires a Curve-on-Surface representation on the Face Geometry_Surface.");
    MYBREP_ASSERT_MESSAGE(isFiniteNonNegative(tolerance), "FaceSplit geometric tolerance must be finite and non-negative.");

    const WireBoundarySplit boundary = splitClosedWireBoundary(face.wire(0), firstLocation, secondLocation);
    const Topology_Edge sharedSplittingEdge = rebuildSplittingEdge(splittingEdge, boundary.firstVertex, boundary.secondVertex, tolerance);
    return splitFaceByEdge(rebuildSingleWireFace(face, boundary.wire), sharedSplittingEdge);
}

}
}
}
