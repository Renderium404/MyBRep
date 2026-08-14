#include "FaceSplit.h"

#include "MyBRep/Foundation/Diagnostic.h"
#include "MyBRep/Operation/Split/WireSplit.h"

namespace
{

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
    MYBREP_ASSERT_MESSAGE(result.isForward() == face.isForward(),
                          "FaceSplit rebuilt Face must preserve the source Face orientation.");
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

}
}
}