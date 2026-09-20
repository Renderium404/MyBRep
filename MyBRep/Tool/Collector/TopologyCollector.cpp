#include "TopologyCollector.h"

#include "MyBRep/Foundation/Diagnostic.h"

#include "MyBRep/Topology/Edge/Topology_TEdge.h"
#include "MyBRep/Topology/Face/Topology_TFace.h"
#include "MyBRep/Topology/Shell/Topology_TShell.h"
#include "MyBRep/Topology/Solid/Topology_TSolid.h"
#include "MyBRep/Topology/Wire/Topology_TWire.h"

namespace
{

template<typename T>
bool containsSame(const std::vector<T>& objects, const T& candidate)
{
    for (std::size_t index = 0; index < objects.size(); ++index)
    {
        if (objects[index].isSame(candidate)) return true;
    }

    return false;
}

template<typename T>
void appendUnique(std::vector<T>& objects, const T& candidate)
{
    if (!containsSame(objects, candidate)) objects.push_back(candidate);
}

void collectWire(const MyBRep::Topology_Wire& wire, MyBRep::Tool::TopologyCollection& result)
{
    MYBREP_ASSERT_MESSAGE(wire.isValid(), "TopologyCollector requires a valid Topology_Wire.");

    if (!wire.isValid()) return;

    for (std::size_t edgeIndex = 0; edgeIndex < wire.edgeCount(); ++edgeIndex)
    {
        appendUnique(result.edges, wire.edge(edgeIndex));
    }
}

void collectFace(const MyBRep::Topology_Face& face, MyBRep::Tool::TopologyCollection& result)
{
    MYBREP_ASSERT_MESSAGE(face.isValid(), "TopologyCollector requires a valid Topology_Face.");

    if (!face.isValid()) return;

    appendUnique(result.faces, face);

    for (std::size_t wireIndex = 0; wireIndex < face.wireCount(); ++wireIndex)
    {
        collectWire(face.wire(wireIndex), result);
    }
}

void collectShell(const MyBRep::Topology_Shell& shell, MyBRep::Tool::TopologyCollection& result)
{
    MYBREP_ASSERT_MESSAGE(shell.isValid(), "TopologyCollector requires a valid Topology_Shell.");

    if (!shell.isValid()) return;

    for (std::size_t faceIndex = 0; faceIndex < shell.faceCount(); ++faceIndex)
    {
        collectFace(shell.face(faceIndex), result);
    }
}

void collectSolid(const MyBRep::Topology_Solid& solid, MyBRep::Tool::TopologyCollection& result)
{
    MYBREP_ASSERT_MESSAGE(solid.isValid(), "TopologyCollector requires a valid Topology_Solid.");

    if (!solid.isValid()) return;

    for (std::size_t shellIndex = 0; shellIndex < solid.shellCount(); ++shellIndex)
    {
        collectShell(solid.shell(shellIndex), result);
    }
}

}

namespace MyBRep
{
namespace Tool
{

TopologyCollection TopologyCollector::collect(const Topology_Object& topology)
{
    MYBREP_ASSERT_MESSAGE(topology.isValid(), "TopologyCollector requires a valid Topology_Object.");

    if (!topology.isValid()) return TopologyCollection();

    Topology_TObject* object = topology.tObject().get();
    if (object == 0) return TopologyCollection();

    if (dynamic_cast<Topology_TEdge*>(object) != 0)
        return collect(Topology_Edge(topology));

    if (dynamic_cast<Topology_TWire*>(object) != 0)
        return collect(Topology_Wire(topology));

    if (dynamic_cast<Topology_TFace*>(object) != 0)
        return collect(Topology_Face(topology));

    if (dynamic_cast<Topology_TShell*>(object) != 0)
        return collect(Topology_Shell(topology));

    if (dynamic_cast<Topology_TSolid*>(object) != 0)
        return collect(Topology_Solid(topology));

    return TopologyCollection();
}

TopologyCollection TopologyCollector::collect(const Topology_Edge& edge)
{
    MYBREP_ASSERT_MESSAGE(edge.isValid(), "TopologyCollector requires a valid Topology_Edge.");

    TopologyCollection result;

    if (edge.isValid()) result.edges.push_back(edge);

    return result;
}

TopologyCollection TopologyCollector::collect(const Topology_Wire& wire)
{
    TopologyCollection result;
    collectWire(wire, result);
    return result;
}

TopologyCollection TopologyCollector::collect(const Topology_Face& face)
{
    TopologyCollection result;
    collectFace(face, result);
    return result;
}

TopologyCollection TopologyCollector::collect(const Topology_Shell& shell)
{
    TopologyCollection result;
    collectShell(shell, result);
    return result;
}

TopologyCollection TopologyCollector::collect(const Topology_Solid& solid)
{
    TopologyCollection result;
    collectSolid(solid, result);
    return result;
}

}
}