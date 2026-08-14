#include "Topology_TFace.h"

#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

Topology_TFace::Topology_TFace(const Foundation::RefPtr<const Geometry_Surface>& geometry,
                               const std::vector<Topology_Wire>& wires)
    : m_geometry(geometry)
    , m_wires(wires)
{
    MYBREP_ASSERT_MESSAGE(geometry,
                          "Topology_TFace requires a non-null Geometry_Surface.");

    for (std::size_t wireIndex = 0; wireIndex < m_wires.size(); ++wireIndex)
    {
        const Topology_Wire& currentWire = m_wires[wireIndex];

        MYBREP_ASSERT_MESSAGE(currentWire.isValid(),
                              "Topology_TFace trimming wires must be valid.");
        MYBREP_ASSERT_MESSAGE(currentWire.isClosed(),
                              "Topology_TFace trimming wires must be closed.");

        for (std::size_t edgeIndex = 0; edgeIndex < currentWire.edgeCount(); ++edgeIndex)
        {
            const Topology_Edge edge = currentWire.edge(edgeIndex);

            MYBREP_ASSERT_MESSAGE(edge.hasCurveOnSurface(*m_geometry),
                                  "Every Topology_TFace trimming Edge requires a Curve-on-Surface representation on the Face geometry.");
        }
    }
}

/// 曲面几何

const Geometry_Surface& Topology_TFace::geometry() const
{
    return *m_geometry;
}

const Foundation::RefPtr<const Geometry_Surface>& Topology_TFace::geometryResource() const
{
    return m_geometry;
}

/// 裁剪Wire

std::size_t Topology_TFace::wireCount() const
{
    return m_wires.size();
}

const Topology_Wire& Topology_TFace::wire(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(index < m_wires.size(),
                          "Topology_TFace wire index is out of range.");

    return m_wires[index];
}

const std::vector<Topology_Wire>& Topology_TFace::wires() const
{
    return m_wires;
}

}
