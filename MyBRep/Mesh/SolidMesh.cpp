#include "SolidMesh.h"

#include "MyBRep/Foundation/Diagnostic.h"
#include "MyBRep/Topology/Shell/Topology_Shell.h"

namespace MyBRep
{

SolidMesh::SolidMesh()
{
}

/// 状态

bool SolidMesh::isEmpty() const
{
    return m_shellMeshes.empty();
}

bool SolidMesh::isValid() const
{
    if (!m_topology.isValid() || m_shellMeshes.size() != m_topology.shellCount())
    {
        return false;
    }

    for (std::size_t shellIndex = 0; shellIndex < m_shellMeshes.size(); ++shellIndex)
    {
        const Topology_Shell shell = m_topology.shell(shellIndex);
        const std::vector<FaceMesh>& faceMeshes = m_shellMeshes[shellIndex];

        if (faceMeshes.size() != shell.faceCount())
        {
            return false;
        }

        for (std::size_t faceIndex = 0; faceIndex < faceMeshes.size(); ++faceIndex)
        {
            if (!faceMeshes[faceIndex].isValid())
            {
                return false;
            }
        }
    }

    return true;
}

/// 拓扑来源

const Topology_Solid& SolidMesh::topology() const
{
    MYBREP_ASSERT_MESSAGE(m_topology.isValid(), "Cannot access the topology of an invalid SolidMesh.");
    return m_topology;
}

/// 网格层次

std::size_t SolidMesh::shellCount() const
{
    return m_shellMeshes.size();
}

std::size_t SolidMesh::faceCount(std::size_t shellIndex) const
{
    MYBREP_ASSERT_MESSAGE(shellIndex < shellCount(), "SolidMesh shell index is out of range.");
    return m_shellMeshes[shellIndex].size();
}

std::size_t SolidMesh::faceCount() const
{
    std::size_t count = 0;

    for (std::size_t shellIndex = 0; shellIndex < m_shellMeshes.size(); ++shellIndex)
    {
        count += m_shellMeshes[shellIndex].size();
    }

    return count;
}

const FaceMesh& SolidMesh::faceMesh(std::size_t shellIndex, std::size_t faceIndex) const
{
    MYBREP_ASSERT_MESSAGE(shellIndex < shellCount(), "SolidMesh shell index is out of range.");
    MYBREP_ASSERT_MESSAGE(faceIndex < faceCount(shellIndex), "SolidMesh face index is out of range.");
    return m_shellMeshes[shellIndex][faceIndex];
}

}