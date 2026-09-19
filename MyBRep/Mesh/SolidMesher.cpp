#include "SolidMesher.h"

#include <vector>

#include "MyBRep/Topology/Face/Topology_Face.h"
#include "MyBRep/Topology/Shell/Topology_Shell.h"

namespace MyBRep
{

bool SolidMesher::canMesh(const Topology_Solid& solid)
{
    if (!solid.isValid())
    {
        return false;
    }

    for (std::size_t shellIndex = 0; shellIndex < solid.shellCount(); ++shellIndex)
    {
        const Topology_Shell shell = solid.shell(shellIndex);

        for (std::size_t faceIndex = 0; faceIndex < shell.faceCount(); ++faceIndex)
        {
            if (!FaceMesher::canMesh(shell.face(faceIndex)))
            {
                return false;
            }
        }
    }

    return true;
}

SolidMesh SolidMesher::mesh(const Topology_Solid& solid, const FaceMeshOptions& options)
{
    if (!canMesh(solid) || !options.isValid())
    {
        return SolidMesh();
    }

    SolidMesh result;
    result.m_topology = solid;
    result.m_shellMeshes.reserve(solid.shellCount());

    for (std::size_t shellIndex = 0; shellIndex < solid.shellCount(); ++shellIndex)
    {
        const Topology_Shell shell = solid.shell(shellIndex);
        std::vector<FaceMesh> faceMeshes;
        faceMeshes.reserve(shell.faceCount());

        for (std::size_t faceIndex = 0; faceIndex < shell.faceCount(); ++faceIndex)
        {
            const Topology_Face face = shell.face(faceIndex);
            const FaceMesh faceMesh = FaceMesher::mesh(face, options);

            if (!faceMesh.isValid())
            {
                return SolidMesh();
            }

            faceMeshes.push_back(faceMesh);
        }

        result.m_shellMeshes.push_back(faceMeshes);
    }

    return result.isValid() ? result : SolidMesh();
}

}