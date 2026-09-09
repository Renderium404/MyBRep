#include "FaceMesh.h"

namespace MyBRep
{

FaceMeshVertex::FaceMeshVertex()
{
}

FaceMeshVertex::FaceMeshVertex(const MyMath::Vector2& parameterValue, const MyMath::Vector3& positionValue, const MyMath::Vector3& normalValue)
    : parameter(parameterValue)
    , position(positionValue)
    , normal(normalValue)
{
}

FaceMesh::FaceMesh()
{
}

bool FaceMesh::isEmpty() const
{
    return m_vertices.empty() && m_indices.empty();
}

bool FaceMesh::isValid() const
{
    if (m_vertices.empty() || m_indices.empty() || m_indices.size() % 3 != 0)
    {
        return false;
    }

    for (std::size_t index = 0; index < m_vertices.size(); ++index)
    {
        const FaceMeshVertex& vertex = m_vertices[index];

        if (!vertex.parameter.isFinite() || !vertex.position.isFinite() || !vertex.normal.isUnit())
        {
            return false;
        }
    }

    for (std::size_t index = 0; index < m_indices.size(); ++index)
    {
        if (m_indices[index] >= m_vertices.size())
        {
            return false;
        }
    }

    return true;
}

std::size_t FaceMesh::vertexCount() const
{
    return m_vertices.size();
}

std::size_t FaceMesh::triangleCount() const
{
    return m_indices.size() / 3;
}

const std::vector<FaceMeshVertex>& FaceMesh::vertices() const
{
    return m_vertices;
}

const std::vector<unsigned int>& FaceMesh::indices() const
{
    return m_indices;
}

void FaceMesh::clear()
{
    m_vertices.clear();
    m_indices.clear();
}

unsigned int FaceMesh::addVertex(const FaceMeshVertex& vertex)
{
    m_vertices.push_back(vertex);
    return static_cast<unsigned int>(m_vertices.size() - 1);
}

void FaceMesh::addTriangle(unsigned int first, unsigned int second, unsigned int third)
{
    m_indices.push_back(first);
    m_indices.push_back(second);
    m_indices.push_back(third);
}

}
