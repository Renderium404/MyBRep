#include "BRepShellBuilder.h"

#include <limits>
#include <vector>

#include "MyBRep/Foundation/Diagnostic.h"
#include "MyOpenGL/Resource/BufferGeometry.h"

namespace
{

bool appendFaceGeometry(const BufferGeometry& faceGeometry, std::vector<GLfloat>& vertices, std::vector<GLuint>& indices)
{
    if (faceGeometry.renderType() != RenderType::Triangles ||
        faceGeometry.valuesPerVertex() != 6 ||
        !faceGeometry.hasAttribute(GeometryAttribute::Position, 3) ||
        !faceGeometry.hasAttribute(GeometryAttribute::Normal, 3))
    {
        return false;
    }

    const std::size_t baseVertex = vertices.size() / 6;
    const std::size_t maximumIndex = static_cast<std::size_t>((std::numeric_limits<GLuint>::max)());

    if (baseVertex > maximumIndex)
    {
        return false;
    }

    const std::vector<GLfloat>& sourceVertices = faceGeometry.vertexData();
    const std::vector<GLuint>& sourceIndices = faceGeometry.indexData();

    if (sourceVertices.empty() || sourceIndices.empty() || sourceVertices.size() % 6 != 0)
    {
        return false;
    }

    vertices.insert(vertices.end(), sourceVertices.begin(), sourceVertices.end());

    for (std::size_t index = 0; index < sourceIndices.size(); ++index)
    {
        const std::size_t sourceIndex = static_cast<std::size_t>(sourceIndices[index]);

        if (sourceIndex >= static_cast<std::size_t>(faceGeometry.vertexCount()) ||
            sourceIndex > maximumIndex - baseVertex)
        {
            return false;
        }

        indices.push_back(static_cast<GLuint>(baseVertex + sourceIndex));
    }

    return true;
}

BufferGeometry* buildSurfaceGeometry(const MyBRep::Topology_Shell& shell, const MyMath::Matrix4& localToWorld, const QString& name,
                                     const MyBRep::Display::BRepShellBuildOptions& options)
{
    MYBREP_ASSERT_MESSAGE(shell.isValid(), "BRep Shell surface build requires a valid Topology_Shell.");
    MYBREP_ASSERT_MESSAGE(localToWorld.isAffine() && localToWorld.isInvertible(),
                          "BRep Shell surface transform must be an invertible affine Matrix4.");
    MYBREP_ASSERT_MESSAGE(options.isValid(), "BRep Shell build options are invalid.");

    if (!shell.isValid() || shell.faceCount() == 0 ||
        !localToWorld.isAffine() || !localToWorld.isInvertible() ||
        !options.isValid())
    {
        return 0;
    }

    std::vector<GLfloat> vertices;
    std::vector<GLuint> indices;

    for (std::size_t faceIndex = 0; faceIndex < shell.faceCount(); ++faceIndex)
    {
        const MyBRep::Topology_Face face = shell.face(faceIndex);
        BufferGeometry* faceGeometry =
            MyBRep::Display::BRepFaceBuilder::build(face, localToWorld, name + "_FaceSurface", options.surface);

        if (faceGeometry == 0)
        {
            return 0;
        }

        const bool appended = appendFaceGeometry(*faceGeometry, vertices, indices);
        delete faceGeometry;

        if (!appended)
        {
            return 0;
        }
    }

    if (vertices.empty() || indices.empty())
    {
        return 0;
    }

    BufferGeometry* geometry = new BufferGeometry(name, BufferUsage::Static, RenderType::Triangles);

    std::vector<GeometryVertexAttribute> attributes;

    GeometryVertexAttribute positionAttribute;
    positionAttribute.location = GeometryAttribute::Position;
    positionAttribute.componentCount = 3;
    positionAttribute.valueOffset = 0;
    attributes.push_back(positionAttribute);

    GeometryVertexAttribute normalAttribute;
    normalAttribute.location = GeometryAttribute::Normal;
    normalAttribute.componentCount = 3;
    normalAttribute.valueOffset = 3;
    attributes.push_back(normalAttribute);

    geometry->setVertexLayout(6, attributes);
    geometry->setVertexData(vertices);
    geometry->setIndexData(indices);

    return geometry;
}

}

namespace MyBRep
{
namespace Display
{

BRepShellBuildOptions::BRepShellBuildOptions()
{
}

bool BRepShellBuildOptions::isValid() const
{
    return surface.isValid() && wireframe.isValid();
}

BufferGeometry* BRepShellBuilder::buildSurface(const Topology_Shell& shell, const QString& name, const BRepShellBuildOptions& options)
{
    return buildSurface(shell, MyMath::Matrix4::identity(), name, options);
}

BufferGeometry* BRepShellBuilder::buildSurface(const Topology_Shell& shell, const MyMath::Matrix4& localToWorld, const QString& name,
                                               const BRepShellBuildOptions& options)
{
    return buildSurfaceGeometry(shell, localToWorld, name, options);
}

BufferGeometry* BRepShellBuilder::buildBoundary(const Topology_Shell& shell, const QString& name, const BRepShellBuildOptions& options)
{
    return buildBoundary(shell, MyMath::Matrix4::identity(), name, options);
}

BufferGeometry* BRepShellBuilder::buildBoundary(const Topology_Shell& shell, const MyMath::Matrix4& localToWorld, const QString& name,
                                                const BRepShellBuildOptions& options)
{
    if (!shell.isValid() || !options.isValid())
    {
        return 0;
    }

    return BRepWireframeBuilder::build(shell, localToWorld, name, options.wireframe);
}

}
}
