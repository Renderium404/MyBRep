#include "BRepSolidBuilder.h"

#include <limits>
#include <vector>

#include "MyBRep/Foundation/Diagnostic.h"
#include "MyBRepOpenGL/Builder/BRepShellBuilder.h"
#include "MyOpenGL/Resource/BufferGeometry.h"

namespace
{

bool appendShellGeometry(const BufferGeometry& shellGeometry, std::vector<GLfloat>& vertices, std::vector<GLuint>& indices)
{
    if (shellGeometry.renderType() != RenderType::Triangles ||
        shellGeometry.valuesPerVertex() != 6 ||
        !shellGeometry.hasAttribute(GeometryAttribute::Position, 3) ||
        !shellGeometry.hasAttribute(GeometryAttribute::Normal, 3))
    {
        return false;
    }

    const std::size_t baseVertex = vertices.size() / 6;
    const std::size_t maximumIndex = static_cast<std::size_t>((std::numeric_limits<GLuint>::max)());

    if (baseVertex > maximumIndex)
    {
        return false;
    }

    const std::vector<GLfloat>& sourceVertices = shellGeometry.vertexData();
    const std::vector<GLuint>& sourceIndices = shellGeometry.indexData();

    if (sourceVertices.empty() || sourceIndices.empty() || sourceVertices.size() % 6 != 0)
    {
        return false;
    }

    vertices.insert(vertices.end(), sourceVertices.begin(), sourceVertices.end());

    for (std::size_t index = 0; index < sourceIndices.size(); ++index)
    {
        const std::size_t sourceIndex = static_cast<std::size_t>(sourceIndices[index]);

        if (sourceIndex >= static_cast<std::size_t>(shellGeometry.vertexCount()) ||
            sourceIndex > maximumIndex - baseVertex)
        {
            return false;
        }

        indices.push_back(static_cast<GLuint>(baseVertex + sourceIndex));
    }

    return true;
}

BufferGeometry* buildSurfaceGeometry(const MyBRep::Topology_Solid& solid, const MyMath::Matrix4& localToWorld, const QString& name,
                                     const MyBRep::Display::BRepSolidBuildOptions& options)
{
    MYBREP_ASSERT_MESSAGE(solid.isValid(), "BRep Solid surface build requires a valid Topology_Solid.");
    MYBREP_ASSERT_MESSAGE(localToWorld.isAffine() && localToWorld.isInvertible(),
                          "BRep Solid surface transform must be an invertible affine Matrix4.");
    MYBREP_ASSERT_MESSAGE(options.isValid(), "BRep Solid build options are invalid.");

    if (!solid.isValid() || solid.shellCount() == 0 ||
        !localToWorld.isAffine() || !localToWorld.isInvertible() ||
        !options.isValid())
    {
        return 0;
    }

    MyBRep::Display::BRepShellBuildOptions shellOptions;
    shellOptions.surface = options.surface;
    shellOptions.wireframe = options.wireframe;

    std::vector<GLfloat> vertices;
    std::vector<GLuint> indices;

    for (std::size_t shellIndex = 0; shellIndex < solid.shellCount(); ++shellIndex)
    {
        BufferGeometry* shellGeometry =
            MyBRep::Display::BRepShellBuilder::buildSurface(
                solid.shell(shellIndex),
                localToWorld,
                name + "_ShellSurface",
                shellOptions);

        if (shellGeometry == 0)
        {
            return 0;
        }

        const bool appended = appendShellGeometry(*shellGeometry, vertices, indices);
        delete shellGeometry;

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

BRepSolidBuildOptions::BRepSolidBuildOptions()
{
}

bool BRepSolidBuildOptions::isValid() const
{
    return surface.isValid() && wireframe.isValid();
}

BufferGeometry* BRepSolidBuilder::buildSurface(const Topology_Solid& solid, const QString& name, const BRepSolidBuildOptions& options)
{
    return buildSurface(solid, MyMath::Matrix4::identity(), name, options);
}

BufferGeometry* BRepSolidBuilder::buildSurface(const Topology_Solid& solid, const MyMath::Matrix4& localToWorld, const QString& name,
                                               const BRepSolidBuildOptions& options)
{
    return buildSurfaceGeometry(solid, localToWorld, name, options);
}

BufferGeometry* BRepSolidBuilder::buildBoundary(const Topology_Solid& solid, const QString& name, const BRepSolidBuildOptions& options)
{
    return buildBoundary(solid, MyMath::Matrix4::identity(), name, options);
}

BufferGeometry* BRepSolidBuilder::buildBoundary(const Topology_Solid& solid, const MyMath::Matrix4& localToWorld, const QString& name,
                                                const BRepSolidBuildOptions& options)
{
    if (!solid.isValid() || !options.isValid())
    {
        return 0;
    }

    return BRepWireframeBuilder::build(solid, localToWorld, name, options.wireframe);
}

}
}
