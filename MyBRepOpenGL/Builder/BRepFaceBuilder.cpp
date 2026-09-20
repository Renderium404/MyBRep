#include "BRepFaceBuilder.h"

#include <vector>

#include "MyBRep/Foundation/Diagnostic.h"
#include "MyOpenGL/Resource/BufferGeometry.h"

namespace MyBRep
{
namespace Display
{

BRepFaceBuildOptions::BRepFaceBuildOptions()
{
}

bool BRepFaceBuildOptions::isValid() const
{
    return meshing.isValid() &&
           cylindricalMeshing.isValid() &&
           sphericalMeshing.isValid() &&
           conicalMeshing.isValid() &&
           extrudedMeshing.isValid() &&
           revolvedMeshing.isValid() &&
           bezierMeshing.isValid() &&
           bsplineMeshing.isValid();
}

BufferGeometry* BRepFaceBuilder::build(const Topology_Face& face, const QString& name, const BRepFaceBuildOptions& options)
{
    MYBREP_ASSERT_MESSAGE(face.isValid(), "BRep Face build requires a valid Topology_Face.");
    MYBREP_ASSERT_MESSAGE(options.isValid(), "BRep Face build options are invalid.");

    if (!face.isValid() || !options.isValid()) return 0;

    FaceMeshOptions meshOptions;
    meshOptions.planar = options.meshing;
    meshOptions.cylindrical = options.cylindricalMeshing;
    meshOptions.spherical = options.sphericalMeshing;
    meshOptions.conical = options.conicalMeshing;
    meshOptions.extruded = options.extrudedMeshing;
    meshOptions.revolved = options.revolvedMeshing;
    meshOptions.bezier = options.bezierMeshing;
    meshOptions.bspline = options.bsplineMeshing;

    const FaceMesh mesh = FaceMesher::mesh(face, meshOptions);
    if (!mesh.isValid()) return 0;

    std::vector<GLfloat> vertices;
    vertices.reserve(mesh.vertexCount() * 6);

    for (std::size_t index = 0; index < mesh.vertexCount(); ++index)
    {
        const FaceMeshVertex& vertex = mesh.vertices()[index];

        if (!vertex.position.isFinite() || !vertex.normal.isUnit()) return 0;

        vertices.push_back(static_cast<GLfloat>(vertex.position.x()));
        vertices.push_back(static_cast<GLfloat>(vertex.position.y()));
        vertices.push_back(static_cast<GLfloat>(vertex.position.z()));
        vertices.push_back(static_cast<GLfloat>(vertex.normal.x()));
        vertices.push_back(static_cast<GLfloat>(vertex.normal.y()));
        vertices.push_back(static_cast<GLfloat>(vertex.normal.z()));
    }

    std::vector<GLuint> indices;
    indices.reserve(mesh.indices().size());

    for (std::size_t index = 0; index < mesh.indices().size(); ++index)
    {
        indices.push_back(static_cast<GLuint>(mesh.indices()[index]));
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
}