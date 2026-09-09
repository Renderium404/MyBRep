#include "BRepFaceBuilder.h"

#include <vector>

#include "MyBRep/Foundation/Diagnostic.h"
#include "MyOpenGL/Resource/BufferGeometry.h"

namespace
{

bool buildNormalTransform(const MyMath::Matrix4& localToWorld, MyMath::Matrix4& normalTransform, double& orientationSign)
{
    MyMath::Matrix4 inverse;

    if (!localToWorld.inverted(inverse))
    {
        return false;
    }

    normalTransform = inverse.transposed();

    // 对可逆仿射变换，4x4行列式符号等于三维线性部分的方向保持/翻转符号。
    orientationSign = localToWorld.determinant() >= 0.0 ? 1.0 : -1.0;
    return true;
}

MyMath::Vector3 transformedNormal(const MyMath::Vector3& normal, const MyMath::Matrix4& normalTransform, double orientationSign)
{
    // 一般仿射下法向使用逆转置变换；方向翻转仿射额外乘行列式符号，使结果与变换后Surface参数方向及三角形绕序一致。
    MyMath::Vector3 result = normalTransform.transformVector(normal) * orientationSign;

    MYBREP_ASSERT_MESSAGE(result.isVector(0.0), "BRep Face transformed normal must remain a finite non-zero vector.");

    result.normalize(0.0);
    return result;
}

BufferGeometry* buildGeometry(const MyBRep::Topology_Face& face, const MyMath::Matrix4& localToWorld, const QString& name, const MyBRep::Display::BRepFaceBuildOptions& options)
{
    MYBREP_ASSERT_MESSAGE(localToWorld.isAffine() && localToWorld.isInvertible(), "BRep Face transform must be an invertible affine Matrix4.");
    MYBREP_ASSERT_MESSAGE(options.isValid(), "BRep Face build options are invalid.");

    if (!face.isValid() || !localToWorld.isAffine() || !localToWorld.isInvertible() || !options.isValid())
    {
        return 0;
    }

    MyBRep::FaceMeshOptions meshOptions;
    meshOptions.planar = options.meshing;
    meshOptions.cylindrical = options.cylindricalMeshing;
    meshOptions.spherical = options.sphericalMeshing;
    meshOptions.conical = options.conicalMeshing;

    const MyBRep::FaceMesh mesh = MyBRep::FaceMesher::mesh(face, meshOptions);

    if (!mesh.isValid())
    {
        return 0;
    }

    MyMath::Matrix4 normalTransform;
    double orientationSign = 1.0;

    if (!buildNormalTransform(localToWorld, normalTransform, orientationSign))
    {
        return 0;
    }

    std::vector<GLfloat> vertices;
    vertices.reserve(mesh.vertexCount() * 6); // 每个显示顶点固定包含Position.xyz和Normal.xyz六个GLfloat。

    for (std::size_t index = 0; index < mesh.vertexCount(); ++index)
    {
        const MyBRep::FaceMeshVertex& sourceVertex = mesh.vertices()[index];
        const MyMath::Vector3 position = localToWorld.transformPoint(sourceVertex.position);
        const MyMath::Vector3 normal = transformedNormal(sourceVertex.normal, normalTransform, orientationSign);

        if (!position.isFinite() || !normal.isUnit())
        {
            return 0;
        }

        vertices.push_back(static_cast<GLfloat>(position.x()));
        vertices.push_back(static_cast<GLfloat>(position.y()));
        vertices.push_back(static_cast<GLfloat>(position.z()));
        vertices.push_back(static_cast<GLfloat>(normal.x()));
        vertices.push_back(static_cast<GLfloat>(normal.y()));
        vertices.push_back(static_cast<GLfloat>(normal.z()));
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

namespace MyBRep
{
namespace Display
{

BRepFaceBuildOptions::BRepFaceBuildOptions()
{
}

bool BRepFaceBuildOptions::isValid() const
{
    return meshing.isValid() && cylindricalMeshing.isValid() && sphericalMeshing.isValid() && conicalMeshing.isValid();
}

BufferGeometry* BRepFaceBuilder::build(const Topology_Face& face, const QString& name, const BRepFaceBuildOptions& options)
{
    return build(face, MyMath::Matrix4::identity(), name, options);
}

BufferGeometry* BRepFaceBuilder::build(const Topology_Face& face, const MyMath::Matrix4& localToWorld, const QString& name, const BRepFaceBuildOptions& options)
{
    return buildGeometry(face, localToWorld, name, options);
}

}
}
