#include "SphericalFaceMesher.h"

#include <algorithm>
#include <limits>
#include <set>
#include <vector>

#include "MyBRep/Geometry/Surface/Geometry_SphericalSurface.h"
#include "MyBRep/Geometry/Surface/SurfaceKind.h"

namespace
{

// 返回参数点所属球面极点：-1为南极，1为北极，0为正则参数。
int parameterPoleSide(const MyBRep::Geometry_SphericalSurface& sphere, const MyMath::Vector2& parameter, double tolerance)
{
    if (parameter.y() <= sphere.vDomainStart() + tolerance)
    {
        return -1;
    }

    if (parameter.y() >= sphere.vDomainEnd() - tolerance)
    {
        return 1;
    }

    return 0;
}

// 将极点V参数规范化到球面自然参数域端点，同时保留原始U参数。
MyMath::Vector2 canonicalPoleParameter(const MyBRep::Geometry_SphericalSurface& sphere, const MyMath::Vector2& parameter, double tolerance)
{
    const int side = parameterPoleSide(sphere, parameter, tolerance);

    if (side < 0)
    {
        return MyMath::Vector2(parameter.x(), sphere.vDomainStart());
    }

    if (side > 0)
    {
        return MyMath::Vector2(parameter.x(), sphere.vDomainEnd());
    }

    return parameter;
}

class SphereSingularityHandler : public MyBRep::ParametricFaceSingularityHandler
{
public:
    bool isSingular(const MyBRep::Geometry_Surface& surface, const MyMath::Vector2& parameter, double tolerance) const override
    {
        if (surface.kind() != MyBRep::SurfaceKind::Spherical)
        {
            return false;
        }

        const MyBRep::Geometry_SphericalSurface& sphere = static_cast<const MyBRep::Geometry_SphericalSurface&>(surface);
        return parameterPoleSide(sphere, parameter, tolerance) != 0;
    }

    bool parametersMeetAtSingularity(const MyBRep::Geometry_Surface& surface, const MyMath::Vector2& first,
                                     const MyMath::Vector2& second, double tolerance) const override
    {
        if (surface.kind() != MyBRep::SurfaceKind::Spherical)
        {
            return false;
        }

        const MyBRep::Geometry_SphericalSurface& sphere = static_cast<const MyBRep::Geometry_SphericalSurface&>(surface);
        const int firstSide = parameterPoleSide(sphere, first, tolerance);
        return firstSide != 0 && firstSide == parameterPoleSide(sphere, second, tolerance);
    }

    bool normalAt(const MyBRep::Topology_Face& face, const MyMath::Vector2& singularParameter,
                  const MyMath::Vector2& regularApproachParameter, double tolerance, MyMath::Vector3& normal) const override
    {
        if (face.geometry().kind() != MyBRep::SurfaceKind::Spherical)
        {
            return false;
        }

        const MyBRep::Geometry_SphericalSurface& sphere = static_cast<const MyBRep::Geometry_SphericalSurface&>(face.geometry());

        if (!isSingular(sphere, singularParameter, tolerance))
        {
            return false;
        }

        (void)regularApproachParameter;

        const MyMath::Vector2 parameter = canonicalPoleParameter(sphere, singularParameter, tolerance);
        normal = face.normalAt(parameter.x(), parameter.y());
        return normal.isUnit();
    }
};

struct OutputTriangleKey
{
    OutputTriangleKey(unsigned int firstValue, unsigned int secondValue, unsigned int thirdValue)
    {
        unsigned int values[3] = { firstValue, secondValue, thirdValue };

        if (values[0] > values[1])
        {
            std::swap(values[0], values[1]);
        }

        if (values[1] > values[2])
        {
            std::swap(values[1], values[2]);
        }

        if (values[0] > values[1])
        {
            std::swap(values[0], values[1]);
        }

        first = values[0];
        second = values[1];
        third = values[2];
    }

    bool operator<(const OutputTriangleKey& other) const
    {
        return first < other.first || (first == other.first && (second < other.second || (second == other.second && third < other.third)));
    }

    unsigned int first;
    unsigned int second;
    unsigned int third;
};

// 将Core保留的不同U极点参数顶点合并为球面唯一极点顶点，并删除合并后退化或重复的三角形。
MyBRep::FaceMesh mergePoleVertices(const MyBRep::FaceMesh& source, const MyBRep::Geometry_SphericalSurface& sphere, double tolerance)
{
    if (!source.isValid())
    {
        return MyBRep::FaceMesh();
    }

    MyBRep::FaceMesh result;
    std::vector<unsigned int> outputIndices(source.vertexCount(), 0);
    const unsigned int invalidIndex = (std::numeric_limits<unsigned int>::max)();
    unsigned int southPoleIndex = invalidIndex;
    unsigned int northPoleIndex = invalidIndex;

    for (std::size_t index = 0; index < source.vertexCount(); ++index)
    {
        const MyBRep::FaceMeshVertex& sourceVertex = source.vertices()[index];
        const MyMath::Vector2 parameter = canonicalPoleParameter(sphere, sourceVertex.parameter, tolerance);
        const int poleSide = parameterPoleSide(sphere, parameter, tolerance);

        if (poleSide < 0 && southPoleIndex != invalidIndex)
        {
            outputIndices[index] = southPoleIndex;
            continue;
        }

        if (poleSide > 0 && northPoleIndex != invalidIndex)
        {
            outputIndices[index] = northPoleIndex;
            continue;
        }

        const MyMath::Vector3 position = sphere.pointAt(parameter.x(), parameter.y());
        const unsigned int outputIndex = result.addVertex(MyBRep::FaceMeshVertex(parameter, position, sourceVertex.normal));
        outputIndices[index] = outputIndex;

        if (poleSide < 0)
        {
            southPoleIndex = outputIndex;
        }
        else if (poleSide > 0)
        {
            northPoleIndex = outputIndex;
        }
    }

    std::set<OutputTriangleKey> emittedTriangles;
    const std::vector<unsigned int>& sourceIndices = source.indices();

    for (std::size_t index = 0; index + 2 < sourceIndices.size(); index += 3)
    {
        const unsigned int first = outputIndices[sourceIndices[index]];
        const unsigned int second = outputIndices[sourceIndices[index + 1]];
        const unsigned int third = outputIndices[sourceIndices[index + 2]];

        if (first == second || second == third || third == first)
        {
            continue;
        }

        const OutputTriangleKey key(first, second, third);

        if (!emittedTriangles.insert(key).second)
        {
            continue;
        }

        result.addTriangle(first, second, third);
    }

    return result.isValid() ? result : MyBRep::FaceMesh();
}

}

namespace MyBRep
{

SphericalFaceMeshOptions::SphericalFaceMeshOptions()
    : ParametricFaceMeshOptions()
{
}

bool SphericalFaceMesher::canMesh(const Topology_Face& face)
{
    if (!face.isValid() || face.geometry().kind() != SurfaceKind::Spherical)
    {
        return false;
    }

    const Geometry_SphericalSurface& sphere = static_cast<const Geometry_SphericalSurface&>(face.geometry());

    if (!sphere.isUPeriodic() || sphere.uPeriod() <= 0.0 || sphere.isVPeriodic() || !sphere.isVDomainBounded())
    {
        return false;
    }

    SphereSingularityHandler singularityHandler;
    ParametricFaceMeshPolicy policy;
    policy.periodicU = true;
    policy.periodicV = false;
    policy.rejectSingularParameters = false;
    policy.singularityHandler = &singularityHandler;

    return ParametricFaceMesherCore::canMesh(face, policy);
}

FaceMesh SphericalFaceMesher::mesh(const Topology_Face& face, const SphericalFaceMeshOptions& options)
{
    if (!canMesh(face) || !options.isValid())
    {
        return FaceMesh();
    }

    const Geometry_SphericalSurface& sphere = static_cast<const Geometry_SphericalSurface&>(face.geometry());

    SphereSingularityHandler singularityHandler;
    ParametricFaceMeshPolicy policy;
    policy.periodicU = true;
    policy.periodicV = false;
    policy.rejectSingularParameters = false;
    policy.singularityHandler = &singularityHandler;

    const FaceMesh coreMesh = ParametricFaceMesherCore::mesh(face, options, policy);
    return mergePoleVertices(coreMesh, sphere, options.geometricTolerance);
}

}

