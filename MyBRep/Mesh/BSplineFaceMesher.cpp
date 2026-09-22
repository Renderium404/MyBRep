#include "BSplineFaceMesher.h"

#include <cmath>
#include <vector>

#include "MyBRep/Geometry/Surface/Geometry_BSplineSurface.h"
#include "MyBRep/Geometry/Surface/SurfaceKind.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

namespace
{

struct BilinearPatch
{
    double u0;
    double u1;
    double v0;
    double v1;
    MyMath::Vector3 p00;
    MyMath::Vector3 p10;
    MyMath::Vector3 p11;
    MyMath::Vector3 p01;
};

bool nearlyEqual(double first, double second, double tolerance)
{
    return std::fabs(first - second) <= tolerance;
}

int naturalDomainCornerIndex(const MyMath::Vector2& parameter,
                             double u0,
                             double u1,
                             double v0,
                             double v1,
                             double tolerance)
{
    const bool atU0 = nearlyEqual(parameter.x(), u0, tolerance);
    const bool atU1 = nearlyEqual(parameter.x(), u1, tolerance);
    const bool atV0 = nearlyEqual(parameter.y(), v0, tolerance);
    const bool atV1 = nearlyEqual(parameter.y(), v1, tolerance);

    if (atU0 && atV0) return 0;
    if (atU1 && atV0) return 1;
    if (atU1 && atV1) return 2;
    if (atU0 && atV1) return 3;
    return -1;
}

int naturalDomainBoundaryIndex(int firstCorner, int secondCorner)
{
    if ((firstCorner == 0 && secondCorner == 1) || (firstCorner == 1 && secondCorner == 0)) return 0;
    if ((firstCorner == 1 && secondCorner == 2) || (firstCorner == 2 && secondCorner == 1)) return 1;
    if ((firstCorner == 2 && secondCorner == 3) || (firstCorner == 3 && secondCorner == 2)) return 2;
    if ((firstCorner == 3 && secondCorner == 0) || (firstCorner == 0 && secondCorner == 3)) return 3;
    return -1;
}

bool isNaturalDomainRectangle(const MyBRep::Topology_Face& face,
                              const MyBRep::Geometry_BSplineSurface& surface,
                              double tolerance)
{
    if (face.wireCount() != 1) return false;

    const MyBRep::Topology_Wire wire = face.wire(0);
    if (!wire.isValid() || !wire.isClosed() || wire.edgeCount() != 4) return false;

    const double u0 = surface.uDomainStart();
    const double u1 = surface.uDomainEnd();
    const double v0 = surface.vDomainStart();
    const double v1 = surface.vDomainEnd();
    bool boundaries[4] = { false, false, false, false };

    for (std::size_t edgeIndex = 0; edgeIndex < wire.edgeCount(); ++edgeIndex)
    {
        const MyBRep::Topology_Edge edge = wire.edge(edgeIndex);
        if (!edge.isValid() || !edge.hasCurveOnSurface(surface)) return false;

        const int firstCorner = naturalDomainCornerIndex(edge.surfaceParameterAt(surface, 0.0), u0, u1, v0, v1, tolerance);
        const int secondCorner = naturalDomainCornerIndex(edge.surfaceParameterAt(surface, 1.0), u0, u1, v0, v1, tolerance);
        const int boundaryIndex = naturalDomainBoundaryIndex(firstCorner, secondCorner);
        if (boundaryIndex < 0 || boundaries[boundaryIndex]) return false;
        boundaries[boundaryIndex] = true;
    }

    return boundaries[0] && boundaries[1] && boundaries[2] && boundaries[3];
}

bool collectDegreeOneSpanParameters(const MyBRep::Geometry_BSplineSurface& surface,
                                    bool uDirection,
                                    double tolerance,
                                    std::vector<double>& parameters)
{
    parameters.clear();

    const std::size_t degree = uDirection ? surface.uDegree() : surface.vDegree();
    const std::size_t controlPointCount = uDirection ? surface.uControlPointCount() : surface.vControlPointCount();
    if (degree != 1 || controlPointCount < 2) return false;

    parameters.reserve(controlPointCount);

    for (std::size_t index = degree; index <= controlPointCount; ++index)
    {
        const double value = uDirection ? surface.uKnot(index) : surface.vKnot(index);
        if (!parameters.empty() && value <= parameters.back() + tolerance) return false;
        parameters.push_back(value);
    }

    return parameters.size() == controlPointCount;
}

MyMath::Vector3 patchNormalVector(const BilinearPatch& patch, double s, double t)
{
    const MyMath::Vector3 derivativeU = (patch.p10 - patch.p00) * (1.0 - t) + (patch.p11 - patch.p01) * t;
    const MyMath::Vector3 derivativeV = (patch.p01 - patch.p00) * (1.0 - s) + (patch.p11 - patch.p10) * s;
    return MyMath::Vector3::cross(derivativeU, derivativeV);
}

bool patchNormal(const MyBRep::Topology_Face& face,
                 const BilinearPatch& patch,
                 double s,
                 double t,
                 MyMath::Vector3& normal)
{
    normal = patchNormalVector(patch, s, t);
    if (!normal.isVector(0.0)) normal = patchNormalVector(patch, 0.5, 0.5);
    if (!normal.isVector(0.0)) return false;

    normal.normalize(0.0);
    if (!face.isForward()) normal *= -1.0;
    return normal.isUnit();
}

bool patchDiagonal(const BilinearPatch& patch, int& diagonal)
{
    const MyMath::Vector3 firstA = MyMath::Vector3::cross(patch.p10 - patch.p00, patch.p11 - patch.p00);
    const MyMath::Vector3 firstB = MyMath::Vector3::cross(patch.p11 - patch.p00, patch.p01 - patch.p00);

    if (firstA.isVector(0.0) && firstB.isVector(0.0))
    {
        diagonal = 0;
        return true;
    }

    const MyMath::Vector3 secondA = MyMath::Vector3::cross(patch.p10 - patch.p00, patch.p01 - patch.p00);
    const MyMath::Vector3 secondB = MyMath::Vector3::cross(patch.p11 - patch.p10, patch.p01 - patch.p10);

    if (secondA.isVector(0.0) && secondB.isVector(0.0))
    {
        diagonal = 1;
        return true;
    }

    return false;
}

double patchChordError(const BilinearPatch& patch)
{
    const MyMath::Vector3 surfaceCenter = (patch.p00 + patch.p10 + patch.p11 + patch.p01) * 0.25;
    const MyMath::Vector3 triangleCenter = (patch.p00 + patch.p11) * 0.5;
    return (surfaceCenter - triangleCenter).length();
}

bool appendPatch(const MyBRep::Topology_Face& face, const BilinearPatch& patch, int diagonal, MyBRep::FaceMesh& mesh)
{
    MyMath::Vector3 normal00;
    MyMath::Vector3 normal10;
    MyMath::Vector3 normal11;
    MyMath::Vector3 normal01;

    if (!patchNormal(face, patch, 0.0, 0.0, normal00) ||
        !patchNormal(face, patch, 1.0, 0.0, normal10) ||
        !patchNormal(face, patch, 1.0, 1.0, normal11) ||
        !patchNormal(face, patch, 0.0, 1.0, normal01))
    {
        return false;
    }

    const unsigned int index00 = mesh.addVertex(MyBRep::FaceMeshVertex(MyMath::Vector2(patch.u0, patch.v0), patch.p00, normal00));
    const unsigned int index10 = mesh.addVertex(MyBRep::FaceMeshVertex(MyMath::Vector2(patch.u1, patch.v0), patch.p10, normal10));
    const unsigned int index11 = mesh.addVertex(MyBRep::FaceMeshVertex(MyMath::Vector2(patch.u1, patch.v1), patch.p11, normal11));
    const unsigned int index01 = mesh.addVertex(MyBRep::FaceMeshVertex(MyMath::Vector2(patch.u0, patch.v1), patch.p01, normal01));

    if (face.isForward())
    {
        if (diagonal == 0)
        {
            mesh.addTriangle(index00, index10, index11);
            mesh.addTriangle(index00, index11, index01);
        }
        else
        {
            mesh.addTriangle(index00, index10, index01);
            mesh.addTriangle(index10, index11, index01);
        }
    }
    else
    {
        if (diagonal == 0)
        {
            mesh.addTriangle(index00, index11, index10);
            mesh.addTriangle(index00, index01, index11);
        }
        else
        {
            mesh.addTriangle(index00, index01, index10);
            mesh.addTriangle(index10, index01, index11);
        }
    }

    return true;
}

void subdividePatch(const BilinearPatch& patch, BilinearPatch children[4])
{
    const double um = (patch.u0 + patch.u1) * 0.5;
    const double vm = (patch.v0 + patch.v1) * 0.5;
    const MyMath::Vector3 bottom = (patch.p00 + patch.p10) * 0.5;
    const MyMath::Vector3 right = (patch.p10 + patch.p11) * 0.5;
    const MyMath::Vector3 top = (patch.p01 + patch.p11) * 0.5;
    const MyMath::Vector3 left = (patch.p00 + patch.p01) * 0.5;
    const MyMath::Vector3 center = (patch.p00 + patch.p10 + patch.p11 + patch.p01) * 0.25;

    children[0].u0 = patch.u0; children[0].u1 = um; children[0].v0 = patch.v0; children[0].v1 = vm;
    children[0].p00 = patch.p00; children[0].p10 = bottom; children[0].p11 = center; children[0].p01 = left;

    children[1].u0 = um; children[1].u1 = patch.u1; children[1].v0 = patch.v0; children[1].v1 = vm;
    children[1].p00 = bottom; children[1].p10 = patch.p10; children[1].p11 = right; children[1].p01 = center;

    children[2].u0 = um; children[2].u1 = patch.u1; children[2].v0 = vm; children[2].v1 = patch.v1;
    children[2].p00 = center; children[2].p10 = right; children[2].p11 = patch.p11; children[2].p01 = top;

    children[3].u0 = patch.u0; children[3].u1 = um; children[3].v0 = vm; children[3].v1 = patch.v1;
    children[3].p00 = left; children[3].p10 = center; children[3].p11 = top; children[3].p01 = patch.p01;
}

bool appendAdaptivePatch(const MyBRep::Topology_Face& face,
                         const BilinearPatch& patch,
                         const MyBRep::BSplineFaceMeshOptions& options,
                         int depth,
                         MyBRep::FaceMesh& mesh)
{
    int diagonal = -1;
    const bool triangulatable = patchDiagonal(patch, diagonal);
    const bool chordSatisfied = patchChordError(patch) <= options.surfaceChordTolerance;

    if (triangulatable && chordSatisfied) return appendPatch(face, patch, diagonal, mesh);
    if (depth >= options.maximumSurfaceSubdivisionRounds) return false;

    BilinearPatch children[4];
    subdividePatch(patch, children);

    for (int index = 0; index < 4; ++index)
    {
        if (!appendAdaptivePatch(face, children[index], options, depth + 1, mesh)) return false;
    }

    return true;
}

MyBRep::FaceMesh meshDegreeOneNaturalDomain(const MyBRep::Topology_Face& face,
                                             const MyBRep::Geometry_BSplineSurface& surface,
                                             const MyBRep::BSplineFaceMeshOptions& options)
{
    std::vector<double> uParameters;
    std::vector<double> vParameters;

    if (!collectDegreeOneSpanParameters(surface, true, options.geometricTolerance, uParameters) ||
        !collectDegreeOneSpanParameters(surface, false, options.geometricTolerance, vParameters))
    {
        return MyBRep::FaceMesh();
    }

    MyBRep::FaceMesh result;

    for (std::size_t vIndex = 0; vIndex + 1 < vParameters.size(); ++vIndex)
    {
        for (std::size_t uIndex = 0; uIndex + 1 < uParameters.size(); ++uIndex)
        {
            BilinearPatch patch;
            patch.u0 = uParameters[uIndex];
            patch.u1 = uParameters[uIndex + 1];
            patch.v0 = vParameters[vIndex];
            patch.v1 = vParameters[vIndex + 1];
            patch.p00 = surface.pointAt(patch.u0, patch.v0);
            patch.p10 = surface.pointAt(patch.u1, patch.v0);
            patch.p11 = surface.pointAt(patch.u1, patch.v1);
            patch.p01 = surface.pointAt(patch.u0, patch.v1);

            if (!appendAdaptivePatch(face, patch, options, 0, result)) return MyBRep::FaceMesh();
        }
    }

    return result.isValid() ? result : MyBRep::FaceMesh();
}

}

namespace MyBRep
{

BSplineFaceMeshOptions::BSplineFaceMeshOptions()
    : ParametricFaceMeshOptions()
{
}

bool BSplineFaceMesher::canMesh(const Topology_Face& face)
{
    if (!face.isValid() || face.geometry().kind() != SurfaceKind::BSpline) return false;

    ParametricFaceMeshPolicy policy;
    policy.periodicU = false;
    policy.periodicV = false;
    policy.rejectSingularParameters = true;
    return ParametricFaceMesherCore::canMesh(face, policy);
}

FaceMesh BSplineFaceMesher::mesh(const Topology_Face& face, const BSplineFaceMeshOptions& options)
{
    if (!canMesh(face) || !options.isValid()) return FaceMesh();

    const Geometry_BSplineSurface& surface = static_cast<const Geometry_BSplineSurface&>(face.geometry());

    // 一次×一次完整自然参数域Surface按节点Span直接三角化，避免C0内部节点被错误要求具有唯一一阶导数。
    if (surface.uDegree() == 1 && surface.vDegree() == 1 && isNaturalDomainRectangle(face, surface, options.geometricTolerance))
        return meshDegreeOneNaturalDomain(face, surface, options);

    // 其余B-Spline Face继续使用通用参数曲面Mesher；当前要求网格采样点处一阶导数唯一。
    ParametricFaceMeshPolicy policy;
    policy.periodicU = false;
    policy.periodicV = false;
    policy.rejectSingularParameters = true;
    return ParametricFaceMesherCore::mesh(face, options, policy);
}

}
