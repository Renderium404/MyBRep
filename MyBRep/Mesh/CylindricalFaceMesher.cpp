#include "CylindricalFaceMesher.h"

#include <algorithm>
#include <cmath>

#include "MyBRep/Geometry/Surface/Geometry_CylindricalSurface.h"
#include "MyBRep/Geometry/Surface/SurfaceKind.h"

namespace
{

const double Pi = 3.1415926535897932384626433832795;
const double MaximumSurfaceAngularStep = Pi * 0.5;

// 将90°最大参数边跨度换算为圆柱面中点弦高上限。
double maximumSurfaceChordTolerance(const MyBRep::Geometry_CylindricalSurface& cylinder)
{
    return cylinder.radius() * (1.0 - std::cos(MaximumSurfaceAngularStep * 0.5));
}

}

namespace MyBRep
{

CylindricalFaceMeshOptions::CylindricalFaceMeshOptions()
    : ParametricFaceMeshOptions()
{
}

bool CylindricalFaceMesher::canMesh(const Topology_Face& face)
{
    if (!face.isValid() || face.geometry().kind() != SurfaceKind::Cylindrical)
    {
        return false;
    }

    const Geometry_CylindricalSurface& cylinder = static_cast<const Geometry_CylindricalSurface&>(face.geometry());

    if (!cylinder.isUPeriodic() || cylinder.uPeriod() <= 0.0 || cylinder.isVPeriodic())
    {
        return false;
    }

    ParametricFaceMeshPolicy policy;
    policy.periodicU = true;
    policy.periodicV = false;
    policy.rejectSingularParameters = true;

    return ParametricFaceMesherCore::canMesh(face, policy);
}

FaceMesh CylindricalFaceMesher::mesh(const Topology_Face& face, const CylindricalFaceMeshOptions& options)
{
    if (!canMesh(face) || !options.isValid())
    {
        return FaceMesh();
    }

    const Geometry_CylindricalSurface& cylinder = static_cast<const Geometry_CylindricalSurface&>(face.geometry());

    CylindricalFaceMeshOptions coreOptions = options;
    coreOptions.surfaceChordTolerance = (std::min)(coreOptions.surfaceChordTolerance, maximumSurfaceChordTolerance(cylinder));

    ParametricFaceMeshPolicy policy;
    policy.periodicU = true;
    policy.periodicV = false;
    policy.rejectSingularParameters = true;

    return ParametricFaceMesherCore::mesh(face, coreOptions, policy);
}

}
