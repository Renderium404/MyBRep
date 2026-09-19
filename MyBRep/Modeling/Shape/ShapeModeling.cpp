#include "ShapeModeling.h"

#include <vector>

#include "MyMath/MathUtils.h"
#include "MyBRep/Foundation/Diagnostic.h"
#include "MyBRep/Geometry/Construction/Geometry_Extruded.h"
#include "MyBRep/Geometry/Shape/Geometry_Box.h"
#include "MyBRep/Geometry/Shape/Geometry_ConeFrustum.h"
#include "MyBRep/Geometry/Shape/Geometry_Cylinder.h"
#include "MyBRep/Geometry/Shape/Geometry_Sphere.h"
#include "MyBRep/Topology/Edge/Topology_Edge.h"

namespace MyBRep
{
namespace Modeling
{

/// 通用Topology_Shape创建

Topology_Shape createShape(const Foundation::RefPtr<const Geometry_Shape>& geometry)
{
    MYBREP_ASSERT_MESSAGE(geometry,"Shape modeling requires a non-null Geometry_Shape.");

    return Topology_Shape(geometry);
}

/// 标准解析Topology_Shape创建

Topology_Shape createBox(double sizeX,double sizeY,double sizeZ)
{
    const Foundation::RefPtr<const Geometry_Shape> geometry(new Geometry_Box(sizeX, sizeY, sizeZ));

    return createShape(geometry);
}

Topology_Shape createSphere(double radius)
{
    const Foundation::RefPtr<const Geometry_Shape> geometry(new Geometry_Sphere(radius));

    return createShape(geometry);
}

Topology_Shape createCylinder(double radius,double height)
{
    const Foundation::RefPtr<const Geometry_Shape> geometry(new Geometry_Cylinder(radius, height));

    return createShape(geometry);
}

Topology_Shape createCone(double bottomRadius,double height)
{
    const Foundation::RefPtr<const Geometry_Shape> geometry(new Geometry_ConeFrustum(bottomRadius,0.0,height));

    return createShape(geometry);
}

Topology_Shape createConeFrustum(double bottomRadius,double topRadius,double height)
{
    const Foundation::RefPtr<const Geometry_Shape> geometry(new Geometry_ConeFrustum(bottomRadius,topRadius,height));

    return createShape(geometry);
}

/// 构造连续体Topology_Shape创建

Topology_Shape createExtruded(const Topology_Wire& profile,double height,double profileTolerance)
{
    MYBREP_ASSERT_MESSAGE(profile.isValid() &&profile.isClosed(),"Extruded modeling requires a valid closed Topology_Wire.");
    MYBREP_ASSERT_MESSAGE(MyMath::isFinite(profileTolerance) && profileTolerance >= 0.0,"Extruded modeling profile tolerance must be finite and non-negative.");

    std::vector<Geometry_Extruded::ProfileSegment> segments;
    segments.reserve(profile.edgeCount());

    for (std::size_t index = 0; index < profile.edgeCount(); ++index)
    {
        const Topology_Edge edge = profile.edge(index);
        segments.push_back(Geometry_Extruded::ProfileSegment(edge.geometryResource(), edge.firstParameter(), edge.lastParameter()));
    }

    const Foundation::RefPtr<const Geometry_Shape> geometry(new Geometry_Extruded(segments,height,profileTolerance));

    return createShape(geometry);
}

/// 空间Shape实例创建

Shape makeShape(const Foundation::RefPtr<const Geometry_Shape>& geometry)
{
    return Shape(createShape(geometry));
}

Shape makeShape(const Foundation::RefPtr<const Geometry_Shape>& geometry,const MyMath::Matrix4& localToWorld)
{
    return Shape(createShape(geometry),localToWorld);
}

Shape makeBox(double sizeX,double sizeY,double sizeZ)
{
    return Shape(createBox(sizeX,sizeY,sizeZ));
}

Shape makeBox(double sizeX,double sizeY,double sizeZ,const MyMath::Matrix4& localToWorld)
{
    return Shape(createBox(sizeX,sizeY,sizeZ),localToWorld);
}

Shape makeSphere(double radius)
{
    return Shape(createSphere(radius));
}

Shape makeSphere(double radius,const MyMath::Matrix4& localToWorld)
{
    return Shape(createSphere(radius),localToWorld);
}

Shape makeCylinder(double radius,double height)
{
    return Shape(createCylinder(radius,height));
}

Shape makeCylinder(double radius,double height,const MyMath::Matrix4& localToWorld)
{
    return Shape(createCylinder(radius,height),localToWorld);
}

Shape makeCone(double bottomRadius, double height)
{
    return Shape(createCone(bottomRadius,height));
}

Shape makeCone(double bottomRadius,double height,const MyMath::Matrix4& localToWorld)
{
    return Shape(createCone(bottomRadius,height),localToWorld);
}

Shape makeConeFrustum(double bottomRadius,double topRadius,double height)
{
    return Shape(createConeFrustum(bottomRadius,topRadius,height));
}

Shape makeConeFrustum(double bottomRadius,double topRadius,double height,const MyMath::Matrix4& localToWorld)
{
    return Shape(createConeFrustum(bottomRadius,topRadius,height),localToWorld);
}

Shape makeExtruded(const Topology_Wire& profile,double height,double profileTolerance)
{
    return Shape(createExtruded(profile,height,profileTolerance));
}

Shape makeExtruded(
    const Topology_Wire& profile,
    double height,
    const MyMath::Matrix4& localToWorld,
    double profileTolerance)
{
    return Shape(createExtruded(profile,
                                height,
                                profileTolerance),
                 localToWorld);
}

Shape makeExtruded(
    const Wire& profile,
    double height,
    double profileTolerance)
{
    MYBREP_ASSERT_MESSAGE(profile.isValid(),"Extruded modeling requires a valid Wire instance.");

    return Shape(createExtruded(profile.topology(),
                                height,
                                profileTolerance),
                 profile.localToWorld());
}

}
}