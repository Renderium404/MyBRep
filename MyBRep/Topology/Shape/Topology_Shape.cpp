#include "Topology_Shape.h"

#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

Topology_Shape::Topology_Shape()
{
}

Topology_Shape::Topology_Shape(const Foundation::RefPtr<const Geometry_Shape>& geometry)
    : Topology_Object(Foundation::RefPtr<Topology_TObject>(
                          new Topology_TShape(geometry)),
                      Topology_Orientation::Forward)
{
}

Topology_Shape::Topology_Shape(const Foundation::RefPtr<Topology_TObject>& object,
                               Topology_Orientation orientation)
    : Topology_Object(object, orientation)
{
}

/// 几何内核

const Geometry_Shape& Topology_Shape::geometry() const
{
    return tShape().geometry();
}

const Foundation::RefPtr<const Geometry_Shape>& Topology_Shape::geometryResource() const
{
    return tShape().geometryResource();
}

/// 方向操作

Topology_Shape Topology_Shape::reversed() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot reverse an invalid Topology_Shape.");

    return Topology_Shape(tObject(), reversedOrientation());
}

/// 内部访问

const Topology_TShape& Topology_Shape::tShape() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access an invalid Topology_Shape.");

    return *static_cast<const Topology_TShape*>(tObject().get());
}

}