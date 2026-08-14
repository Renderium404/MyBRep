#include "Topology_Face.h"

#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

Topology_Face::Topology_Face()
{
}

Topology_Face::Topology_Face(const Foundation::RefPtr<const Geometry_Surface>& geometry,
                             const std::vector<Topology_Wire>& wires)
    : Topology_Object(Foundation::RefPtr<Topology_TObject>(
                          new Topology_TFace(geometry, wires)),
                      Topology_Orientation::Forward)
{
}

Topology_Face::Topology_Face(const Foundation::RefPtr<const Geometry_Surface>& geometry)
    : Topology_Object(Foundation::RefPtr<Topology_TObject>(
                          new Topology_TFace(geometry, std::vector<Topology_Wire>())),
                      Topology_Orientation::Forward)
{
}

Topology_Face::Topology_Face(const Foundation::RefPtr<Topology_TObject>& object,
                             Topology_Orientation orientation)
    : Topology_Object(object, orientation)
{
}

/// 曲面几何

const Geometry_Surface& Topology_Face::geometry() const
{
    return tFace().geometry();
}

const Foundation::RefPtr<const Geometry_Surface>& Topology_Face::geometryResource() const
{
    return tFace().geometryResource();
}

MyMath::Vector3 Topology_Face::normalAt(double u, double v) const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot query an invalid Topology_Face.");

    const MyMath::Vector3 normal = geometry().normalAt(u, v);
    return isForward() ? normal : normal * -1.0;
}

/// 裁剪Wire

std::size_t Topology_Face::wireCount() const
{
    return isValid() ? tFace().wireCount() : 0;
}

Topology_Wire Topology_Face::wire(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access a wire of an invalid Topology_Face.");
    MYBREP_ASSERT_MESSAGE(index < wireCount(),
                          "Topology_Face wire index is out of range.");

    const Topology_Wire& storedWire = tFace().wire(index);
    return isForward() ? storedWire : storedWire.reversed();
}

std::vector<Topology_Wire> Topology_Face::wires() const
{
    std::vector<Topology_Wire> result;

    if (!isValid())
    {
        return result;
    }

    result.reserve(wireCount());

    for (std::size_t index = 0; index < wireCount(); ++index)
    {
        result.push_back(wire(index));
    }

    return result;
}

/// 方向操作

Topology_Face Topology_Face::reversed() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot reverse an invalid Topology_Face.");

    return Topology_Face(tObject(), reversedOrientation());
}

/// 内部访问

const Topology_TFace& Topology_Face::tFace() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access an invalid Topology_Face.");

    return *static_cast<const Topology_TFace*>(tObject().get());
}

}
