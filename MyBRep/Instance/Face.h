#ifndef MYBREP_INSTANCE_FACE_H
#define MYBREP_INSTANCE_FACE_H

#include <cstddef>
#include <vector>

#include "MyBRep/Geometry/Surface/Geometry_Surface.h"
#include "MyBRep/Instance/Instance.h"
#include "MyBRep/Instance/Wire.h"
#include "MyBRep/Topology/Face/Topology_Face.h"

namespace MyBRep
{

class Face : public Instance
{
public:
    Face();
    explicit Face(const Topology_Face& topology);
    Face(const Topology_Face& topology, const MyMath::Matrix4& localToWorld);

    Face(const Face&) = default;
    Face& operator=(const Face&) = default;

    /// Topology与Geometry

    Topology_Face topology() const;

    bool sharesGeometryWith(const Face& other) const;

    const Geometry_Surface& geometry() const;
    const Geometry_Surface* geometryPointer() const;
    SurfaceKind kind() const;

    /// 裁剪Wire

    std::size_t wireCount() const;
    Topology_Wire topologyWire(std::size_t index) const;
    std::vector<Topology_Wire> topologyWires() const;
    Wire wire(std::size_t index) const;

    /// 方向操作

    Face reversed() const;
};

}

#endif // MYBREP_INSTANCE_FACE_H