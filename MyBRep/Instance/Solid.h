
#ifndef MYBREP_INSTANCE_SOLID_H
#define MYBREP_INSTANCE_SOLID_H

#include <cstddef>
#include <vector>

#include "MyBRep/Instance/Instance.h"
#include "MyBRep/Instance/Shell.h"
#include "MyBRep/Topology/Solid/Topology_Solid.h"

namespace MyBRep
{

class Solid : public Instance
{
public:
    Solid();
    explicit Solid(const Topology_Solid& topology);
    Solid(const Topology_Solid& topology, const MyMath::Matrix4& localToWorld);

    Solid(const Solid&) = default;
    Solid& operator=(const Solid&) = default;

    /// Topology

    Topology_Solid topology() const;

    std::size_t shellCount() const;
    Topology_Shell topologyShell(std::size_t index) const;
    std::vector<Topology_Shell> topologyShells() const;

    /// 子实例

    Shell shell(std::size_t index) const;

    /// 方向操作

    Solid reversed() const;
};

}

#endif // MYBREP_INSTANCE_SOLID_H