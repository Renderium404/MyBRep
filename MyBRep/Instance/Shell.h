#ifndef MYBREP_INSTANCE_SHELL_H
#define MYBREP_INSTANCE_SHELL_H

#include <cstddef>
#include <vector>

#include "MyBRep/Instance/Face.h"
#include "MyBRep/Instance/Instance.h"
#include "MyBRep/Topology/Shell/Topology_Shell.h"

namespace MyBRep
{

class Shell : public Instance
{
public:
    Shell();
    explicit Shell(const Topology_Shell& topology);
    Shell(const Topology_Shell& topology, const MyMath::Matrix4& localToWorld);

    Shell(const Shell&) = default;
    Shell& operator=(const Shell&) = default;

    /// Topology

    Topology_Shell topology() const;

    bool isClosed() const;

    std::size_t faceCount() const;
    Topology_Face topologyFace(std::size_t index) const;
    std::vector<Topology_Face> topologyFaces() const;

    /// 子实例

    Face face(std::size_t index) const;

    /// 方向操作

    Shell reversed() const;
};

}

#endif // MYBREP_INSTANCE_SHELL_H
