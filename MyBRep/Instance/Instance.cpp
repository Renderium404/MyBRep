#include "Instance.h"

#include "MyBRep/Foundation/Diagnostic.h"

#include <atomic>

namespace MyBRep
{

namespace InstanceDetail
{

InstanceId allocateInstanceId()
{
    static std::atomic<InstanceId> nextId(1);

    InstanceId id = nextId.fetch_add(1);

    while (id == InvalidInstanceId)
        id = nextId.fetch_add(1);

    return id;
}

}

/// Instance

Instance::Instance()
    : m_id(InstanceDetail::allocateInstanceId())
    , m_localToWorld(MyMath::Matrix4::identity())
    , m_worldToLocal(MyMath::Matrix4::identity())
{
}

Instance::Instance(const Topology_Object& topology)
    : m_id(InstanceDetail::allocateInstanceId())
    , m_topology(topology)
    , m_localToWorld(MyMath::Matrix4::identity())
    , m_worldToLocal(MyMath::Matrix4::identity())
{
    MYBREP_ASSERT_MESSAGE(topology.isValid(), "Instance topology must be valid.");
}

Instance::Instance(const Topology_Object& topology, const MyMath::Matrix4& localToWorld)
    : m_id(InstanceDetail::allocateInstanceId())
    , m_topology(topology)
    , m_localToWorld(MyMath::Matrix4::identity())
    , m_worldToLocal(MyMath::Matrix4::identity())
{
    MYBREP_ASSERT_MESSAGE(topology.isValid(), "Instance topology must be valid.");

    const bool validTransform = setLocalToWorld(localToWorld);
    MYBREP_ASSERT_MESSAGE(validTransform, "Instance transform must be invertible and affine.");
}

Instance::Instance(const Instance& other)
    : m_id(InstanceDetail::allocateInstanceId())
    , m_topology(other.m_topology)
    , m_localToWorld(other.m_localToWorld)
    , m_worldToLocal(other.m_worldToLocal)
{
}

Instance& Instance::operator=(const Instance& other)
{
    if (this == &other) return *this;

    m_topology = other.m_topology;
    m_localToWorld = other.m_localToWorld;
    m_worldToLocal = other.m_worldToLocal;

    onInstanceChanged();
    return *this;
}

Instance::~Instance()
{
}

/// 空间放置

bool Instance::setLocalToWorld(const MyMath::Matrix4& localToWorld)
{
    if (!localToWorld.isAffine()) return false;

    MyMath::Matrix4 worldToLocal;
    if (!localToWorld.inverted(worldToLocal)) return false;

    m_localToWorld = localToWorld;
    m_worldToLocal = worldToLocal;

    onInstanceChanged();
    return true;
}

void Instance::onInstanceChanged()
{
}

}