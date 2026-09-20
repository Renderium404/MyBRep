#ifndef MYBREP_INSTANCE_INSTANCE_OBJECT_H
#define MYBREP_INSTANCE_INSTANCE_OBJECT_H

#include "MyMath/Matrix4.h"
#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

/// 实例身份

// MyBRep空间实例唯一标识类型。
typedef unsigned long long InstanceId;

// 无效实例ID。
const InstanceId InvalidInstanceId = 0;

namespace InstanceObjectDetail
{

// 分配一个进程内唯一且非零的InstanceId。
InstanceId allocateInstanceId();

}

/// Instance_Object

// 表示一个Topology对象在世界空间中的一次独立实例。
// 每个Instance_Object拥有独立InstanceId；Topology句柄可以在多个实例之间共享。
// Instance_Object只负责实例身份、共享Topology句柄和空间放置，不代理具体Topology类型的业务接口。
template<typename TopologyType>
class Instance_Object
{
public:
    // 构造不引用任何Topology的空实例；仍然分配独立InstanceId，空间放置为单位变换。
    Instance_Object()
        : m_id(InstanceObjectDetail::allocateInstanceId())
        , m_localToWorld(MyMath::Matrix4::identity())
        , m_worldToLocal(MyMath::Matrix4::identity())
    {
    }

    // 使用单位变换放置指定有效Topology，并分配新的InstanceId。
    explicit Instance_Object(const TopologyType& topology)
        : m_id(InstanceObjectDetail::allocateInstanceId())
        , m_topology(topology)
        , m_localToWorld(MyMath::Matrix4::identity())
        , m_worldToLocal(MyMath::Matrix4::identity())
    {
        MYBREP_ASSERT_MESSAGE(topology.isValid(), "Instance_Object topology must be valid.");
    }

    // 使用指定可逆仿射变换放置指定有效Topology，并分配新的InstanceId。
    Instance_Object(const TopologyType& topology, const MyMath::Matrix4& localToWorld)
        : m_id(InstanceObjectDetail::allocateInstanceId())
        , m_topology(topology)
        , m_localToWorld(MyMath::Matrix4::identity())
        , m_worldToLocal(MyMath::Matrix4::identity())
    {
        MYBREP_ASSERT_MESSAGE(topology.isValid(), "Instance_Object topology must be valid.");
        const bool validTransform = setLocalToWorld(localToWorld);
        MYBREP_ASSERT_MESSAGE(validTransform, "Instance_Object transform must be invertible and affine.");
    }

    // 复制为新的独立实例：分配新InstanceId，但共享同一Topology身份并复制空间放置。
    Instance_Object(const Instance_Object& other)
        : m_id(InstanceObjectDetail::allocateInstanceId())
        , m_topology(other.m_topology)
        , m_localToWorld(other.m_localToWorld)
        , m_worldToLocal(other.m_worldToLocal)
    {
    }

    // 将源实例内容赋给当前实例；当前实例自身InstanceId保持不变。
    Instance_Object& operator=(const Instance_Object& other)
    {
        if (this == &other) return *this;

        m_topology = other.m_topology;
        m_localToWorld = other.m_localToWorld;
        m_worldToLocal = other.m_worldToLocal;
        return *this;
    }

    /// 实例身份

    // 返回当前实例稳定且非零的唯一ID。
    InstanceId id() const{return m_id;}

    // 判断两个对象是否表示同一个实例身份。
    bool isSameInstance(const Instance_Object& other) const{return m_id == other.m_id;}

    /// 状态判断

    // 判断当前实例是否引用有效Topology。
    bool isValid() const{return m_topology.isValid();}
    // 判断当前实例是否未引用Topology。
    bool isNull() const{return m_topology.isNull();}
    // 判断当前实例是否引用有效Topology。
    explicit operator bool() const{return isValid();}

    // 判断两个实例是否共享同一个底层Topology身份，InstanceId和空间放置不参与判断。
    bool sharesTopologyWith(const Instance_Object& other) const
    {
        return isValid() && other.isValid() && m_topology.isSame(other.m_topology);
    }

    /// Topology

    // 返回当前实例引用的Topology。
    const TopologyType& topology() const
    {
        MYBREP_ASSERT_MESSAGE(isValid(), "Cannot access the topology of an invalid Instance_Object.");
        return m_topology;
    }

    /// 空间放置

    // 返回当前实例从局部空间到世界空间的可逆仿射变换。
    const MyMath::Matrix4& localToWorld() const{return m_localToWorld;}
    // 返回当前实例从世界空间到局部空间的逆变换。
    const MyMath::Matrix4& worldToLocal() const{return m_worldToLocal;}

    // 修改当前实例空间放置；输入不是可逆仿射矩阵时保持原状态并返回false。
    // 修改空间放置不会改变InstanceId或Topology身份。
    bool setLocalToWorld(const MyMath::Matrix4& localToWorld)
    {
        if (!localToWorld.isAffine()) return false;

        MyMath::Matrix4 worldToLocal;
        if (!localToWorld.inverted(worldToLocal)) return false;

        m_localToWorld = localToWorld;
        m_worldToLocal = worldToLocal;
        return true;
    }

private:
    InstanceId m_id;                     // 当前实例自身稳定身份。
    TopologyType m_topology;              // 当前实例共享的Topology句柄。
    MyMath::Matrix4 m_localToWorld;       // 当前实例从局部空间到世界空间的放置。
    MyMath::Matrix4 m_worldToLocal;       // 当前实例从世界空间到局部空间的逆放置。
};
using Instance = Instance_Object<Topology_Object&>;
using Edge = Instance;
using Wire = Instance;
using Face = Instance;
using Shell = Instance;
using Solid = Instance;
}

#endif // MYBREP_INSTANCE_INSTANCE_OBJECT_H
