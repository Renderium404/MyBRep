#ifndef MYBREP_INSTANCE_INSTANCE_H
#define MYBREP_INSTANCE_INSTANCE_H

#include "MyMath/Matrix4.h"
#include "MyBRep/Topology/Topology_Object.h"

namespace MyBRep
{

/// 实例身份

typedef unsigned long long InstanceId;

const InstanceId InvalidInstanceId = 0;

namespace InstanceDetail
{

// 分配进程内唯一且非零的InstanceId。
InstanceId allocateInstanceId();

}

/// Instance

// 表示任意Topology对象在世界空间中的一次独立实例。
// Instance统一负责实例身份、共享Topology句柄和空间放置。
// 具体Vertex、Edge、Wire、Face、Shell、Solid等类型只补充强类型Topology访问和实例级便利接口。
class Instance
{
public:
    // 构造不引用Topology的空实例；仍拥有独立InstanceId。
    Instance();

    // 使用单位放置创建指定Topology的实例。
    explicit Instance(const Topology_Object& topology);

    // 使用指定可逆仿射变换创建指定Topology的实例。
    Instance(const Topology_Object& topology, const MyMath::Matrix4& localToWorld);

    // 复制产生新的独立实例身份，同时共享同一Topology并复制空间放置。
    Instance(const Instance& other);

    // 赋值复制Topology和空间放置，但保持当前InstanceId。
    Instance& operator=(const Instance& other);

    virtual ~Instance();

    /// 实例身份

    InstanceId id() const{return m_id;}
    bool isSameInstance(const Instance& other) const{return m_id == other.m_id;}

    /// 状态

    bool isValid() const{return m_topology.isValid();}
    bool isNull() const{return m_topology.isNull();}
    explicit operator bool() const{return isValid();}

    // 判断两个实例是否共享同一个底层Topology身份。
    bool sharesTopologyWith(const Instance& other) const
    {
        return isValid() && other.isValid() && m_topology.isSame(other.m_topology);
    }

    /// Topology

    // 返回当前实例持有的通用Topology句柄。
    const Topology_Object& topologyObject() const{return m_topology;}

    /// 空间放置

    const MyMath::Matrix4& localToWorld() const{return m_localToWorld;}
    const MyMath::Matrix4& worldToLocal() const{return m_worldToLocal;}

    // 修改空间放置；失败时保持原状态。
    bool setLocalToWorld(const MyMath::Matrix4& localToWorld);

protected:
    // Instance内部状态改变后调用。
    // 普通实例无需处理；例如Shape可据此刷新World Bounds缓存。
    virtual void onInstanceChanged();

private:
    InstanceId m_id;
    Topology_Object m_topology;
    MyMath::Matrix4 m_localToWorld;
    MyMath::Matrix4 m_worldToLocal;
};

}

#endif // MYBREP_INSTANCE_INSTANCE_H