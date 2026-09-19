#include "Instance_Object.h"

#include <atomic>

namespace MyBRep
{
namespace InstanceObjectDetail
{

static std::atomic<InstanceId> g_nextInstanceId(1);

// 分配一个进程内唯一且非零的InstanceId。
InstanceId allocateInstanceId()
{
    const InstanceId id = g_nextInstanceId.fetch_add(1, std::memory_order_relaxed);
    MYBREP_ASSERT_MESSAGE(id != InvalidInstanceId, "InstanceId range exhausted.");
    return id;
}

}
}