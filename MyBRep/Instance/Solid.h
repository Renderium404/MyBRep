#ifndef MYBREP_INSTANCE_SOLID_H
#define MYBREP_INSTANCE_SOLID_H

#include "MyBRep/Instance/Instance_Object.h"
#include "MyBRep/Topology/Solid/Topology_Solid.h"

namespace MyBRep
{

// Topology_Solid的空间实例。
using Solid = Instance_Object<Topology_Solid>;

}

#endif // MYBREP_INSTANCE_SOLID_H
