#ifndef MYBREP_INSTANCE_WIRE_H
#define MYBREP_INSTANCE_WIRE_H

#include "MyBRep/Instance/Instance_Object.h"
#include "MyBRep/Topology/Wire/Topology_Wire.h"

namespace MyBRep
{

// Topology_Wire的空间实例。
using Wire = Instance_Object<Topology_Wire>;

}

#endif // MYBREP_INSTANCE_WIRE_H
