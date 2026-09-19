#ifndef MYBREP_INSTANCE_FACE_H
#define MYBREP_INSTANCE_FACE_H

#include "MyBRep/Instance/Instance_Object.h"
#include "MyBRep/Topology/Face/Topology_Face.h"

namespace MyBRep
{

// Topology_Face的空间实例。
using Face = Instance_Object<Topology_Face>;

}

#endif // MYBREP_INSTANCE_FACE_H
