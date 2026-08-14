#include "Topology_Orientation.h"

namespace MyBRep
{

Topology_Orientation oppositeOrientation(Topology_Orientation orientation)
{
    return orientation == Topology_Orientation::Forward
               ? Topology_Orientation::Reversed
               : Topology_Orientation::Forward;
}

}
