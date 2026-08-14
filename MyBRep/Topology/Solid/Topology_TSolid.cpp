#include "Topology_TSolid.h"

#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

Topology_TSolid::Topology_TSolid(const std::vector<Topology_Shell>& shells)
    : m_shells(shells)
{
    MYBREP_ASSERT_MESSAGE(!m_shells.empty(),
                          "Topology_TSolid requires at least one Topology_Shell.");

    for (std::size_t index = 0; index < m_shells.size(); ++index)
    {
        MYBREP_ASSERT_MESSAGE(m_shells[index].isValid(),
                              "Topology_TSolid shells must be valid.");
        MYBREP_ASSERT_MESSAGE(m_shells[index].isClosed(),
                              "Topology_TSolid requires every shell to be closed.");

        for (std::size_t previous = 0; previous < index; ++previous)
        {
            MYBREP_ASSERT_MESSAGE(!m_shells[index].isSame(m_shells[previous]),
                                  "Topology_TSolid must not contain the same Topology_TShell identity more than once.");
        }
    }
}

/// Shell集合

std::size_t Topology_TSolid::shellCount() const
{
    return m_shells.size();
}

const Topology_Shell& Topology_TSolid::shell(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(index < m_shells.size(),
                          "Topology_TSolid shell index is out of range.");

    return m_shells[index];
}

const std::vector<Topology_Shell>& Topology_TSolid::shells() const
{
    return m_shells;
}

}
