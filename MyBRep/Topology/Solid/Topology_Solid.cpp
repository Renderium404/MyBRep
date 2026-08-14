#include "Topology_Solid.h"

#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

namespace
{

// 将单个Shell构造参数转换为Topology_TSolid统一使用的Shell集合。
std::vector<Topology_Shell> singleShellVector(const Topology_Shell& shell)
{
    std::vector<Topology_Shell> result;
    result.push_back(shell);
    return result;
}

}

Topology_Solid::Topology_Solid()
{
}

Topology_Solid::Topology_Solid(const std::vector<Topology_Shell>& shells)
    : Topology_Object(Foundation::RefPtr<Topology_TObject>(
                          new Topology_TSolid(shells)),
                      Topology_Orientation::Forward)
{
}

Topology_Solid::Topology_Solid(const Topology_Shell& shell)
    : Topology_Object(Foundation::RefPtr<Topology_TObject>(
                          new Topology_TSolid(singleShellVector(shell))),
                      Topology_Orientation::Forward)
{
}

Topology_Solid::Topology_Solid(const Foundation::RefPtr<Topology_TObject>& object,
                               Topology_Orientation orientation)
    : Topology_Object(object, orientation)
{
}

/// 有向Shell集合

std::size_t Topology_Solid::shellCount() const
{
    return isValid() ? tSolid().shellCount() : 0;
}

Topology_Shell Topology_Solid::shell(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access a shell of an invalid Topology_Solid.");
    MYBREP_ASSERT_MESSAGE(index < shellCount(),
                          "Topology_Solid shell index is out of range.");

    const Topology_Shell& storedShell = tSolid().shell(index);
    return isForward() ? storedShell : storedShell.reversed();
}

std::vector<Topology_Shell> Topology_Solid::shells() const
{
    std::vector<Topology_Shell> result;

    if (!isValid())
    {
        return result;
    }

    result.reserve(shellCount());

    for (std::size_t index = 0; index < shellCount(); ++index)
    {
        result.push_back(shell(index));
    }

    return result;
}

/// 方向操作

Topology_Solid Topology_Solid::reversed() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot reverse an invalid Topology_Solid.");

    return Topology_Solid(tObject(), reversedOrientation());
}

/// 内部访问

const Topology_TSolid& Topology_Solid::tSolid() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access an invalid Topology_Solid.");

    return *static_cast<const Topology_TSolid*>(tObject().get());
}

}
