#include "Topology_Shell.h"

#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

Topology_Shell::Topology_Shell()
{
}

Topology_Shell::Topology_Shell(const std::vector<Topology_Face>& faces)
    : Topology_Object(Foundation::RefPtr<Topology_TObject>(
                          new Topology_TShell(faces)),
                      Topology_Orientation::Forward)
{
}

Topology_Shell::Topology_Shell(const Foundation::RefPtr<Topology_TObject>& object,
                               Topology_Orientation orientation)
    : Topology_Object(object, orientation)
{
}

/// 拓扑状态

bool Topology_Shell::isClosed() const
{
    return isValid() && tShell().isClosed();
}

/// 有向Face集合

std::size_t Topology_Shell::faceCount() const
{
    return isValid() ? tShell().faceCount() : 0;
}

Topology_Face Topology_Shell::face(std::size_t index) const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access a face of an invalid Topology_Shell.");
    MYBREP_ASSERT_MESSAGE(index < faceCount(),
                          "Topology_Shell face index is out of range.");

    const Topology_Face& storedFace = tShell().face(index);
    return isForward() ? storedFace : storedFace.reversed();
}

std::vector<Topology_Face> Topology_Shell::faces() const
{
    std::vector<Topology_Face> result;

    if (!isValid())
    {
        return result;
    }

    result.reserve(faceCount());

    for (std::size_t index = 0; index < faceCount(); ++index)
    {
        result.push_back(face(index));
    }

    return result;
}

/// 方向操作

Topology_Shell Topology_Shell::reversed() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot reverse an invalid Topology_Shell.");

    return Topology_Shell(tObject(), reversedOrientation());
}

/// 内部访问

const Topology_TShell& Topology_Shell::tShell() const
{
    MYBREP_ASSERT_MESSAGE(isValid(),
                          "Cannot access an invalid Topology_Shell.");

    return *static_cast<const Topology_TShell*>(tObject().get());
}

}
