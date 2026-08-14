#include "Instance_Object.h"

#include "MyBRep/Foundation/Diagnostic.h"

namespace MyBRep
{

Instance_Object::Instance_Object()
    : m_localToWorld(MyMath::Matrix4::identity())
    , m_worldToLocal(MyMath::Matrix4::identity())
{
}

Instance_Object::Instance_Object(const MyMath::Matrix4& localToWorld)
    : m_localToWorld(localToWorld)
    , m_worldToLocal(MyMath::Matrix4::identity())
{
    MYBREP_ASSERT_MESSAGE(localToWorld.isAffine(),
                          "Instance_Object transform must be affine.");

    const bool invertible = localToWorld.inverted(m_worldToLocal);

    MYBREP_ASSERT_MESSAGE(invertible,
                          "Instance_Object transform must be invertible.");
}

Instance_Object::~Instance_Object()
{
}

/// 空间放置

const MyMath::Matrix4& Instance_Object::localToWorld() const
{
    return m_localToWorld;
}

const MyMath::Matrix4& Instance_Object::worldToLocal() const
{
    return m_worldToLocal;
}

}
