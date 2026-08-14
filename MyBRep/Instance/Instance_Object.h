#ifndef MYBREP_INSTANCE_INSTANCE_OBJECT_H
#define MYBREP_INSTANCE_INSTANCE_OBJECT_H

#include "MyMath/Matrix4.h"

namespace MyBRep
{

// 作为全部空间实例对象的放置基类，只负责局部空间与世界空间之间的可逆仿射变换。
//
// Instance_Object不规定实例必须引用Geometry或Topology，也不承担具体对象的有效性、包围盒或显示状态。
// 成功构造的Instance_Object始终具有有效放置；输入变换必须由调用者保证为可逆仿射矩阵。
class Instance_Object
{
public:
    /// 空间放置

    // 返回当前实例从局部空间到世界空间的可逆仿射变换。
    const MyMath::Matrix4& localToWorld() const;
    // 返回当前实例从世界空间到局部空间的逆放置变换。
    const MyMath::Matrix4& worldToLocal() const;

protected:
    // 使用单位矩阵构造空间放置。
    Instance_Object();
    // 使用指定可逆仿射矩阵构造空间放置。
    explicit Instance_Object(const MyMath::Matrix4& localToWorld);
    // 通过具体实例类型销毁空间放置基类。
    ~Instance_Object();

private:
    MyMath::Matrix4 m_localToWorld; // 当前实例从局部空间到世界空间的放置变换。
    MyMath::Matrix4 m_worldToLocal; // 当前实例从世界空间到局部空间的逆放置变换。
};

}

#endif // MYBREP_INSTANCE_INSTANCE_OBJECT_H
