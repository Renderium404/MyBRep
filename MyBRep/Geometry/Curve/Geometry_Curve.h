#ifndef MYBREP_GEOMETRY_CURVE_GEOMETRY_CURVE_H
#define MYBREP_GEOMETRY_CURVE_GEOMETRY_CURVE_H

#include "MyMath/Vector3.h"
#include "MyBRep/Foundation/Diagnostic.h"
#include "MyBRep/Geometry/Geometry_Object.h"
#include "CurveKind.h"

namespace MyBRep
{

// 定义三维几何空间中的完整参数曲线，具体曲线负责规定自然参数域和参数化方式。
// 曲线统一表示为(x,y,z)=f(t)。
class Geometry_Curve : public Geometry_Object
{
public:
    /// 曲线类型

    // 返回当前曲线的标准几何类型。
    virtual CurveKind kind() const = 0;

    /// 定义域

    // 判断曲线自然参数域是否同时具有有限左右边界。
    virtual bool isDomainBounded() const = 0;
    // 返回自然参数域左边界，向负无穷延伸时返回负无穷。
    virtual double domainStart() const = 0;
    // 返回自然参数域右边界，向正无穷延伸时返回正无穷。
    virtual double domainEnd() const = 0;

    // 判断指定参数是否位于当前曲线自然参数域内。
    bool isParameterInDomain(double parameter) const
    {
        return parameter >= domainStart() && parameter <= domainEnd();
    }

    /// 周期性

    // 判断当前曲线是否为周期参数曲线。
    bool isPeriodic() const
    {
        return period() > 0.0;
    }

    // 返回曲线参数周期，非周期曲线返回0。
    virtual double period() const = 0;

    /// 参数查询

    // 返回指定参数对应的三维曲线点，parameter必须位于曲线自然参数域内。
    virtual MyMath::Vector3 pointAt(double parameter) const = 0;
    // 返回指定参数处相对于曲线参数的一阶导数。
    virtual MyMath::Vector3 firstDerivativeAt(double parameter) const = 0;
    // 返回指定参数处相对于曲线参数的二阶导数。
    virtual MyMath::Vector3 secondDerivativeAt(double parameter) const = 0;

    // 返回指定正则参数处沿参数增大方向的单位切向量，一阶导数必须为有限非零向量。
    MyMath::Vector3 tangentAt(double parameter) const
    {
        MYBREP_ASSERT_MESSAGE(isParameterInDomain(parameter),"Geometry_Curve tangent parameter is outside the natural parameter domain.");
        const MyMath::Vector3 derivative = firstDerivativeAt(parameter);
        return derivative.normalized(0.0);
    }

protected:
    // 通过Geometry_Object侵入式引用计数管理具体曲线几何生命周期。
    ~Geometry_Curve() override = default;
};

}

#endif // MYBREP_GEOMETRY_CURVE_GEOMETRY_CURVE_H
