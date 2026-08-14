#ifndef MYBREP_GEOMETRY_CURVE2D_GEOMETRY_CURVE2D_H
#define MYBREP_GEOMETRY_CURVE2D_GEOMETRY_CURVE2D_H

#include "MyMath/Vector2.h"
#include "MyBRep/Foundation/Diagnostic.h"
#include "MyBRep/Geometry/Geometry_Object.h"
#include "MyBRep/Geometry/Curve/CurveKind.h"

namespace MyBRep
{

// 定义二维参数空间中的完整参数曲线，具体曲线负责规定自然参数域和参数化方式。
//
// Geometry_Curve2D与Geometry_Curve处于相同几何层级，只是值域分别为二维参数空间和三维几何空间。
class Geometry_Curve2D : public Geometry_Object
{
public:
    /// 曲线类型

    // 返回当前二维曲线的标准几何类型。
    virtual CurveKind kind() const = 0;

    /// 定义域

    // 判断曲线自然参数域是否同时具有有限左右边界。
    virtual bool isDomainBounded() const = 0;
    // 返回自然参数域左边界，向负无穷延伸时返回负无穷。
    virtual double domainStart() const = 0;
    // 返回自然参数域右边界，向正无穷延伸时返回正无穷。
    virtual double domainEnd() const = 0;

    // 判断指定参数是否位于当前曲线自然参数域内，parameter由调用者保证为合法数值。
    bool isParameterInDomain(double parameter) const
    {
        return parameter >= domainStart() && parameter <= domainEnd();
    }

    /// 周期性

    // 判断当前二维曲线是否为周期参数曲线。
    bool isPeriodic() const
    {
        return period() > 0.0;
    }

    // 返回曲线参数周期，非周期曲线返回0。
    virtual double period() const = 0;

    /// 参数查询

    // 返回指定参数对应的二维曲线点，parameter必须位于曲线自然参数域内。
    virtual MyMath::Vector2 pointAt(double parameter) const = 0;
    // 返回指定参数处相对于曲线参数的一阶导数。
    virtual MyMath::Vector2 firstDerivativeAt(double parameter) const = 0;
    // 返回指定参数处相对于曲线参数的二阶导数。
    virtual MyMath::Vector2 secondDerivativeAt(double parameter) const = 0;

    // 返回指定正则参数处沿参数增大方向的二维单位切向量。
    MyMath::Vector2 tangentAt(double parameter) const
    {
        MYBREP_ASSERT_MESSAGE(isParameterInDomain(parameter),
                              "Geometry_Curve2D tangent parameter is outside the natural parameter domain.");

        const MyMath::Vector2 derivative = firstDerivativeAt(parameter);

        MYBREP_ASSERT_MESSAGE(derivative.isVector(0.0),
                              "Geometry_Curve2D tangent requires a finite non-zero first derivative.");

        return derivative.normalized(0.0);
    }

protected:
    // 通过Geometry_Object侵入式引用计数管理具体二维曲线几何生命周期。
    ~Geometry_Curve2D() override = default;
};

}

#endif // MYBREP_GEOMETRY_CURVE2D_GEOMETRY_CURVE2D_H
