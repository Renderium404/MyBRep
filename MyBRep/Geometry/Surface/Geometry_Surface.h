#ifndef MYBREP_GEOMETRY_SURFACE_GEOMETRY_SURFACE_H
#define MYBREP_GEOMETRY_SURFACE_GEOMETRY_SURFACE_H

#include "MyMath/Vector3.h"
#include "MyBRep/Foundation/Diagnostic.h"
#include "MyBRep/Geometry/Geometry_Object.h"
#include "SurfaceKind.h"

namespace MyBRep
{

// 定义三维几何空间中的完整参数曲面，具体曲面负责规定U/V自然参数域和参数化方式。
// 曲面统一表示为(x,y,z)=f(u,v)。
class Geometry_Surface : public Geometry_Object
{
public:
    /// 曲面类型

    // 返回当前曲面的标准几何类型。
    virtual SurfaceKind kind() const = 0;

    /// U定义域

    // 判断U方向自然参数域是否同时具有有限左右边界。
    virtual bool isUDomainBounded() const = 0;
    // 返回U方向自然参数域左边界，向负无穷延伸时返回负无穷。
    virtual double uDomainStart() const = 0;
    // 返回U方向自然参数域右边界，向正无穷延伸时返回正无穷。
    virtual double uDomainEnd() const = 0;

    // 判断指定U参数是否位于当前曲面的U方向自然参数域内。
    bool isUParameterInDomain(double u) const
    {
        return u >= uDomainStart() && u <= uDomainEnd();
    }

    /// V定义域

    // 判断V方向自然参数域是否同时具有有限左右边界。
    virtual bool isVDomainBounded() const = 0;
    // 返回V方向自然参数域左边界，向负无穷延伸时返回负无穷。
    virtual double vDomainStart() const = 0;
    // 返回V方向自然参数域右边界，向正无穷延伸时返回正无穷。
    virtual double vDomainEnd() const = 0;

    // 判断指定V参数是否位于当前曲面的V方向自然参数域内。
    bool isVParameterInDomain(double v) const
    {
        return v >= vDomainStart() && v <= vDomainEnd();
    }

    // 判断指定参数对是否位于当前曲面的自然参数域内。
    bool isParameterInDomain(double u, double v) const
    {
        return isUParameterInDomain(u) && isVParameterInDomain(v);
    }

    /// U周期性

    // 判断U方向是否为周期参数方向。
    bool isUPeriodic() const
    {
        return uPeriod() > 0.0;
    }

    // 返回U方向参数周期，非周期方向返回0。
    virtual double uPeriod() const = 0;

    /// V周期性

    // 判断V方向是否为周期参数方向。
    bool isVPeriodic() const
    {
        return vPeriod() > 0.0;
    }

    // 返回V方向参数周期，非周期方向返回0。
    virtual double vPeriod() const = 0;

    /// 参数查询

    // 返回指定参数对对应的三维曲面点。
    virtual MyMath::Vector3 pointAt(double u, double v) const = 0;

    /// 一阶偏导

    // 返回指定参数处相对于U参数的一阶偏导。
    virtual MyMath::Vector3 firstDerivativeUAt(double u, double v) const = 0;
    // 返回指定参数处相对于V参数的一阶偏导。
    virtual MyMath::Vector3 firstDerivativeVAt(double u, double v) const = 0;

    /// 二阶偏导

    // 返回指定参数处相对于U参数的二阶偏导。
    virtual MyMath::Vector3 secondDerivativeUUAt(double u, double v) const = 0;
    // 返回指定参数处相对于U和V参数的混合二阶偏导。
    virtual MyMath::Vector3 secondDerivativeUVAt(double u, double v) const = 0;
    // 返回指定参数处相对于V参数的二阶偏导。
    virtual MyMath::Vector3 secondDerivativeVVAt(double u, double v) const = 0;

    /// 曲面方向

    // 返回指定正则参数处由U方向叉乘V方向确定的单位法向。
    virtual MyMath::Vector3 normalAt(double u, double v) const
    {
        MYBREP_ASSERT_MESSAGE(isParameterInDomain(u, v),
                              "Geometry_Surface normal parameters are outside the natural parameter domain.");

        const MyMath::Vector3 uDerivative = firstDerivativeUAt(u, v);
        const MyMath::Vector3 vDerivative = firstDerivativeVAt(u, v);
        const MyMath::Vector3 normal = MyMath::Vector3::cross(uDerivative, vDerivative);

        MYBREP_ASSERT_MESSAGE(normal.isVector(0.0),
                              "Geometry_Surface normal requires a regular surface parameter.");

        return normal.normalized(0.0);
    }

protected:
    // 通过Geometry_Object侵入式引用计数管理具体曲面几何生命周期。
    ~Geometry_Surface() override = default;
};

}

#endif // MYBREP_GEOMETRY_SURFACE_GEOMETRY_SURFACE_H