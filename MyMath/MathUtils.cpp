#include "MathUtils.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

namespace MyMath
{

const double Pi = 3.141592653589793238462643383279502884;
const double HalfPi = Pi * 0.5;
const double TwoPi = Pi * 2.0;

/// 数值状态

bool isFinite(double value)
{
    const double infinity = std::numeric_limits<double>::infinity();
    return value == value && value != infinity && value != -infinity;
}

bool isNaN(double value)
{
    return value != value;
}

double quietNaN()
{
    return std::numeric_limits<double>::quiet_NaN();
}

/// 标量计算

double clamp(double value, double minimum, double maximum)
{
    assert(minimum <= maximum);
    return (std::max)(minimum, (std::min)(value, maximum));
}

double maximumAbsolute(double first, double second)
{
    return (std::max)(std::fabs(first), std::fabs(second));
}

double maximumAbsolute(double first, double second, double third)
{
    return (std::max)(maximumAbsolute(first, second), std::fabs(third));
}

double maximumAbsolute(double first, double second, double third, double fourth)
{
    return (std::max)(maximumAbsolute(first, second, third), std::fabs(fourth));
}

/// 稳定模长

double scaledNorm(double first, double second, double scale)
{
    assert(isFinite(first) && isFinite(second));
    assert(isFinite(scale) && scale > 0.0);

    const double scaledFirst = first / scale;
    const double scaledSecond = second / scale;
    return std::sqrt(scaledFirst * scaledFirst + scaledSecond * scaledSecond);
}

double scaledNorm(double first, double second, double third, double scale)
{
    assert(isFinite(first) && isFinite(second) && isFinite(third));
    assert(isFinite(scale) && scale > 0.0);

    const double scaledFirst = first / scale;
    const double scaledSecond = second / scale;
    const double scaledThird = third / scale;
    return std::sqrt(scaledFirst * scaledFirst + scaledSecond * scaledSecond + scaledThird * scaledThird);
}

double scaledNorm(double first, double second, double third, double fourth, double scale)
{
    assert(isFinite(first) && isFinite(second) && isFinite(third) && isFinite(fourth));
    assert(isFinite(scale) && scale > 0.0);

    const double scaledFirst = first / scale;
    const double scaledSecond = second / scale;
    const double scaledThird = third / scale;
    const double scaledFourth = fourth / scale;
    return std::sqrt(scaledFirst * scaledFirst + scaledSecond * scaledSecond + scaledThird * scaledThird + scaledFourth * scaledFourth);
}

double norm(double first, double second)
{
    assert(isFinite(first) && isFinite(second));

    const double scale = maximumAbsolute(first, second);
    if (scale == 0.0) return 0.0;

    return scale * scaledNorm(first, second, scale);
}

double norm(double first, double second, double third)
{
    assert(isFinite(first) && isFinite(second) && isFinite(third));

    const double scale = maximumAbsolute(first, second, third);
    if (scale == 0.0) return 0.0;

    return scale * scaledNorm(first, second, third, scale);
}

double norm(double first, double second, double third, double fourth)
{
    assert(isFinite(first) && isFinite(second) && isFinite(third) && isFinite(fourth));

    const double scale = maximumAbsolute(first, second, third, fourth);
    if (scale == 0.0) return 0.0;

    return scale * scaledNorm(first, second, third, fourth, scale);
}

/// 角度转换

double degreesToRadians(double degrees)
{
    return degrees * Pi / 180.0;
}

double radiansToDegrees(double radians)
{
    return radians * 180.0 / Pi;
}

}