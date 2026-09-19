#ifndef MYMATH_MATH_H
#define MYMATH_MATH_H

namespace MyMath
{

/// 数学常量
extern const double Pi;
extern const double HalfPi;
extern const double TwoPi;

/// 数值状态

// 判断数值是否为有限值，不包含NaN和正负无穷。
bool isFinite(double value);
// 判断数值是否为NaN。
bool isNaN(double value);
// 返回双精度静默NaN。
double quietNaN();

/// 标量计算

// 将数值限制在闭区间[minimum, maximum]内，调用者必须保证minimum不大于maximum。
double clamp(double value, double minimum, double maximum);
double maximumAbsolute(double first, double second);
double maximumAbsolute(double first, double second, double third);
double maximumAbsolute(double first, double second, double third, double fourth);

/// 稳定模长，防止溢出
double scaledNorm(double first, double second, double scale);
double scaledNorm(double first, double second, double third, double scale);
double scaledNorm(double first, double second, double third, double fourth, double scale);

///模长
double norm(double first, double second);
double norm(double first, double second, double third);
double norm(double first, double second, double third, double fourth);

/// 角度转换
double degreesToRadians(double degrees);        // 将角度转换为弧度。
double radiansToDegrees(double radians);        // 将弧度转换为角度。

}

#endif // MYMATH_MATH_H