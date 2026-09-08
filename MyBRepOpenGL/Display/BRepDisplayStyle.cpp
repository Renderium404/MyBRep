#include "BRepDisplayStyle.h"

#include <limits>

namespace
{

bool isFiniteValue(float value)
{
    const float infinity = (std::numeric_limits<float>::infinity)();
    return value == value && value != infinity && value != -infinity;
}

bool isValidColor(const QVector4D& color)
{
    return isFiniteValue(color.x()) && isFiniteValue(color.y()) && isFiniteValue(color.z()) && isFiniteValue(color.w()) &&
           color.x() >= 0.0f && color.y() >= 0.0f && color.z() >= 0.0f && color.w() >= 0.0f && color.w() <= 1.0f;
}

}

namespace MyBRep
{
namespace Display
{

BRepDisplayStyle::BRepDisplayStyle()
    : wireColor(0.08f, 0.08f, 0.08f, 1.0f) // 深灰用于浅色Viewer背景上的普通B-Rep边界显示。
{
}

bool BRepDisplayStyle::isValid() const
{
    return isValidColor(wireColor) && wireframe.isValid();
}

}
}
