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
    return isFiniteValue(color.x()) && isFiniteValue(color.y()) &&
           isFiniteValue(color.z()) && isFiniteValue(color.w()) &&
           color.x() >= 0.0f && color.x() <= 1.0f &&
           color.y() >= 0.0f && color.y() <= 1.0f &&
           color.z() >= 0.0f && color.z() <= 1.0f &&
           color.w() >= 0.0f && color.w() <= 1.0f;
}

}

namespace MyBRep
{
namespace Display
{

BRepDisplayStyle::BRepDisplayStyle()
    : surfaceColor(0.72f, 0.76f, 0.82f, 1.0f) // 默认使用中性浅灰蓝，便于观察光照产生的曲面明暗变化。
    , wireColor(0.08f, 0.08f, 0.08f, 1.0f)    // 默认使用深灰边界，保证浅色表面上的轮廓辨识度。
    , surfaceLightingEnabled(true)             // BRepViewerWidget提供默认场景灯光，因此表面默认启用法向漫反射。
{
}

bool BRepDisplayStyle::isValid() const
{
    return isFaceValid();
}

bool BRepDisplayStyle::isWireframeValid() const
{
    return isValidColor(wireColor) && wireframe.isValid();
}

bool BRepDisplayStyle::isFaceValid() const
{
    return isValidColor(surfaceColor) && isWireframeValid() && surface.isValid();
}

}
}