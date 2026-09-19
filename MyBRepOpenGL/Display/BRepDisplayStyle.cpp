#include "BRepDisplayStyle.h"

#include "MyMath/MathUtils.h"

namespace MyBRep
{
namespace Display
{

BRepDisplayStyle::BRepDisplayStyle()
    : surfaceColor(0.72f, 0.76f, 0.82f, 1.0f)
    , wireColor(0.08f, 0.08f, 0.08f, 1.0f)
    , surfaceLightingEnabled(true)
{
}

bool BRepDisplayStyle::isValid() const
{
    return isFaceValid();
}

bool BRepDisplayStyle::isWireframeValid() const
{
    return isValidColor(wireColor);
}

bool BRepDisplayStyle::isFaceValid() const
{
    return isValidColor(surfaceColor) && isWireframeValid();
}

bool BRepDisplayStyle::isValidColor(const QVector4D& color)
{
    return MyMath::isFinite(static_cast<double>(color.x())) &&
           MyMath::isFinite(static_cast<double>(color.y())) &&
           MyMath::isFinite(static_cast<double>(color.z())) &&
           MyMath::isFinite(static_cast<double>(color.w())) &&
           color.x() >= 0.0f && color.x() <= 1.0f &&
           color.y() >= 0.0f && color.y() <= 1.0f &&
           color.z() >= 0.0f && color.z() <= 1.0f &&
           color.w() >= 0.0f && color.w() <= 1.0f;
}

}
}
