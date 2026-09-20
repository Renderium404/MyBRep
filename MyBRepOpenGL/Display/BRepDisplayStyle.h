#ifndef MYBREPOPENGL_DISPLAY_BREPDISPLAYSTYLE_H
#define MYBREPOPENGL_DISPLAY_BREPDISPLAYSTYLE_H

#include <QVector4D>

namespace MyBRep
{
namespace Display
{

// BRepViewerWidget单个Instance的显示样式。
// 几何离散参数属于Builder全局配置，不属于实例Style。
struct BRepDisplayStyle
{
    BRepDisplayStyle();

    bool isValid() const;
    bool isWireframeValid() const;
    bool isFaceValid() const;

    QVector4D surfaceColor;      // Face表面颜色；Alpha<1时使用透明混合。
    QVector4D wireColor;         // Edge线框颜色；Alpha<1时使用透明混合。
    float wireWidth;             // Edge显示线宽，单位Pixel。
    bool surfaceLightingEnabled; // Face表面是否启用基础光照。

private:
    static bool isValidColor(const QVector4D& color);
};

}
}

#endif // MYBREPOPENGL_DISPLAY_BREPDISPLAYSTYLE_H