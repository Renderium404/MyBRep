#ifndef MYBREPOPENGL_DISPLAY_BREPDISPLAYSTYLE_H
#define MYBREPOPENGL_DISPLAY_BREPDISPLAYSTYLE_H

#include <QVector4D>

namespace MyBRep
{
namespace Display
{

// BRepViewerWidget单个显示实例的显示样式。
// 离散参数由BRepViewerWidget统一管理，不属于实例Style。
struct BRepDisplayStyle
{
    BRepDisplayStyle();

    // 判断全部实例显示参数是否有效。
    bool isValid() const;
    // 判断线框显示所需颜色参数是否有效。
    bool isWireframeValid() const;
    // 判断Face/Shell/Solid实体显示所需参数是否有效。
    bool isFaceValid() const;

    QVector4D surfaceColor;      // Face表面统一颜色。
    QVector4D wireColor;         // B-Rep边界统一颜色。
    bool surfaceLightingEnabled; // Face表面是否启用MyOpenGL基础光照。

private:
    // 判断RGBA颜色四个有限分量是否全部位于[0,1]。
    static bool isValidColor(const QVector4D& color);
};

}
}

#endif // MYBREPOPENGL_DISPLAY_BREPDISPLAYSTYLE_H
