#ifndef MYBREPOPENGL_DISPLAY_BREPDISPLAYSTYLE_H
#define MYBREPOPENGL_DISPLAY_BREPDISPLAYSTYLE_H

#include <QVector4D>

#include "MyBRepOpenGL/Builder/BRepFaceBuilder.h"
#include "MyBRepOpenGL/Builder/BRepWireframeBuilder.h"

namespace MyBRep
{
namespace Display
{

// BRepViewerWidget统一显示样式。
// Wireframe显示只使用wireColor和wireframe；Face实体显示同时使用surface与wireframe两组参数。
struct BRepDisplayStyle
{
    BRepDisplayStyle();

    // 判断全部显示参数是否有效。
    bool isValid() const;
    // 判断线框显示所需参数是否有效。
    bool isWireframeValid() const;
    // 判断Face实体显示所需参数是否有效。
    bool isFaceValid() const;

    QVector4D surfaceColor;              // Face表面统一颜色。
    QVector4D wireColor;                 // B-Rep边界统一颜色。
    bool surfaceLightingEnabled;         // Face表面是否启用MyOpenGL基础光照。
    BRepFaceBuildOptions surface;        // Face三角网格构建参数。
    BRepWireframeBuildOptions wireframe; // B-Rep边界曲线离散精度和线宽。
};

}
}

#endif // MYBREPOPENGL_DISPLAY_BREPDISPLAYSTYLE_H
