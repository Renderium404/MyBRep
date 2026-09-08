#ifndef MYBREPOPENGL_DISPLAY_BREPDISPLAYSTYLE_H
#define MYBREPOPENGL_DISPLAY_BREPDISPLAYSTYLE_H

#include <QVector4D>

#include "MyBRepOpenGL/Builder/BRepWireframeBuilder.h"

namespace MyBRep
{
namespace Display
{

// BRepViewerWidget第一阶段线框显示样式。
struct BRepDisplayStyle
{
    BRepDisplayStyle();

    bool isValid() const;

    QVector4D wireColor;                    // 无光照统一边线颜色。
    BRepWireframeBuildOptions wireframe;    // 曲线离散精度和线宽。
};

}
}

#endif // MYBREPOPENGL_DISPLAY_BREPDISPLAYSTYLE_H
