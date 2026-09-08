# ============================================================
# MyBRepOpenGL
# MyBRep 与 MyOpenGL 之间的显示适配层
# ============================================================

set(MYBREP_OPENGL_SRC
    Builder/BRepWireframeBuilder.cpp
    Builder/BRepWireframeBuilder.h

    Display/BRepDisplayObject.cpp
    Display/BRepDisplayObject.h
    Display/BRepDisplayStyle.cpp
    Display/BRepDisplayStyle.h

    Viewer/BRepViewerWidget.cpp
    Viewer/BRepViewerWidget.h
)

# ============================================================
# 静态库
# ============================================================

add_library(MyBRepOpenGL STATIC
    ${MYBREP_OPENGL_SRC}
)

# ============================================================
# 头文件路径
# ============================================================

target_include_directories(MyBRepOpenGL
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/..
)

# ============================================================
# 依赖
# ============================================================

target_link_libraries(MyBRepOpenGL
    PUBLIC
        MyBRep
        MyOpenGLViewer
)