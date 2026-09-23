# ============================================================
# MyBRepOpenGL
# MyBRep 与 MyOpenGL 之间的显示适配层
# ============================================================

set(MYBREP_OPENGL_SRC
    Builder/BRepFaceBuilder.cpp
    Builder/BRepFaceBuilder.h
    Builder/BRepEdgeBuilder.cpp
    Builder/BRepEdgeBuilder.h
    Builder/BRepShapeBuilder.cpp
    Builder/BRepShapeBuilder.h

    Display/BRepDisplayManager.cpp
    Display/BRepDisplayManager.h
    Display/BRepDisplayObject.cpp
    Display/BRepDisplayObject.h
    Display/BRepDisplayStyle.cpp
    Display/BRepDisplayStyle.h

    Viewer/BRepViewerWidget.cpp
    Viewer/BRepViewerWidget.h
)

add_library(MyBRepOpenGL STATIC
    ${MYBREP_OPENGL_SRC}
)

target_include_directories(MyBRepOpenGL
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_link_libraries(MyBRepOpenGL
    PUBLIC
        MyBRep
        MyOpenGLViewer
)
