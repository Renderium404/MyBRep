file(GLOB_RECURSE BREP_SRC CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
)


add_library(MyBRep STATIC
    ${BREP_SRC}
)

target_include_directories(MyBRep
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(MyBRep
    PUBLIC
    Qt5::Core
    Qt5::Gui
    MyMath          # 添加对 MyMath 的依赖
)