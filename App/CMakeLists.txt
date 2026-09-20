# ============================================================
# MyBRepApp
# MyBRep综合集成应用
# ============================================================

set(MYBREP_APP_SRC
    main.cpp
    MainWindow.cpp
    MainWindow.h

    UI/MainWindow.ui
)

add_executable(MyBRepApp
    ${MYBREP_APP_SRC}
)

target_include_directories(MyBRepApp
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_link_libraries(MyBRepApp
    PRIVATE
        MyBRepOpenGL
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
)

set_target_properties(MyBRepApp PROPERTIES
    AUTOMOC ON
    AUTOUIC ON
    AUTORCC ON
    AUTOUIC_SEARCH_PATHS "${CMAKE_CURRENT_SOURCE_DIR}/UI"
    FOLDER "Applications"
)