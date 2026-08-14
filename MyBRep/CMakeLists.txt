add_library(MyBRep STATIC

)

target_include_directories(MyBRep
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(MyBRep
    PUBLIC
    Qt5::Core
    Qt5::Gui
)