set(MYENVELOPE_SOURCES
    Profile.h
    Line.h
    Line.cpp
    Arc.h
    Arc.cpp
)

set(MYENVELOPE_HEADERS
    Profile.h
    Line.h
    Line.cpp
    Arc.h
    Arc.cpp
)

add_library(MyEnvelope STATIC ${MYENVELOPE_SOURCES} ${MYENVELOPE_HEADERS})

target_include_directories(MyEnvelope PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/.."
)

target_link_libraries(MyEnvelope PUBLIC
    MyBRep
    MyMath
)

set_target_properties(MyEnvelope PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)
