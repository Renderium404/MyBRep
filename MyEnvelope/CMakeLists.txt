set(MYENVELOPE_SOURCES
    # Geometry/RevolvedEnvelopeSurface.cpp
    # Motion/MotionSegment.cpp
    # Envelope/EnvelopeSurfaceBuilder.cpp
)

set(MYENVELOPE_HEADERS
    # Geometry/RevolvedEnvelopeSurface.h
    # Motion/MotionSegment.h
    # Envelope/RevolvedEnvelopeBranch.h
    # Envelope/EnvelopeSurfaceBuilder.h
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
