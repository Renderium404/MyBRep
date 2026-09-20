
file(GLOB SANDBOX_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

foreach(SANDBOX_SOURCE ${SANDBOX_SOURCES})
    get_filename_component(TARGET_NAME
        "${SANDBOX_SOURCE}"
        NAME_WE
    )

    add_executable(${TARGET_NAME}
        "${SANDBOX_SOURCE}"
    )

    target_include_directories(${TARGET_NAME}
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${CMAKE_SOURCE_DIR}
    )

    if(TARGET_NAME MATCHES "^my_brep_opengl_")
        target_link_libraries(${TARGET_NAME}
            PRIVATE
                MyBRepOpenGL
        )
    elseif(TARGET_NAME MATCHES "^my_brep_")
        target_link_libraries(${TARGET_NAME}
            PRIVATE
                MyBRep
        ) 
    else()
        message(FATAL_ERROR
            "Unknown SandBox target type: ${TARGET_NAME}"
        )
    endif()
endforeach()