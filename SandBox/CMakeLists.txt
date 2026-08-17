#收集沙盒源文件
file(GLOB_RECURSE SANDBOX_SRC CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

#遍历沙盒源文件，并创建对应的.exe
foreach(SOURCE_FILE IN LISTS SANDBOX_SRC)
    #提取文件名,去掉路径和扩展名，得到基础文件名（如 test.cpp → test）
    get_filename_component(
        SOURCE_NAME
        "${SOURCE_FILE}"
        NAME_WE
    )
    #生成合法目标名,将文件名转换为合法的 C 标识符（例如将非法字符转为下划线），确保 CMake 目标名有效
    string(
        MAKE_C_IDENTIFIER
        "${SOURCE_NAME}"
        TARGET_NAME
    )
    #检查是否已存在同名目标，若有则报错，避免重复定义
    if(TARGET ${TARGET_NAME})
        message(FATAL_ERROR
            "Duplicate sandbox target name: ${TARGET_NAME}\n"
            "Source: ${SOURCE_FILE}"
        )
    endif()
    #创建可执行文件
    add_executable(${TARGET_NAME}
        "${SOURCE_FILE}"
    )
    #将当前目录加入该目标的头文件搜索路径
    target_include_directories(${TARGET_NAME} PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}"
    )
    #链接公共库
    target_link_libraries(${TARGET_NAME} PRIVATE
        MyBRep
    )

    #输出信息
    message(STATUS
        "Sandbox: ${TARGET_NAME}"
    )
endforeach()