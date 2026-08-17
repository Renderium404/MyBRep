# CMake 最低版本要求
cmake_minimum_required(VERSION 3.5)

# 定义项目，PROJECT_NAME 的值将为 MyBRep
project(MyBRep)

# 查找 Qt5：
# 要求版本 >= 5.8
# REQUIRED：找不到时直接终止 CMake 配置
# COMPONENTS Widgets：需要 Qt Widgets 模块
find_package(Qt5 5.8 REQUIRED COMPONENTS Widgets)


set(CMAKE_AUTOMOC ON)       #是否自动运行 Qt 的 moc 工具
set(CMAKE_AUTOUIC ON)       #是否自动使用 Qt 的 uic 处理 .ui 文件
set(CMAKE_AUTORCC ON)       #是否自动使用 Qt 的 rcc 处理 .qrc 资源文件

# ============================================================
# 新框架
# ============================================================
add_subdirectory(MyMath)

add_subdirectory(MyBRep)

# ============================================================
# 测试沙盒
# ============================================================

add_subdirectory(SandBox)