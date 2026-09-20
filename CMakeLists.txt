# CMake 最低版本要求
cmake_minimum_required(VERSION 3.5)

# 定义项目
project(MyBRep)

# 查找 Qt5
find_package(Qt5 5.8 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

add_subdirectory(MyMath)
add_subdirectory(MyBRep)
add_subdirectory(MyOpenGL)
add_subdirectory(MyBRepOpenGL)
add_subdirectory(MyEnvelope)
add_subdirectory(SandBox)
add_subdirectory(App)