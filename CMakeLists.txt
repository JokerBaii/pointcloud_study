cmake_minimum_required(VERSION 3.16)

project(study_PCL VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# 新增库模块时，在这里添加 add_subdirectory(libs/模块名)。
add_subdirectory(libs/kdtree)

