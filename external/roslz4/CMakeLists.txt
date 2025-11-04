cmake_minimum_required(VERSION 3.16)
project(roslz4 C)

add_library(roslz4 STATIC src/lz4s.c src/xxhash.c)

# roslz4 自身头 + cpp_common（提供 ros/macros.h）
target_include_directories(roslz4 PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${CMAKE_SOURCE_DIR}/external/roscpp_core/cpp_common/include
)

# 让 lz4s.c 能找到本地的 xxhash.h
target_include_directories(roslz4 PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)

# LZ4 仍用系统库
find_package(lz4 REQUIRED)
target_link_libraries(roslz4 PUBLIC lz4::lz4)
