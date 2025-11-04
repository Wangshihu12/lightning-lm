cmake_minimum_required(VERSION 3.16)
project(roscpp_serialization CXX)

# 收集实现文件：有就编进来（不同分支文件名可能不同）
set(ROSCPP_SERIALIZATION_SRC
  ${CMAKE_CURRENT_LIST_DIR}/src/serialization.cpp
  # ${CMAKE_CURRENT_LIST_DIR}/src/header.cpp
)
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/src/header.cpp")
  list(APPEND ROSCPP_SERIALIZATION_SRC ${CMAKE_CURRENT_LIST_DIR}/src/header.cpp)
endif()
# 有些分支把 Header 实现放到另外的 cpp，例如 ros/header.cpp；兜底再探测一次
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/src/ros/header.cpp")
  list(APPEND ROSCPP_SERIALIZATION_SRC ${CMAKE_CURRENT_LIST_DIR}/src/ros/header.cpp)
endif()

add_library(roscpp_serialization STATIC ${ROSCPP_SERIALIZATION_SRC})

# 头文件：必须能看到 ros/time.h、ros/macros.h、ros/message_traits.h
target_include_directories(roscpp_serialization PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${CMAKE_CURRENT_LIST_DIR}/../rostime/include
  ${CMAKE_CURRENT_LIST_DIR}/../cpp_common/include
  ${CMAKE_CURRENT_LIST_DIR}/../roscpp_traits/include
)

# 依赖：这些库提供宏与 Time 等符号
target_link_libraries(roscpp_serialization PUBLIC
  rostime
  cpp_common
  roscpp_traits
)
