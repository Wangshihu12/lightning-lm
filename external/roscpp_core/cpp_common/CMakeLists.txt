# external/roscpp_core/cpp_common/CMakeLists.txt
# 最小可编版（去 catkin）

# 只创建一次库
add_library(cpp_common STATIC
  src/debug.cpp
)

target_include_directories(cpp_common PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 统一使用“关键字签名”写法（避免混用）
find_package(console_bridge CONFIG REQUIRED)
target_link_libraries(cpp_common PUBLIC console_bridge::console_bridge)
