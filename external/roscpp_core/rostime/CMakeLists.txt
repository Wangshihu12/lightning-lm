# external/roscpp_core/rostime/CMakeLists.txt

add_library(rostime STATIC
  src/duration.cpp
  src/rate.cpp
  src/time.cpp
  # src/wall_duration.cpp
  # src/wall_time.cpp
)

target_sources(rostime PRIVATE src/time_constants.cpp)

target_include_directories(rostime PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../cpp_common/include
)

target_link_libraries(rostime PUBLIC cpp_common)
