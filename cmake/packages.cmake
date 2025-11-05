find_package(glog CONFIG REQUIRED)
find_package(gflags CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenGL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(TBB CONFIG REQUIRED)

# vcpkg toolchain 外部传入：-DCMAKE_TOOLCHAIN_FILE=...
find_package(ZLIB REQUIRED)
find_package(BZip2 REQUIRED)
find_package(lz4 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

# 先做四个核心组件
add_subdirectory(external/roscpp_core/cpp_common)
add_subdirectory(external/roscpp_core/roscpp_serialization)
add_subdirectory(external/roscpp_core/rostime)

# 这两个基本为 header-only — 我们给它们定义 INTERFACE 目标，方便依赖
add_library(roscpp_traits INTERFACE)
target_include_directories(roscpp_traits INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/external/roscpp_core/roscpp_traits/include)

# add_library(roscpp_serialization INTERFACE)
# target_include_directories(roscpp_serialization INTERFACE
#   ${CMAKE_CURRENT_SOURCE_DIR}/external/roscpp_core/roscpp_serialization/include)
# target_link_libraries(roscpp_serialization INTERFACE roscpp_traits)

add_subdirectory(external/roslz4)

# 编译 rosbag_storage（去掉 catkin，直接普通 CMake）
# external/ros_comm/tools/rosbag_storage
file(GLOB ROSBAG_STORAGE_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/external/ros_comm/tools/rosbag_storage/src/*.cpp)

# 去掉需要 gpgme / 外部加密插件的实现文件
list(REMOVE_ITEM ROSBAG_STORAGE_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/external/ros_comm/tools/rosbag_storage/src/aes_encryptor.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/external/ros_comm/tools/rosbag_storage/src/gpgme_utils.cpp
)

add_library(rosbag_storage STATIC ${ROSBAG_STORAGE_SRC})

# 关闭 pluginlib / gpgme（非常关键！）
target_compile_definitions(rosbag_storage PRIVATE
  HAVE_PLUGINLIB=0
  ROSBAG_NO_ENCRYPTION=1
  ROSBAG_STORAGE_NO_GPGME=1
  ROSBAG_STORAGE_NO_ENCRYPTION=1
)

target_include_directories(rosbag_storage PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/external/ros_comm/tools/rosbag_storage/include
  ${CMAKE_CURRENT_SOURCE_DIR}/external/roscpp_core/rostime/include
  ${CMAKE_CURRENT_SOURCE_DIR}/external/roscpp_core/cpp_common/include
  ${CMAKE_CURRENT_SOURCE_DIR}/external/roscpp_core/roscpp_traits/include
  ${CMAKE_CURRENT_SOURCE_DIR}/external/roscpp_core/roscpp_serialization/include
  ${CMAKE_CURRENT_SOURCE_DIR}/external/roslz4/include         # roslz4/lz4s.h 的头
)

target_link_libraries(rosbag_storage
  PRIVATE Boost::filesystem Boost::system ZLIB::ZLIB BZip2::BZip2 lz4::lz4 roslz4
  PUBLIC  rostime cpp_common roscpp_serialization
)

# OMP
find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

if (BUILD_WITH_MARCH_NATIVE)
    add_compile_options(-march=native)
else ()
    add_definitions(-std=c++17 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
endif ()

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${GLOG_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${GLEW_INCLUDE_DIRS}
)

include_directories(
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/thirdparty
)


set(third_party_libs
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        ${Pangolin_LIBRARIES}
        glog::glog gflags::gflags
        ${yaml-cpp_LIBRARIES}
        yaml-cpp::yaml-cpp
        TBB::tbb
        rosbag_storage
        roscpp_serialization
        rostime
        cpp_common
        ZLIB::ZLIB BZip2::BZip2 lz4::lz4
)

