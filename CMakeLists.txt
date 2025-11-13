cmake_minimum_required(VERSION 3.10)

project(lightning)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

# set to release by default
set(CMAKE_BUILD_TYPE Release)

# -------------------------------------------------------------
# 导入第三方库，同时做一些全局编译选项配置
add_definitions("-DPCL_NO_PRECOMPILE")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fopenmp -fPIC -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O2 -fopenmp -fPIC -g -ggdb")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -ggdb -g -fopenmp -fPIC -g")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -g")

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/libs)
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/libs)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        std_srvs
        visualization_msgs
        tf2
        tf2_ros
        pcl_conversions
        pcl_ros
        message_generation
        rosbag
        livox_ros_driver2
)

add_service_files(
        DIRECTORY srv
        FILES LocCmd.srv SaveMap.srv
)

generate_messages(
        DEPENDENCIES std_msgs geometry_msgs sensor_msgs nav_msgs
)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(cmake/packages.cmake)

catkin_package(
        LIBRARIES ${PROJECT_NAME}.libs miao.core miao.utils
        CATKIN_DEPENDS message_runtime roscpp std_msgs geometry_msgs sensor_msgs nav_msgs std_srvs visualization_msgs
        tf2 tf2_ros pcl_conversions pcl_ros rosbag livox_ros_driver2
        DEPENDS Eigen3 OpenCV PCL OpenGL glog yaml-cpp Pangolin
        INCLUDE_DIRS src ${PROJECT_SOURCE_DIR}/thirdparty
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/thirdparty
        ${THIRD_PARTY_INCLUDE_DIRS}
)

install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

add_subdirectory(src)
