cmake_minimum_required(VERSION 3.0.2)
project(livox_ros_driver2)

find_package(catkin REQUIRED COMPONENTS
        std_msgs
        message_generation
)

add_message_files(
        DIRECTORY msg
        FILES CustomPoint.msg CustomMsg.msg
)

generate_messages(
        DEPENDENCIES std_msgs
)

catkin_package(
        CATKIN_DEPENDS std_msgs message_runtime
)

install(DIRECTORY msg
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        FILES_MATCHING PATTERN "*.msg"
)
