cmake_minimum_required( VERSION 2.8.12 )

project( downward_ros )

find_package( catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    roslib
)


include_directories(${catkin_INCLUDE_DIRS} include)

add_service_files(
    DIRECTORY srv
    FILES
    Plan.srv
)

generate_messages(
    DEPENDENCIES roscpp std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)

#add_subdirectory(src/ros)
