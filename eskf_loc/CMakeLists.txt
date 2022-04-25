cmake_minimum_required(VERSION 2.8.3)
project(localization)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    genmsg
    message_generation
    rosmsg
)

find_package(Eigen3)

generate_messages(DEPENDENCIES std_msgs rosmsg)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp std_msgs rosmsg
    #DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_library(eskf 
    src/ins.cpp 
    src/kalman_filter.cpp 
    src/kfstates.cpp 
    src/motion_monitor.cpp 
    src/kf_utils.cpp
)

add_executable(localization
    src/localization.cpp
)
target_link_libraries(localization
    ${catkin_LIBRARIES}
    yaml-cpp 
    eskf
)
