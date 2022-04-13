cmake_minimum_required(VERSION 3.15)
PROJECT(VanishingPoint )
add_definitions(-std=c++11)

find_package(orocos_kdl REQUIRED)
find_package( OpenCV REQUIRED )
find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        roscpp
        rospy
        tf
        sensor_msgs
        trajectory_msgs
        moveit_msgs
        moveit_ros_perception
        moveit_ros_planning_interface
        trajectory_msgs
        )
AUX_SOURCE_DIRECTORY(src source_file)

include_directories(include ${orocos_kdl_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

ADD_EXECUTABLE(main main.cpp ${source_file} )
target_link_libraries(main ${orocos_kdl_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})
