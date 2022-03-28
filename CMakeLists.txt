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
INCLUDE_DIRECTORIES(./findchessboard)
AUX_SOURCE_DIRECTORY(./findchessboard gSOURCE_FILES_)

include_directories(${orocos_kdl_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


ADD_EXECUTABLE(VanishingPoint main.cpp ${gSOURCE_FILES_} )
target_link_libraries(VanishingPoint ${orocos_kdl_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})
