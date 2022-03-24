cmake_minimum_required(VERSION 2.8.3)
PROJECT(VanishingPoint)
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
        )
INCLUDE_DIRECTORIES(./)
INCLUDE_DIRECTORIES(./include)
AUX_SOURCE_DIRECTORY(./ gSOURCE_FILES_)

include_directories(${orocos_kdl_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


ADD_EXECUTABLE(VanishingPoint_test main_test.cpp ${gSOURCE_FILES_})
target_link_libraries(VanishingPoint_test ${orocos_kdl_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})
