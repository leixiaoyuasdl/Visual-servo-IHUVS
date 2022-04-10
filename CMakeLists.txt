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
INCLUDE_DIRECTORIES(rotation_controller position_controller robot controller)
AUX_SOURCE_DIRECTORY(rotation_controller source_rotation)
AUX_SOURCE_DIRECTORY(position_controller source_position)
AUX_SOURCE_DIRECTORY(robot source_robot)
AUX_SOURCE_DIRECTORY(controller source_controller)

include_directories(${orocos_kdl_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


ADD_EXECUTABLE(rotation_controller main_rotation.cpp ${source_robot} ${source_rotation} )
target_link_libraries(rotation_controller ${orocos_kdl_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})


ADD_EXECUTABLE(position_controller main_pos.cpp ${source_robot} ${source_position} ${source_rotation} )
target_link_libraries(position_controller ${orocos_kdl_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})

ADD_EXECUTABLE(main main.cpp ${source_rotation}  ${source_position} ${source_robot} controller/controller.cpp controller/controller.h)
target_link_libraries(main ${orocos_kdl_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})
