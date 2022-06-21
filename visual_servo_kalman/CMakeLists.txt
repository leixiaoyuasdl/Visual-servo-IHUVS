cmake_minimum_required(VERSION 3.15)
PROJECT(visual_servo_kalman)
add_definitions(-std=c++11)

find_package(orocos_kdl REQUIRED)
find_package( OpenCV REQUIRED )
find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        roscpp
        rospy
        tf
        moveit_msgs
        moveit_ros_perception
        moveit_ros_planning_interface
        )
        
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES probot_demo
   CATKIN_DEPENDS roscpp rospy moveit_core moveit_ros_planning_interface
#  DEPENDS system_lib
)
AUX_SOURCE_DIRECTORY(src/lib source_file)

include_directories(include ${orocos_kdl_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECT_NAME}_kalman_6dof src/kalman_6dof.cpp ${source_file} )
target_link_libraries(${PROJECT_NAME}_kalman_6dof ${orocos_kdl_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})

ADD_EXECUTABLE(${PROJECT_NAME}_setgoal src/setgoal.cpp ${source_file} )
target_link_libraries(${PROJECT_NAME}_setgoal ${orocos_kdl_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})
