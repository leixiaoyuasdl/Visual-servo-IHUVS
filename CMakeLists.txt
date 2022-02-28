PROJECT(VanishingPoint)
add_definitions(-std=c++11)

find_package(orocos_kdl REQUIRED)
find_package( OpenCV REQUIRED )

include_directories(${orocos_kdl_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} )

ADD_EXECUTABLE(VanishingPoint main.cpp )
target_link_libraries(VanishingPoint ${orocos_kdl_LIBRARIES} ${OpenCV_LIBS})

ADD_EXECUTABLE(VanishingPoint_test main_test.cpp firstprocessor.cpp firstprocessor.h secondprocessor.cpp secondprocessor.h thirdprocessor.cpp thirdprocessor.h robot.cpp robot.h)
target_link_libraries(VanishingPoint_test ${orocos_kdl_LIBRARIES} ${OpenCV_LIBS})
