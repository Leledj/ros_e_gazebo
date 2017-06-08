cmake_minimum_required(VERSION 2.8.3)
project(mybot_control)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	sensor_msgs
	nav_msgs
	tf
)

catkin_package()

add_executable(scansub src/scansub.cpp)
target_link_libraries(scansub ${catkin_LIBRARIES})

add_executable(mybot_control src/mybot_control.cpp)
target_link_libraries(mybot_control ${catkin_LIBRARIES})
