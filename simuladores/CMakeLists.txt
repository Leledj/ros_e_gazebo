cmake_minimum_required(VERSION 2.8.3)
project(simuladores)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	geometry_msgs 
	turtlesim
)

catkin_package()


add_executable(hello src/hello.cpp)
target_link_libraries(hello ${catkin_LIBRARIES})

add_executable(pubvel src/pubvel.cpp)
target_link_libraries(pubvel ${catkin_LIBRARIES})

add_executable(subpose src/subpose.cpp)
target_link_libraries(subpose ${catkin_LIBRARIES})
















