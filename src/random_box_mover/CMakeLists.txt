cmake_minimum_required(VERSION 2.8.3)
project(random_box_mover)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(random_box_mover src/random_box_mover.cpp)
target_link_libraries(random_box_mover ${catkin_LIBRARIES})

