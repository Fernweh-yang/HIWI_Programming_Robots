cmake_minimum_required(VERSION 3.0.2)
project(dqpanda)

FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
ADD_COMPILE_OPTIONS(-Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  roscpp
)

include_directories( ${catkin_INCLUDE_DIRS} )

add_executable(task_3_4 src/task_3_4.cpp)
target_link_libraries(task_3_4 dqrobotics dqrobotics-interface-vrep ${catkin_LIBRARIES})

add_executable(task5 src/task5.cpp)
target_link_libraries(task5 dqrobotics dqrobotics-interface-vrep ${catkin_LIBRARIES})