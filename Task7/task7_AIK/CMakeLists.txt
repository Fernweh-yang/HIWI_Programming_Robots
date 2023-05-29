cmake_minimum_required(VERSION 3.0.2)
project(task7)

include_directories("/usr/include/eigen3")

find_package (Threads REQUIRED)

add_executable(task7_AIK task7_AIK.cpp)
target_link_libraries(task7_AIK dqrobotics dqrobotics-interface-vrep ${CMAKE_THREAD_LIBS_INIT})
