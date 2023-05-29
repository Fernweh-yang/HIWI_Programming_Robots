cmake_minimum_required(VERSION 3.0)
project(Task7_Pinocchio)

include_directories("/usr/include/eigen3")
# 设置C++标准为C++11
set(CMAKE_CXX_STANDARD 11)

# 寻找Pinocchio库
find_package(pinocchio REQUIRED)

# 添加可执行文件
add_executable(task7_pinocchio task7_pinocchio.cpp)

# # 将Pinocchio库链接到可执行文件
target_link_libraries(task7_pinocchio PUBLIC pinocchio::pinocchio)
# target_include_directories(task7_pinocchio PRIVATE ${PINOCCHIO_INCLUDE_DIRS})
