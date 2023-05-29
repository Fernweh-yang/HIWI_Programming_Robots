cmake_minimum_required(VERSION 3.0)
project(task8)

include_directories("/usr/include/eigen3")
# 设置C++标准为C++11
set(CMAKE_CXX_STANDARD 11)

# 寻找Pinocchio库
find_package(pinocchio REQUIRED)

# 添加可执行文件
add_executable(task8 task8.cpp)

# 将Pinocchio库链接到可执行文件
target_link_libraries(task8 PUBLIC pinocchio::pinocchio)
