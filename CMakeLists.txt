cmake_minimum_required(VERSION 3.5)
project(eit-cobod)

find_package(ur_rtde REQUIRED)
add_executable(eit-cobod main.cpp)
target_link_libraries(eit-cobod PRIVATE ur_rtde::rtde)
