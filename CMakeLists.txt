cmake_minimum_required(VERSION 3.5)
project(eit-cobod)
set(CMAKE_CXX_STANDARD 11)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Set build type to release
SET(CMAKE_BUILD_TYPE Release)
MESSAGE("-- Build type: " ${CMAKE_BUILD_TYPE})

# Set the RobWork root (edit in .bashrc if necessary)
#SET(RW_ROOT $ENV{RW_ROOT})
#SET(RWHARDWARE_ROOT $ENV{RWHW_ROOT})

# Include robwork
find_package(RobWork REQUIRED)
link_directories( ${ROBWORK_LIBRARY_DIRS} )
include_directories( ${ROBWORK_INCLUDE_DIRS} )

# Include robworkhardware
FIND_PACKAGE(RobWorkHardware REQUIRED universalrobots_rtde)
link_directories( ${ROBWORKHARDWARE_LIBRARY_DIRS})
include_directories( ${ROBWORKHARDWARE_INCLUDE_DIRS})

# Include boost
find_package(Boost COMPONENTS system filesystem regex date_time thread chrono REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

#Include ur_rtde
#find_package(ur_rtde REQUIRED)

add_executable(eit-cobod main.cpp ur_control.cpp)
#target_link_libraries(eit-cobod PRIVATE ur_rtde::rtde)
target_link_libraries(eit-cobod ${ROBWORK_LIBRARIES} ${ROBWORKHARDWARE_LIBRARIES})
