# Test CMake version
cmake_minimum_required(VERSION 3.5)

# The name of the project.
project(eit-plugin)

# Used to resolve absolute path names
set(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Now set the RW/RWS root (edit this if necessary)
if(DEFINED ENV{RW_ROOT} AND DEFINED ENV{RWS_ROOT})
  file(TO_CMAKE_PATH "$ENV{RW_ROOT}" RW_ROOT)
  file(TO_CMAKE_PATH "$ENV{RWS_ROOT}" RWSTUDIO_ROOT)
else()
  set(RW_ROOT "${ROOT}/../../../RobWork")
  set(RWSTUDIO_ROOT "${ROOT}/../../../RobWorkStudio")
endif()

# If not the -DCMAKE_BUILD_TYPE option is given when running CMake, use Release as default.
if( NOT DEFINED CMAKE_BUILD_TYPE )
    set(CMAKE_BUILD_TYPE Release)
endif()
message("-- Build type: " ${CMAKE_BUILD_TYPE})

# Include default settings for constructing a RobWork dependent project
set(RobWork_DIR ${RW_ROOT}/cmake)
find_package(RobWork REQUIRED)
message("-- RobWork found: " ${RW_ROOT})

# Include default settings for constructing a RobWorkStudio dependent project
set(RobWorkStudio_DIR ${RWSTUDIO_ROOT}/cmake)
find_package(RobWorkStudio REQUIRED)
message("-- RobWorkStudio found: " ${RWSTUDIO_ROOT})

# Set the output dir for generated libraries and binaries
if(MSVC)
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin" CACHE PATH "Runtime directory" FORCE)
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Library directory" FORCE)
        set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Archive directory" FORCE)
else()
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin/${CMAKE_BUILD_TYPE}" CACHE PATH "Runtime directory" FORCE)
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Library directory" FORCE)
        set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs/${CMAKE_BUILD_TYPE}" CACHE PATH "Archive directory" FORCE)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# RTDE
find_package(ur_rtde REQUIRED)

# PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# Add the source files to build
set(SrcFiles Plugin.cpp Plugin.hpp Plugin.json resources.qrc)

# The shared library to build:
add_library(Plugin MODULE ${SrcFiles})
target_link_libraries(Plugin PUBLIC ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES} ${PCL_LIBRARIES} PRIVATE ur_rtde::rtde)
target_include_directories(Plugin PUBLIC ${ROBWORKSTUDIO_INCLUDE_DIRS} ${ROBWORK_INCLUDE_DIRS})

message("-- " ${PROJECT_NAME} " done!")

