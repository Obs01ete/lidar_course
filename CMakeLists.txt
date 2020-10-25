cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(pcl-test)

find_package(PCL 1.11 COMPONENTS common io segmentation visualization surface REQUIRED CONFIG)
find_package(Boost COMPONENTS system REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "pcl lib dir: ${PCL_LIBRARY_DIRS}")

link_directories(${Boost_LIBRARY_DIR})

add_executable(process-pcd src/process_pcd.cpp src/visualizer.cpp)
target_link_libraries(process-pcd ${PCL_LIBRARIES})

add_executable(bin-to-pcd src/bin_to_pcd.cpp)
target_link_libraries(bin-to-pcd ${PCL_LIBRARIES})

add_executable(process-sequence src/process_sequence.cpp src/visualizer.cpp)
target_link_libraries(process-sequence ${PCL_LIBRARIES} pthread png)

SET(COMPILE_FLAGS "-std=c++14 -Wno-deprecated")
add_definitions(${COMPILE_FLAGS})

set(CMAKE_BUILD_TYPE Debug)
