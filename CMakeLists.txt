cmake_minimum_required(VERSION 3.3)

project(pcl_multi_openni2_recorder)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SOURCE_FILES src/main.cpp src/PCDBuffer.h src/Producer.h src/Consumer.h src/Util.h
        src/Timer.h src/Logger.h src/Logger.cpp src/CloudRecord.h src/RecorderNode.h src/Recorder.h)

add_executable(pcl_multi_openni2_recorder ${SOURCE_FILES})

target_link_libraries(pcl_multi_openni2_recorder ${PCL_LIBRARIES})