cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(cg_recognition)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cg_recognition cg_recognition.cpp)
target_link_libraries (cg_recognition ${PCL_LIBRARIES})
