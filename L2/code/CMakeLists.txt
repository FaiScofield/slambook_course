cmake_minimum_required(VERSION 2.8)
project(homework)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#add_definitions(-DCOMPILEDWITHC11)

include_directories( "/usr/include/eigen3" )

add_executable(ques2 ques2.cpp)
add_executable(ques3 ques3.cpp)
add_executable(ques6 ques6.cpp)
