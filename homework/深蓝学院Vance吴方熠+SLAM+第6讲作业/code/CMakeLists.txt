cmake_minimum_required(VERSION 2.8)
project(homework_06)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-std=c++11")
set(OpenCV_DIR "/home/vance/softwares/opencv-3.3.1/build")

find_package(OpenCV  REQUIRED)
find_package(Pangolin)
find_package(Sophus REQUIRED)

include_directories(
#    "/usr/include/eigen3/"
    ${Sophus_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)



add_executable(optical_flow optical_flow.cpp)
target_link_libraries(optical_flow ${OpenCV_LIBS})

add_executable(direct_method direct_method.cpp)
target_link_libraries(direct_method ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES} )

