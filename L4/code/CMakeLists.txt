cmake_minimum_required(VERSION 2.8)
project(homework_04)

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



add_executable(undistort_image undistort_image.cpp)
target_link_libraries(undistort_image ${OpenCV_LIBS})

add_executable( gaussnewton gaussnewton.cpp )
target_link_libraries( gaussnewton ${OpenCV_LIBS})

add_executable( disparity disparity.cpp )
target_link_libraries( disparity ${OpenCV_LIBS} ${Pangolin_LIBRARIES})

