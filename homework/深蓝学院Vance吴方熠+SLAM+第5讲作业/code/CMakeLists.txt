cmake_minimum_required(VERSION 2.8)
project(homework_05)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-std=c++11")
set(OpenCV_DIR "/home/vance/softwares/opencv-3.3.1/build")

find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin)

include_directories(
#    "/usr/include/eigen3/"
    ${Sophus_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
)

add_executable(computeORB computeORB.cpp)
target_link_libraries(computeORB ${OpenCV_LIBS})

add_executable(E2Rt E2Rt.cpp)
target_link_libraries(E2Rt ${OpenCV_LIBS})

add_executable(GN-BA GN-BA.cpp)
target_link_libraries(GN-BA ${OpenCV_LIBS}  ${Sophus_LIBRARIES})


add_executable(ICP_trajectory ICP_trajectory.cpp)
target_link_libraries(ICP_trajectory ${Pangolin_LIBRARIES}  ${Sophus_LIBRARIES})
