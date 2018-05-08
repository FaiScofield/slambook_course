cmake_minimum_required(VERSION 2.8)
project(homework_03)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++11")

# Sophus和Pangolin的INCLUDE_DIRS都有包含Eigen3，故这里可以注释
# find_package(Eigen3 3.1.0 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin)
find_package(OpenCV REQUIRED)

include_directories(
#    ${EIGEN3_INCLUDE_DIR}  
    ${Sophus_INCLUDE_DIRS}  
    ${Pangolin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)


add_executable(draw_trajectory draw_trajectory.cpp)
target_link_libraries(draw_trajectory
    ${Pangolin_LIBRARIES}
    ${Sophus_LIBRARIES}
)

add_executable(rmse rmse.cpp)
target_link_libraries(rmse
    ${Sophus_LIBRARIES}
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
)
