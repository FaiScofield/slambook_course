cmake_minimum_required(VERSION 2.8)
project(homework_07)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-O3" )
set(CMAKE_CXX_FLAGS "-std=c++11")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

set(OpenCV_DIR "/home/vance/softwares/opencv-3.3.1/build")
find_package(OpenCV REQUIRED)
find_package(Pangolin)
find_package(Sophus REQUIRED)
Find_Package(G2O REQUIRED)
Find_Package(Cholmod REQUIRED)
find_package(CSparse REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/include/
    ${Sophus_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIR}
    ${CHOLMOD_INCLUDE_DIR}
    ${CSPARSE_INCLUDE_DIR}
)

SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear
    g2o_types_icp g2o_types_slam2d g2o_types_sba g2o_types_slam3d g2o_core g2o_interface
    g2o_solver_csparse g2o_solver_structure_only g2o_csparse_extension g2o_opengl_helper
    g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data
    g2o_types_sim3 cxsparse )

add_library(BALProblem SHARED ${PROJECT_SOURCE_DIR}/src/BALProblem.cpp)
add_library(ParseCmd SHARED ${PROJECT_SOURCE_DIR}/src/command_args.cpp)

add_executable(g2o_bundle src/g2o_bundle.cpp)
target_link_libraries(g2o_bundle ${Sophus_LIBRARIES} ${G2O_LIBS} ${CHOLMOD_LIBRARIES} BALProblem ParseCmd)

add_executable(directBA src/directBA.cpp)
target_link_libraries(directBA ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES} ${CSPARSE_LIBRARY} ${G2O_LIBS} )

#add_executable(ss-g2o_bundle src/ss-g2o_bundle.cpp)
#target_link_libraries(ss-g2o_bundle ${Sophus_LIBRARIES} ${G2O_LIBS} ${CHOLMOD_LIBRARIES} BALProblem ParseCmd)

