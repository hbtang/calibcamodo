cmake_minimum_required(VERSION 2.8.3)
project(calibcamodo)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules/)

# c++ 11 support for g2o, multi-thread and shared_ptr
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall")

# Find third party packages
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CSparse REQUIRED)
find_package(Cholmod REQUIRED)
find_package(G2O REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
    roscpp
    rospy
    tf
    roslib
    cv_bridge
    cmake_modules
    image_transport
)

include_directories(
    src/
    src/aruco
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CSPARSE_INCLUDE_DIR}
    ${Cholmod_INCLUDE_DIR}
    ${G2O_INCLUDE_DIR}
)

list(APPEND LINK_LIBS
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

list(APPEND G2O_LIBS
    cxsparse
    cholmod
    g2o_cli g2o_ext_freeglut_minimal g2o_simulator
    g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d
    g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only
    g2o_types_sba g2o_types_slam3d g2o_csparse_extension
    g2o_opengl_helper g2o_solver_dense g2o_stuff
    g2o_types_sclam2d g2o_parser g2o_solver_pcg
    g2o_types_data g2o_types_sim3
)

## Declare a C++ executable
# file(GLOB_RECURSE FILE_INCLUDE "include/*.h")
aux_source_directory(./src SRC_DIR)
aux_source_directory(./src/aruco SRC_DIR_ARUCO)
add_executable(calibcamodo ${SRC_DIR} ${SRC_DIR_ARUCO})

# message("SRC_DIR: ${SRC_DIR}")
# message("SRC_DIR_ARUCO: ${SRC_DIR_ARUCO}")
# message("FILE_INCLUDE: ${FILE_INCLUDE}")

## Specify libraries to link a library or executable target against
target_link_libraries(calibcamodo
    ${LINK_LIBS}
    ${G2O_LIBS}
)
